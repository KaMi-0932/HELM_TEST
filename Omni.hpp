#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 全向轮底盘控制实现，由 Chassis 模板使用 / Omnidirectional-wheel chassis controller used by the Chassis template
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "CMD.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"

#define M3508_NM_TO_LSB_RATIO 52437.5f /* 3508 转子扭矩到电机控制值的比例 */

#define OMNI_MOTOR_MAX_OMEGA 52 /* 全向轮输出轴最大角速度 rad/s */

#define OMNI_CHASSIS_MAX_POWER 100 /* 裁判系统离线时的默认功率上限 W */

template <typename ChassisType>
class Chassis;

/**
 * @brief 全向轮底盘控制实现，由 Chassis 模板使用。
 *        Omnidirectional-wheel chassis controller used by the Chassis template.
 */
class Omni
{
 public:
  /**
   * @brief 底盘几何、动力与小陀螺缩放参数。
   *        Chassis geometry, dynamics and spin-scaling parameters.
   */
  struct ChassisParam
  {
    float wheel_radius = 0.0f;  ///< 轮半径 (m)
    ///< Wheel radius (m)
    float wheel_to_center = 0.0f;  ///< 轮心到底盘中心的距离 (m)
    ///< Distance from the wheel center to the chassis center (m)
    float gravity_height = 0.0f;  ///< 质心高度 (m)，用于全向轮姿态前馈
    ///< Center-of-mass height (m), used by the Omni attitude feedforward
    float reduction_ratio = 0.0f;  ///< 轮电机减速比
    ///< Reduction ratio of the wheel motors
    float wheel_resistance = 0.0f;  ///< 轮阻
    ///< Wheel resistance
    float error_compensation = 0.0f;  ///< 误差补偿
    ///< Error compensation
    float gravity = 0.0f;  ///< 底盘重力 (N)，用于全向轮姿态前馈
    ///< Chassis weight (N), used by the Omni attitude feedforward
    float length = 0.0f;  ///< 轮距的长 (m)，用于全向轮姿态前馈
    ///< Length of the wheel layout (m), used by the Omni attitude feedforward
    float width = 0.0f;  ///< 轮距的宽 (m)，用于全向轮姿态前馈
    ///< Width of the wheel layout (m), used by the Omni attitude feedforward
    float rotor_speed_scale = 1.0f;  ///< 平移输入下的小陀螺转速缩放比例
    ///< Spin speed scale under translation input
    float rotor_omega_min_scale = 0.55f;  ///< 小陀螺动态缩放的下限
    ///< Lower bound of the dynamic spin scale
    float rotor_buffer_low_j = 35.0f;  ///< 缓冲能量低阈值 (J)
    ///< Low buffer energy threshold (J)
    float rotor_buffer_high_j = 70.0f;  ///< 缓冲能量高阈值 (J)
    ///< High buffer energy threshold (J)
    float rotor_scale_lpf_alpha = 0.2f;  ///< 动态缩放的一阶低通系数
    ///< First-order low-pass coefficient of the dynamic scale
  };
  /**
   * @brief 底盘模式。
   *        Chassis modes.
   */
  enum class ChassisMode : uint8_t
  {
    RELAX,  ///< 放松：全部电机放松
    ///< Relax: all motors relaxed
    INDEPENDENT,  ///< 独立：平移相对底盘坐标系
    ///< Independent: translation in the chassis frame
    ROTOR,  ///< 小陀螺：底盘旋转，平移相对云台坐标系
    ///< Spin: the chassis rotates, translation in the gimbal frame
    FOLLOW  ///< 跟随：底盘跟随云台 yaw，平移相对云台坐标系
    ///< Follow: the chassis follows the gimbal yaw, translation in the gimbal frame
  };

  /**
   * @brief 构造全向轮底盘控制对象，创建控制线程。
   *        Construct the omnidirectional-wheel chassis controller and create the control
   *        thread.
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   *                      Drive wheel motor 0.
   * @param motor_wheel_1 第 1 个驱动轮电机。
   *                      Drive wheel motor 1.
   * @param motor_wheel_2 第 2 个驱动轮电机。
   *                      Drive wheel motor 2.
   * @param motor_wheel_3 第 3 个驱动轮电机。
   *                      Drive wheel motor 3.
   * @param motor_steer_0 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
   * @param motor_steer_1 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
   * @param motor_steer_2 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
   * @param motor_steer_3 与其他底盘类型共用的构造参数。
   *                      Constructor parameter shared with the other chassis types.
   * @param cmd 控制命令模块实例。
   *            Control command Module instance.
   * @param power_control 功率控制模块实例。
   *                      Power control Module instance.
   * @param referee 裁判系统模块实例。
   *                Referee Module instance.
   * @param task_stack_depth 控制线程栈深。
   *                         Control thread stack depth.
   * @param chassis_param 底盘几何与动力参数。
   *                      Chassis geometry and dynamics parameters.
   * @param pid_follow 跟随云台的角度 PID 参数。
   *                   PID parameters of the gimbal-following angle loop.
   * @param pid_velocity_x x 方向速度 PID 参数。
   *                       PID parameters of the x velocity loop.
   * @param pid_velocity_y y 方向速度 PID 参数。
   *                       PID parameters of the y velocity loop.
   * @param pid_omega 角速度 PID 参数。
   *                  PID parameters of the angular-velocity loop.
   * @param pid_wheel_speed_0 轮 0 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 0.
   * @param pid_wheel_speed_1 轮 1 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 1.
   * @param pid_wheel_speed_2 轮 2 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 2.
   * @param pid_wheel_speed_3 轮 3 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 3.
   * @param pid_steer_angle_0 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_angle_1 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_angle_2 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_angle_3 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_0 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_1 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_2 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param pid_steer_speed_3 与其他底盘类型共用的构造参数。
   *                          Constructor parameter shared with the other chassis types.
   * @param thread_priority 控制线程优先级。
   *                        Control thread priority.
   * @param topic_names 订阅的 Topic 名称。
   *                    Names of the subscribed Topics.
   */
  Omni(Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
       Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
       Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd, PowerControl* power_control,
       Referee* referee, uint32_t task_stack_depth, ChassisParam chassis_param,
       LibXR::PID<float>::Param pid_follow, LibXR::PID<float>::Param pid_velocity_x,
       LibXR::PID<float>::Param pid_velocity_y, LibXR::PID<float>::Param pid_omega,
       LibXR::PID<float>::Param pid_wheel_speed_0,
       LibXR::PID<float>::Param pid_wheel_speed_1,
       LibXR::PID<float>::Param pid_wheel_speed_2,
       LibXR::PID<float>::Param pid_wheel_speed_3,
       LibXR::PID<float>::Param pid_steer_angle_0,
       LibXR::PID<float>::Param pid_steer_angle_1,
       LibXR::PID<float>::Param pid_steer_angle_2,
       LibXR::PID<float>::Param pid_steer_angle_3,
       LibXR::PID<float>::Param pid_steer_speed_0,
       LibXR::PID<float>::Param pid_steer_speed_1,
       LibXR::PID<float>::Param pid_steer_speed_2,
       LibXR::PID<float>::Param pid_steer_speed_3,
       LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH,
       ChassisTopicNames topic_names = {})
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),   /* wheel0   ▲ y  wheel3 */
        motor_wheel_1_(motor_wheel_1),   /*     ↙    │     ↖     */
        motor_wheel_2_(motor_wheel_2),   /*          │           */
        motor_wheel_3_(motor_wheel_3),   /* ─────────┼────────▶x */
        pid_follow_(pid_follow),         /*          │           */
        pid_velocity_x_(pid_velocity_x), /*     ↘    │     ↗     */
        pid_velocity_y_(pid_velocity_y), /* wheel1   │    wheel2 */
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1, pid_wheel_speed_2,
                         pid_wheel_speed_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1, pid_steer_angle_2,
                         pid_steer_angle_3},
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1, pid_steer_speed_2,
                         pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control),
        referee_(referee)
  {
    topic_names_ = topic_names;
    UNUSED(motor_steer_0);
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    UNUSED(pid_steer_speed_0);
    UNUSED(pid_steer_speed_1);
    UNUSED(pid_steer_speed_2);
    UNUSED(pid_steer_speed_3);
    UNUSED(pid_steer_angle_0);
    UNUSED(pid_steer_angle_1);
    UNUSED(pid_steer_angle_2);
    UNUSED(pid_steer_angle_3);

    for (int i = 0; i < 4; i++)
    {
      motor_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_cmd_[i].torque = 0.0f;
      motor_cmd_[i].position = 0.0f;
      motor_cmd_[i].velocity = 0.0f;
      motor_cmd_[i].kp = 0.0f;
      motor_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "OmniChassisThread", task_stack_depth,
                   thread_priority);
    if (referee_ != nullptr)
    {
      /* 底盘裁判系统 UI 由定时器任务周期刷新 */
      timer_ui_ = LibXR::Timer::CreateTask(DrawUI, this, UI_REFRESH_PERIOD_MS);
      LibXR::Timer::Add(timer_ui_);
      LibXR::Timer::Start(timer_ui_);
    }

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Omni* omni, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          omni->mutex_.Lock();
          omni->chassis_event_ = ChassisMode::RELAX;
          ResetModeUILocked(omni);
          omni->mutex_.Unlock();
        },
        this);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Omni* omni, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          omni->mutex_.Lock();
          omni->chassis_event_ = ChassisMode::RELAX;
          ResetModeUILocked(omni);
          omni->mutex_.Unlock();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 全向轮底盘控制线程函数。
   *        Control thread function of the omnidirectional-wheel chassis.
   *
   * @details 线程主循环：接收控制命令，执行运动学解算并输出控制量。
   *          Main loop: receives the control command, solves the kinematics and sends the
   *          outputs.
   *
   * @param omni Omni 对象指针。
   *             Pointer to the Omni object.
   */
  static void ThreadFunction(Omni* omni)
  {
    omni->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber(omni->topic_names_.chassis_cmd);
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        omni->topic_names_.chassis_ref);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        omni->topic_names_.gimbal_euler);
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");
    LibXR::Topic::ASyncSubscriber<float> pitchmotor_angle_suber("pitchmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    euler_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();
    pitchmotor_angle_suber.StartWaiting();
    omni->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();
    omni->mutex_.Unlock();

    while (true)
    {
      if (cmd_suber.Available())
      {
        omni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available())
      {
        omni->referee_chassis_pack_ = referee_suber.GetData();
        omni->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (euler_suber.Available())
      {
        omni->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
        omni->imu_pitch_ = -(omni->euler_.Pitch());
        omni->imu_roll_ = -(omni->euler_.Roll());
        omni->imu_yaw_ = (omni->euler_.Yaw());
      }

      if (yawmotor_angle_suber.Available())
      {
        omni->yawmotor_angle_ = yawmotor_angle_suber.GetData();
        yawmotor_angle_suber.StartWaiting();
      }
      if (pitchmotor_angle_suber.Available())
      {
        omni->pitchmotor_angle_ = pitchmotor_angle_suber.GetData();
        pitchmotor_angle_suber.StartWaiting();
      }
      omni->mutex_.Lock();
      omni->Update();
      omni->UpdateCMD();
      omni->SelfResolution();
      omni->InverseKinematicsSolution();
      omni->DynamicInverseSolution();
      omni->FeedForward();
      omni->CalculateMotorCurrent();
      omni->PowerControlUpdate();
      omni->mutex_.Unlock();
      omni->OutputToDynamics();

      omni->thread_.SleepUntil(last_time, 2);
    }
  }

  /**
   * @brief 更新电机状态。
   *        Update the motor states.
   *
   * @details 记录采样间隔，更新四个驱动轮电机并读取反馈。
   *          Records the sampling interval, updates the four wheel motors and reads their
   *          feedback.
   */
  void Update()
  {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    for (int i = 0; i < 4; i++)
    {
      motor_wheel_[i]->Update();
      motor_feedback_[i] = motor_wheel_[i]->GetFeedback();
    }
  }

  /**
   * @brief 设置底盘模式并复位速度与轮速 PID，由 Chassis 外壳调用。
   *        Set the chassis mode and reset the velocity and wheel-speed PIDs; called by
   *        the Chassis shell.
   *
   * @param mode 新模式，取 ChassisMode 的值。
   *             New mode, a value of ChassisMode.
   */
  void SetMode(uint32_t mode)
  {
    mutex_.Lock();
    const ChassisMode NEXT_MODE = static_cast<ChassisMode>(mode);
    chassis_event_ = NEXT_MODE;
    ui_text_initialized_ = false;
    ui_refresh_tick_ = UI_MODE_TEXT_TICK;
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; i++)
    {
      pid_wheel_speed_[i].Reset();
    }
    mutex_.Unlock();
  }

  /**
   * @brief 从 CMD 命令更新目标角速度与目标平移速度。
   *        Update the target angular velocity and the target translation velocity from
   *        the CMD command.
   */
  void UpdateCMD()
  {
    float max_v = PARAM.wheel_radius * OMNI_MOTOR_MAX_OMEGA;

    /* 先生成目标角速度 */
    switch (chassis_event_)
    {
      case ChassisMode::RELAX:
        target_omega_ = 0.0f;
        break;

      case ChassisMode::INDEPENDENT:
        target_omega_ = max_v * cmd_data_.z / PARAM.wheel_to_center;
        break;

      case ChassisMode::ROTOR:
        target_omega_ = -static_cast<float>(max_v / PARAM.wheel_to_center);
        break;

      case ChassisMode::FOLLOW:
        target_omega_ = -pid_follow_.Calculate(0.0f, yawmotor_angle_, dt_);
        break;

      default:
        break;
    }

    /* 再生成目标平移速度 */
    switch (chassis_event_)
    {
      case ChassisMode::RELAX:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case ChassisMode::ROTOR:
      case ChassisMode::FOLLOW:
      {
        float beta = yawmotor_angle_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ = (cos_beta * cmd_data_.x * max_v - sin_beta * cmd_data_.y * max_v);
        target_vy_ = (sin_beta * cmd_data_.x * max_v + cos_beta * cmd_data_.y * max_v);
      }
      break;
      case ChassisMode::INDEPENDENT:
      {
        const float SQRT2 = 1.41421356237f;
        /* 独立模式用菱形限幅适配摇杆边界 */
        float s = fabsf(cmd_data_.x) + fabsf(cmd_data_.y);
        float k = (s <= 1.0f) ? max_v : (max_v / s);
        target_vx_ = SQRT2 * k * cmd_data_.x;
        target_vy_ = SQRT2 * k * cmd_data_.y;
      }
      break;
      default:
        break;
    }

    /* 小陀螺模式按平移输入和功率状态动态压低转速 */
    float rotor_translation_scale = 1.0f;
    if (chassis_event_ == ChassisMode::ROTOR)
    {
      float translation_magnitude =
          sqrtf(target_vx_ * target_vx_ + target_vy_ * target_vy_);
      float translation_ratio = 0.0f;
      if (max_v > 1e-3f)
      {
        translation_ratio = std::clamp(translation_magnitude / max_v, 0.0f, 1.0f);
      }
      rotor_translation_scale =
          1.0f - (1.0f - PARAM.rotor_speed_scale) * translation_ratio;
      target_omega_ *= rotor_translation_scale * rotor_dynamic_scale_;
    }
  }

  /**
   * @brief 带死区的软限幅，用于姿态前馈。
   *        Soft dead-zone used by the attitude feedforward.
   *
   * @param x 输入值。
   *          Input value.
   * @param dz 死区范围。
   *           Dead-zone width.
   * @return 死区外的输出值。
   *         Output outside the dead zone.
   */
  float SoftDeadzone(float x, float dz)
  {
    if (fabs(x) < dz)
    {
      return 0.0f;
    }
    else
    {
      return (fabs(x) - dz) * (x > 0.0f ? 1.0f : -1.0f);
    }
  }

  /**
   * @brief 由云台姿态与云台电机角度计算底盘姿态前馈力矩。
   *        Compute the attitude feedforward torques from the gimbal attitude and the
   *        gimbal motor angles.
   */
  void FeedForward()
  {
    /* 角度先包到正负 pi */
    auto WrapToPi = [](float a)
    {
      while (a > M_PI) a -= 2.0f * M_PI;
      while (a < -M_PI) a += 2.0f * M_PI;
      return a;
    };

    float yaw_g = WrapToPi(imu_yaw_);
    float pitch_g = WrapToPi(imu_pitch_);
    float roll_g = WrapToPi(imu_roll_);

    float yaw_m = WrapToPi(yawmotor_angle_);
    float pitch_m = WrapToPi(pitchmotor_angle_);

    /* 预计算姿态三角函数 */
    float cy = cosf(yaw_g), sy = sinf(yaw_g);
    float cp = cosf(pitch_g), sp = sinf(pitch_g);
    float cr = cosf(roll_g), sr = sinf(roll_g);

    float cy_m = cosf(yaw_m), sy_m = sinf(yaw_m);
    float cp_m = cosf(pitch_m), sp_m = sinf(pitch_m);

    /* 世界系到云台系的旋转矩阵 */
    float R_wg[3][3];

    post_x_[0] = -PARAM.width / 2;
    post_x_[1] = -PARAM.width / 2;
    post_x_[2] = PARAM.width / 2;
    post_x_[3] = PARAM.width / 2;

    post_y_[0] = PARAM.length / 2;
    post_y_[1] = -PARAM.length / 2;
    post_y_[2] = -PARAM.length / 2;
    post_y_[3] = PARAM.length / 2;

    R_wg[0][0] = cy * cp;
    R_wg[0][1] = cy * sp * sr - sy * cr;
    R_wg[0][2] = cy * sp * cr + sy * sr;

    R_wg[1][0] = sy * cp;
    R_wg[1][1] = sy * sp * sr + cy * cr;
    R_wg[1][2] = sy * sp * cr - cy * sr;

    R_wg[2][0] = -sp;
    R_wg[2][1] = cp * sr;
    R_wg[2][2] = cp * cr;

    /* 底盘系到云台系的旋转矩阵 */
    float R_cg[3][3];

    R_cg[0][0] = cy_m * cp_m;
    R_cg[0][1] = -sy_m;
    R_cg[0][2] = cy_m * sp_m;

    R_cg[1][0] = sy_m * cp_m;
    R_cg[1][1] = cy_m;
    R_cg[1][2] = sy_m * sp_m;

    R_cg[2][0] = -sp_m;
    R_cg[2][1] = 0.0f;
    R_cg[2][2] = cp_m;

    /* 转置得到云台系到底盘系的旋转矩阵 */
    float R_gc[3][3];
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        R_gc[i][j] = R_cg[j][i];
      }
    }

    /* 合成世界系到底盘系的旋转矩阵 */
    float R_wc[3][3] = {{0}};

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        R_wc[i][j] =
            R_wg[i][0] * R_gc[0][j] + R_wg[i][1] * R_gc[1][j] + R_wg[i][2] * R_gc[2][j];
      }
    }

    /* 提取底盘俯仰和横滚用于前馈 */
    float val = -R_wc[2][0];
    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;

    chassis_pitch_ = asinf(val);
    chassis_roll_ = atan2f(R_wc[2][1], R_wc[2][2]);

    chassis_pitch_ = WrapToPi(chassis_pitch_);
    chassis_roll_ = WrapToPi(chassis_roll_);

    float k = M_PI / 50;
    gy_ff_ = -PARAM.gravity * SoftDeadzone(sinf(chassis_pitch_), sinf(k));
    gx_ff_ = -PARAM.gravity * SoftDeadzone(sinf(chassis_roll_), sinf(k));

    py = -PARAM.gravity_height * SoftDeadzone(sinf(chassis_pitch_), sin(k));
    px = -PARAM.gravity_height * SoftDeadzone(sinf(chassis_roll_), sin(k));

    for (int i = 0; i < 4; i++)
    {
      float dx = post_x_[i] - px;
      float dy = post_y_[i] - py;
      length_[i] = sqrtf(dx * dx + dy * dy);
    }

    const float SQRT2 = 0.70710678118f * 2;
    baseff_[0] = (gx_ff_ + gy_ff_) * SQRT2;
    baseff_[1] = (-gx_ff_ + gy_ff_) * SQRT2;
    baseff_[2] = (-gx_ff_ - gy_ff_) * SQRT2;
    baseff_[3] = (gx_ff_ - gy_ff_) * SQRT2;

    for (int i = 0; i < 4; i++)
    {
      baseff_l_[i] = 1.0f / length_[i];
      torque_n_[i] =
          baseff_l_[i] / (baseff_l_[0] + baseff_l_[1] + baseff_l_[2] + baseff_l_[3]);
      baseff_[i] = baseff_[i] * torque_n_[i];
    }

    for (int i = 0; i < 4; i++)
    {
      torque_ff_[i] = baseff_[i] * PARAM.wheel_radius;
    }
  }

  /**
   * @brief 全向轮底盘正运动学解算。
   *        Forward kinematics of the omnidirectional-wheel chassis.
   *
   * @details 由四个全向轮的角速度解算底盘的运动状态。
   *          Solves the chassis motion from the angular velocities of the four wheels.
   */
  void SelfResolution()
  {
    const float SQRT2 = 1.41421356237f;

    now_vx_ = -(motor_feedback_[0].omega / PARAM.reduction_ratio -
                motor_feedback_[1].omega / PARAM.reduction_ratio -
                motor_feedback_[2].omega / PARAM.reduction_ratio +
                motor_feedback_[3].omega / PARAM.reduction_ratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_vy_ = -(motor_feedback_[0].omega / PARAM.reduction_ratio +
                motor_feedback_[1].omega / PARAM.reduction_ratio -
                motor_feedback_[2].omega / PARAM.reduction_ratio -
                motor_feedback_[3].omega / PARAM.reduction_ratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
                  motor_feedback_[1].omega / PARAM.reduction_ratio +
                  motor_feedback_[2].omega / PARAM.reduction_ratio +
                  motor_feedback_[3].omega / PARAM.reduction_ratio) *
                 PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 全向轮底盘逆运动学解算。
   *        Inverse kinematics of the omnidirectional-wheel chassis.
   *
   * @details 由目标底盘速度 (vx, vy, ω) 计算四个全向轮的目标角速度。
   *          Computes the target angular velocity of the four wheels from the target
   *          chassis velocity (vx, vy, ω).
   */
  void InverseKinematicsSolution()
  {
    const float SQRT1 = 0.70710678118f;

    target_motor_omega_[0] = (-SQRT1 * target_vx_ - SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[1] = (SQRT1 * target_vx_ - SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[2] = (SQRT1 * target_vx_ + SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[3] = (-SQRT1 * target_vx_ + SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
  }

  /**
   * @brief 计算轮速 PID 输出，并与前馈力矩及姿态前馈合成。
   *        Compute the wheel-speed PID outputs and combine them with the feedforward
   *        torque and the attitude feedforward.
   */
  void CalculateMotorCurrent()
  {
    if (chassis_event_ == ChassisMode::RELAX)
    {
      LostCtrl();
    }
    else
    {
      /* 轮速 PID 输出 */
      for (int i = 0; i < 4; i++)
      {
        target_motor_current_[i] = pid_wheel_speed_[i].Calculate(
            target_motor_omega_[i], motor_feedback_[i].omega / PARAM.reduction_ratio,
            dt_);
      }

      /* 合成动力学输出和上坡前馈 */
      for (int i = 0; i < 4; i++)
      {
        output_[i] = target_motor_force_[i] * PARAM.wheel_radius +
                     target_motor_current_[i] + torque_ff_[i];
      }
    }
  }

  /**
   * @brief 更新功率控制与小陀螺动态缩放。
   *        Update power control and the dynamic spin scale.
   *
   * @details 提交反馈与期望输出，计算功率上限并读取限幅结果。
   *          Submits the feedback and the requested outputs, computes the power limit and
   *          reads the limited result.
   */
  void PowerControlUpdate()
  {
    for (int i = 0; i < 4; i++)
    {
      motor_data_.rotorspeed_rpm_3508[i] = motor_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);
    power_control_->CalculatePowerControlParam();

    float speed_error[4];
    for (int i = 0; i < 4; i++)
    {
      speed_error[i] =
          target_motor_omega_[i] - motor_feedback_[i].omega / PARAM.reduction_ratio;
      motor_data_.output_current_3508[i] =
          std::clamp(output_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508, speed_error);

    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    float max_power = static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f)
    {
      max_power = OMNI_CHASSIS_MAX_POWER;
    }

    if (power_control_online && boost_mode)
    {
      float cap_energy = power_control_->GetCapEnergy();
      if (cap_energy > 0.8f)
      {
        max_power += 300.0f;
      }
      else if (cap_energy > 0.5f)
      {
        max_power += 200.0f;
      }
      else if (cap_energy > 0.25f)
      {
        max_power += 100.0f;
      }
    }

    power_control_->OutputLimit(max_power);
    power_control_data_ = power_control_->GetPowerControlData();

    float req_current_abs_sum = 0.0f;
    float lim_current_abs_sum = 0.0f;
    for (int i = 0; i < 4; i++)
    {
      float req_current_abs = fabsf(motor_data_.output_current_3508[i]);
      float lim_current_abs = power_control_data_.is_power_limited
                                  ? fabsf(power_control_data_.new_output_current_3508[i])
                                  : req_current_abs;
      req_current_abs_sum += req_current_abs;
      lim_current_abs_sum += lim_current_abs;
    }

    float power_limit_ratio = 1.0f;
    if (req_current_abs_sum > 1e-3f)
    {
      power_limit_ratio =
          std::clamp(lim_current_abs_sum / req_current_abs_sum, 0.0f, 1.0f);
    }

    float buffer_scale = 1.0f;
    if (referee_online)
    {
      float buffer_range =
          std::max(PARAM.rotor_buffer_high_j - PARAM.rotor_buffer_low_j, 1.0f);
      float referee_buffer_j = static_cast<float>(referee_chassis_pack_.power_buffer);
      float buffer_norm = std::clamp(
          (referee_buffer_j - PARAM.rotor_buffer_low_j) / buffer_range, 0.0f, 1.0f);
      buffer_scale = PARAM.rotor_omega_min_scale +
                     (1.0f - PARAM.rotor_omega_min_scale) * buffer_norm;
    }

    float limit_scale =
        power_control_data_.is_power_limited
            ? std::clamp(power_limit_ratio, PARAM.rotor_omega_min_scale, 1.0f)
            : 1.0f;
    float target_dynamic_scale =
        std::clamp(buffer_scale * limit_scale, PARAM.rotor_omega_min_scale, 1.0f);
    float lpf_alpha = std::clamp(PARAM.rotor_scale_lpf_alpha, 0.0f, 1.0f);
    rotor_dynamic_scale_ += (target_dynamic_scale - rotor_dynamic_scale_) * lpf_alpha;
    rotor_dynamic_scale_ =
        std::clamp(rotor_dynamic_scale_, PARAM.rotor_omega_min_scale, 1.0f);

    if (chassis_event_ != ChassisMode::ROTOR)
    {
      rotor_dynamic_scale_ = 1.0f;
    }
  }

  /**
   * @brief 全向轮底盘逆动力学解算。
   *        Inverse dynamics of the omnidirectional-wheel chassis.
   *
   * @details 以运动学正解得到的当前状态与目标状态做 PID，得到各轮的前馈力。
   *          Runs PIDs between the state from the forward kinematics and the target state
   *          to obtain the feedforward force of each wheel.
   */
  void DynamicInverseSolution()
  {
    const float SQRT2 = 1.41421356237f;

    float force_x = pid_velocity_x_.Calculate(target_vx_, now_vx_, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, now_vy_, dt_);
    float force_z = pid_omega_.Calculate(target_omega_, now_omega_, dt_);

    /* 按全向轮受力方向分配前馈力 */
    target_motor_force_[0] = (-SQRT2 * force_x - SQRT2 * force_y + force_z) / 4;
    target_motor_force_[1] = (SQRT2 * force_x - SQRT2 * force_y + force_z) / 4;
    target_motor_force_[2] = (SQRT2 * force_x + SQRT2 * force_y + force_z) / 4;
    target_motor_force_[3] = (-SQRT2 * force_x + SQRT2 * force_y + force_z) / 4;
  }

  /**
   * @brief 全向轮底盘动力学输出。
   *        Dynamics output of the omnidirectional-wheel chassis.
   *
   * @details 限幅后向四个全向轮下发力矩指令。
   *          Limits the torques and sends them to the four wheels.
   */
  void OutputToDynamics()
  {
    /* 功率受限时使用限幅后的电流反算输出扭矩 */
    if (power_control_data_.is_power_limited)
    {
      for (int i = 0; i < 4; i++)
      {
        output_[i] = std::clamp(power_control_data_.new_output_current_3508[i] /
                                    M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                                -6.0f, 6.0f);
      }
    }
    if (chassis_event_ == ChassisMode::RELAX)
    {
      LostCtrl();
      return;
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        motor_cmd_[i].torque = std::clamp(output_[i], -6.0f, 6.0f);
      }
      for (int i = 0; i < 4; i++)
      {
        motor_wheel_[i]->Control(motor_cmd_[i]);
      }
    }
  }
  /**
   * @brief 失去控制时使全部电机放松。
   *        Relax all motors when control is lost.
   */
  void LostCtrl()
  {
    for (int i = 0; i < 4; i++)
    {
      motor_wheel_[i]->Relax();
    }
  }

 private:
  /* 底盘 UI 图层, 模式文字、中间外框、电容框和电容条共用 */
  static constexpr uint8_t UI_LAYER_CHASSIS = 3;
  /* 底盘 UI 文字共用线宽和字号 */
  static constexpr uint16_t UI_CHAR_WIDTH = 2;
  static constexpr uint16_t UI_FONT_SIZE = 20;
  /* 左下区域底盘模式文字位置 */
  static constexpr uint16_t UI_MODE_TEXT_X = 160;
  static constexpr uint16_t UI_MODE_TEXT_Y = 725;
  /* 左下区域 AI 模式文字位置 */
  static constexpr uint16_t UI_AI_TEXT_X = 160;
  static constexpr uint16_t UI_AI_TEXT_Y = 675;
  /* 左侧电容外框位置和尺寸 */
  static constexpr uint16_t UI_CAP_BOX_X1 = 160;
  static constexpr uint16_t UI_CAP_BOX_Y1 = 612;
  static constexpr uint16_t UI_CAP_BOX_X2 = 320;
  static constexpr uint16_t UI_CAP_BOX_Y2 = 640;
  static constexpr uint16_t UI_CAP_BOX_WIDTH = 2;
  /* 电容能量填充条内边距和线宽 */
  static constexpr uint16_t UI_CAP_FILL_MARGIN = 4;
  static constexpr uint16_t UI_CAP_FILL_WIDTH = 8;
  /* 中间外框的位置和尺寸 */
  static constexpr uint16_t UI_STATUS_BOX_X1 = 780;
  static constexpr uint16_t UI_STATUS_BOX_Y1 = 360;
  static constexpr uint16_t UI_STATUS_BOX_X2 = 1200;
  static constexpr uint16_t UI_STATUS_BOX_Y2 = 760;
  static constexpr uint16_t UI_STATUS_BOX_WIDTH = 4;
  /* 中间外框左右两侧引导斜线的线宽与端点坐标 */
  static constexpr uint16_t UI_STATUS_GUIDE_WIDTH = 4;
  static constexpr uint16_t UI_STATUS_GUIDE_LEFT_X1 = 493;
  static constexpr uint16_t UI_STATUS_GUIDE_LEFT_Y1 = 199;
  static constexpr uint16_t UI_STATUS_GUIDE_LEFT_X2 = 913;
  static constexpr uint16_t UI_STATUS_GUIDE_LEFT_Y2 = 471;
  static constexpr uint16_t UI_STATUS_GUIDE_RIGHT_X1 = 1427;
  static constexpr uint16_t UI_STATUS_GUIDE_RIGHT_Y1 = 199;
  static constexpr uint16_t UI_STATUS_GUIDE_RIGHT_X2 = 1007;
  static constexpr uint16_t UI_STATUS_GUIDE_RIGHT_Y2 = 471;
  /* 底盘 UI 刷新周期和分时重发节奏 */
  static constexpr uint32_t UI_REFRESH_PERIOD_MS = 100;
  static constexpr uint32_t UI_BOX_RESEND_DIV = 10;
  static constexpr uint32_t UI_GUIDE_RESEND_OFFSET = 1;
  static constexpr uint32_t UI_MODE_TEXT_TICK = 3;
  static constexpr uint32_t UI_AI_TEXT_TICK = 0;
  static constexpr uint32_t UI_TEXT_READD_DIV = 10;
  static constexpr uint32_t UI_AI_TEXT_REFRESH_DIV = 3;
  static constexpr uint32_t UI_AI_TEXT_READD_DIV = 37;
  static constexpr uint32_t UI_CAP_FILL_REFRESH_DIV = 3;
  static constexpr uint32_t UI_CAP_FILL_REFRESH_OFFSET = 2;
  /* 电容条的低频重建周期，周期性发送 ADD 使客户端丢失图形后恢复 */
  static constexpr uint32_t UI_CAP_FILL_READD_DIV = 50;

  static void ResetModeUILocked(Omni* omni)
  {
    omni->ui_text_initialized_ = false;
    omni->ui_frame_initialized_ = false;
    omni->ui_guide_initialized_ = false;
    omni->ui_cap_fill_initialized_ = false;
    omni->ui_ai_text_initialized_ = false;
    omni->ui_layer_cleared_ = false;
    omni->ui_refresh_tick_ = 0;
  }

  static const char* GetModeText(ChassisMode mode)
  {
    switch (mode)
    {
      case ChassisMode::RELAX:
        return "RELX";
      case ChassisMode::INDEPENDENT:
        return "INDP";
      case ChassisMode::ROTOR:
        return "ROTO";
      case ChassisMode::FOLLOW:
        return "FOLW";
      default:
        return "UNKN";
    }
  }

  static void FormatModeText(char (&text)[16], ChassisMode mode)
  {
    std::snprintf(text, sizeof(text), "%s", GetModeText(mode));
  }

  static const char* GetAIModeText(bool enabled) { return enabled ? "AI ON" : "AI OFF"; }

  static Referee::UIColor GetAIModeColor(bool enabled)
  {
    return enabled ? Referee::UIColor::UI_COLOR_YELLOW
                   : Referee::UIColor::UI_COLOR_ORANGE;
  }

  static void DrawUI(Omni* omni)
  {
    if (omni->referee_ == nullptr)
    {
      return;
    }
    const uint16_t ROBOT_ID = omni->referee_->GetRobotID();
    if (ROBOT_ID == 0)
    {
      return;
    }
    const uint16_t CLIENT_ID = omni->referee_->GetClientID(ROBOT_ID);

    omni->mutex_.Lock();
    const ChassisMode MODE = omni->chassis_event_;
    const bool UI_TEXT_INITIALIZED = omni->ui_text_initialized_;
    const bool UI_FRAME_INITIALIZED = omni->ui_frame_initialized_;
    const bool UI_GUIDE_INITIALIZED = omni->ui_guide_initialized_;
    const bool UI_CAP_FILL_INITIALIZED = omni->ui_cap_fill_initialized_;
    const bool UI_AI_TEXT_INITIALIZED = omni->ui_ai_text_initialized_;
    const bool UI_LAYER_CLEARED = omni->ui_layer_cleared_;
    const uint32_t UI_TICK = omni->ui_refresh_tick_++;
    const bool AI_MODE_ENABLED =
        omni->cmd_ != nullptr && omni->cmd_->GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL;
    const bool CAP_ONLINE =
        omni->power_control_ != nullptr && omni->power_control_->IsOnline();
    const float CAP_ENERGY =
        omni->power_control_ != nullptr ? omni->power_control_->GetCapEnergy() : 0.0f;
    omni->mutex_.Unlock();
    Referee::UILayerDelete ui_del{};
    ui_del.delete_type = static_cast<uint8_t>(Referee::UIDeleteType::UI_DELETE_LAYER);
    ui_del.layer = UI_LAYER_CHASSIS;
    if (!UI_LAYER_CLEARED)
    {
      if (omni->referee_->SendUILayerDelete(ROBOT_ID, CLIENT_ID, ui_del) !=
          LibXR::ErrorCode::OK)
      {
        return;
      }
      omni->mutex_.Lock();
      omni->ui_layer_cleared_ = true;
      omni->mutex_.Unlock();
      return;
    }

    if (!UI_FRAME_INITIALIZED || (UI_TICK % UI_BOX_RESEND_DIV) == 0)
    {
      Referee::UIFigure2 box_figs{};
      /*
       * 绘制两个外框
       * CBX 是中间自瞄外框
       * CPF 是左侧电容外框
       */
      omni->referee_->FillRect(
          box_figs.interaction_figure[0], "CBX", Referee::UIFigureOp::UI_OP_ADD,
          UI_LAYER_CHASSIS, Referee::UIColor::UI_COLOR_CYAN, UI_STATUS_BOX_WIDTH,
          UI_STATUS_BOX_X1, UI_STATUS_BOX_Y1, UI_STATUS_BOX_X2, UI_STATUS_BOX_Y2);
      omni->referee_->FillRect(
          box_figs.interaction_figure[1], "CPF", Referee::UIFigureOp::UI_OP_ADD,
          UI_LAYER_CHASSIS, Referee::UIColor::UI_COLOR_WHITE, UI_CAP_BOX_WIDTH,
          UI_CAP_BOX_X1, UI_CAP_BOX_Y1, UI_CAP_BOX_X2, UI_CAP_BOX_Y2);
      if (omni->referee_->SendUIFigure2(ROBOT_ID, CLIENT_ID, box_figs) ==
          LibXR::ErrorCode::OK)
      {
        omni->mutex_.Lock();
        omni->ui_frame_initialized_ = true;
        omni->ui_cap_fill_initialized_ = false;
        omni->mutex_.Unlock();
      }
      return;
    }

    if ((UI_TICK % UI_CAP_FILL_REFRESH_DIV) == UI_CAP_FILL_REFRESH_OFFSET)
    {
      Referee::UIFigure cap_fill_fig{};
      const bool REBUILD_CAP_FILL =
          !UI_CAP_FILL_INITIALIZED || (UI_TICK % UI_CAP_FILL_READD_DIV) == 2;
      /* 单独更新电容框内部的能量填充条 */
      const uint16_t INNER_X1 = UI_CAP_BOX_X1 + UI_CAP_FILL_MARGIN;
      const uint16_t INNER_Y1 = UI_CAP_BOX_Y1 + UI_CAP_FILL_MARGIN;
      const uint16_t INNER_Y2 = UI_CAP_BOX_Y2 - UI_CAP_FILL_MARGIN;
      uint16_t inner_x2 = INNER_X1;
      if (CAP_ONLINE)
      {
        const float CLAMPED_CAP_ENERGY = std::clamp(CAP_ENERGY, 0.0f, 1.0f);
        const float INNER_WIDTH = static_cast<float>((UI_CAP_BOX_X2 - UI_CAP_BOX_X1) -
                                                     (UI_CAP_FILL_MARGIN * 2));
        inner_x2 = static_cast<uint16_t>(INNER_X1 +
                                         std::lround(INNER_WIDTH * CLAMPED_CAP_ENERGY));
        inner_x2 = std::clamp(inner_x2, INNER_X1,
                              static_cast<uint16_t>(UI_CAP_BOX_X2 - UI_CAP_FILL_MARGIN));
      }
      omni->referee_->FillRect(cap_fill_fig, "CPI",
                               REBUILD_CAP_FILL ? Referee::UIFigureOp::UI_OP_ADD
                                                : Referee::UIFigureOp::UI_OP_MODIFY,
                               UI_LAYER_CHASSIS,
                               CAP_ONLINE ? Referee::UIColor::UI_COLOR_WHITE
                                          : Referee::UIColor::UI_COLOR_BLACK,
                               UI_CAP_FILL_WIDTH, INNER_X1, INNER_Y1, inner_x2, INNER_Y2);
      if (omni->referee_->SendUIFigure(ROBOT_ID, CLIENT_ID, cap_fill_fig) ==
          LibXR::ErrorCode::OK)
      {
        omni->mutex_.Lock();
        omni->ui_cap_fill_initialized_ = true;
        omni->mutex_.Unlock();
      }
      return;
    }

    if (!UI_GUIDE_INITIALIZED || (UI_TICK % UI_BOX_RESEND_DIV) == UI_GUIDE_RESEND_OFFSET)
    {
      Referee::UIFigure2 guide_figs{};
      /* 左右两侧的引导斜线 */
      omni->referee_->FillLine(guide_figs.interaction_figure[0], "GLF",
                               Referee::UIFigureOp::UI_OP_ADD, UI_LAYER_CHASSIS,
                               Referee::UIColor::UI_COLOR_GREEN, UI_STATUS_GUIDE_WIDTH,
                               UI_STATUS_GUIDE_LEFT_X1, UI_STATUS_GUIDE_LEFT_Y1,
                               UI_STATUS_GUIDE_LEFT_X2, UI_STATUS_GUIDE_LEFT_Y2);
      omni->referee_->FillLine(guide_figs.interaction_figure[1], "GRI",
                               Referee::UIFigureOp::UI_OP_ADD, UI_LAYER_CHASSIS,
                               Referee::UIColor::UI_COLOR_GREEN, UI_STATUS_GUIDE_WIDTH,
                               UI_STATUS_GUIDE_RIGHT_X1, UI_STATUS_GUIDE_RIGHT_Y1,
                               UI_STATUS_GUIDE_RIGHT_X2, UI_STATUS_GUIDE_RIGHT_Y2);
      if (omni->referee_->SendUIFigure2(ROBOT_ID, CLIENT_ID, guide_figs) ==
          LibXR::ErrorCode::OK)
      {
        omni->mutex_.Lock();
        omni->ui_guide_initialized_ = true;
        omni->mutex_.Unlock();
      }
      return;
    }

    const bool REBUILD_AI_TEXT =
        !UI_AI_TEXT_INITIALIZED ||
        (UI_AI_TEXT_INITIALIZED && UI_TICK >= UI_AI_TEXT_READD_DIV &&
         (UI_TICK % UI_AI_TEXT_READD_DIV) == UI_AI_TEXT_TICK);
    const bool UPDATE_AI_TEXT =
        REBUILD_AI_TEXT || (UI_TICK % UI_AI_TEXT_REFRESH_DIV) == UI_AI_TEXT_TICK;
    if (UPDATE_AI_TEXT)
    {
      Referee::UICharacter ai_fig{};
      /* AI 模式开关状态文字 */
      omni->referee_->FillCharacter(ai_fig, "AIM",
                                    REBUILD_AI_TEXT ? Referee::UIFigureOp::UI_OP_ADD
                                                    : Referee::UIFigureOp::UI_OP_MODIFY,
                                    UI_LAYER_CHASSIS, GetAIModeColor(AI_MODE_ENABLED),
                                    UI_FONT_SIZE, UI_CHAR_WIDTH, UI_AI_TEXT_X,
                                    UI_AI_TEXT_Y, GetAIModeText(AI_MODE_ENABLED));
      if (omni->referee_->SendUICharacter(ROBOT_ID, CLIENT_ID, ai_fig) ==
          LibXR::ErrorCode::OK)
      {
        omni->mutex_.Lock();
        omni->ui_ai_text_initialized_ = true;
        omni->mutex_.Unlock();
      }
      return;
    }

    Referee::UICharacter mode_fig{};
    char mode_text[16]{};
    FormatModeText(mode_text, MODE);
    const bool REBUILD_MODE_TEXT =
        !UI_TEXT_INITIALIZED || (UI_TICK % UI_TEXT_READD_DIV) == 4;
    /* 底盘模式文字本体 */
    omni->referee_->FillCharacter(mode_fig, "CMT",
                                  REBUILD_MODE_TEXT ? Referee::UIFigureOp::UI_OP_ADD
                                                    : Referee::UIFigureOp::UI_OP_MODIFY,
                                  UI_LAYER_CHASSIS, Referee::UIColor::UI_COLOR_PINK,
                                  UI_FONT_SIZE, UI_CHAR_WIDTH, UI_MODE_TEXT_X,
                                  UI_MODE_TEXT_Y, mode_text);
    if (omni->referee_->SendUICharacter(ROBOT_ID, CLIENT_ID, mode_fig) ==
        LibXR::ErrorCode::OK)
    {
      omni->mutex_.Lock();
      omni->ui_text_initialized_ = true;
      omni->mutex_.Unlock();
    }
  }

 private:
  const ChassisParam PARAM;

  float target_motor_omega_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_force_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_current_[4]{0.0f, 0.0f, 0.0f, 0.0f};

  float output_[4]{0.0f, 0.0f, 0.0f, 0.0f};

  float now_vx_ = 0.0f;
  float now_vy_ = 0.0f;
  float now_omega_ = 0.0f;

  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;
  float rotor_dynamic_scale_ = 1.0f; /* 功率相关动态缩放 */

  float imu_yaw_ = 0.0f;
  float imu_roll_ = 0.0f;
  float imu_pitch_ = 0.0f;
  float yawmotor_angle_ = 0.0f;
  float pitchmotor_angle_ = 0.0f;

  float length_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float torque_n_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float baseff_l_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float px = 0.0f;
  float py = 0.0f;
  float post_x_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float post_y_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float chassis_roll_ = 0.0f;
  float chassis_pitch_ = 0.0f;
  float torque_ff_[4]{0.0, 0.0, 0.0, 0.0};
  float baseff_[4]{0.0, 0.0, 0.0, 0.0};
  float gx_ff_;
  float gy_ff_;
  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;

  Motor* motor_wheel_[4]{motor_wheel_0_, motor_wheel_1_, motor_wheel_2_, motor_wheel_3_};
  Motor::Feedback motor_feedback_[4]{};
  Motor::MotorCmd motor_cmd_[4]{};
  MotorData motor_data_{};

  LibXR::PID<float> pid_follow_;
  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_speed_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_angle_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};

  LibXR::PID<float> pid_steer_speed_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};

  ChassisTopicNames topic_names_{};
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  PowerControl* power_control_;
  PowerControlData power_control_data_;

  Referee* referee_;
  Referee::ChassisPack referee_chassis_pack_{};
  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;

  LibXR::EulerAngle<float> euler_;
  ChassisMode chassis_event_ = ChassisMode::RELAX;

  bool ui_text_initialized_ = false;
  bool ui_frame_initialized_ = false;
  bool ui_guide_initialized_ = false;
  bool ui_cap_fill_initialized_ = false;
  bool ui_ai_text_initialized_ = false;
  bool ui_layer_cleared_ = false;
  uint32_t ui_refresh_tick_ = 0;
  LibXR::Timer::TimerHandle timer_ui_{};
};
