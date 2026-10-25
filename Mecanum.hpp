#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 麦轮底盘控制实现，由 Chassis 模板使用 / Mecanum chassis controller used by the Chassis template
depends: []
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "SuperPower.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#define M3508_NM_TO_LSB_RATIO 52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */

#define MECANUM_MOTOR_MAX_OMEGA 52.0f     /* 麦轮输出轴最大角速度 rad/s */
#define MECANUM_CHASSIS_MAX_POWER 70      /* 麦轮默认功率上限 W */
#define TRACK_MAX_LINEAR_SPEED_MPS 0.409f /* 履带最大线速度 m/s */

template <typename ChassisType>
class Chassis;

/**
 * @brief 麦轮底盘控制实现，由 Chassis 模板使用。
 *        Mecanum chassis controller used by the Chassis template.
 */
class Mecanum
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
    FOLLOW,  ///< 跟随：底盘跟随云台 yaw，平移相对云台坐标系
    ///< Follow: the chassis follows the gimbal yaw, translation in the gimbal frame
    TRACK_START  ///< 履带：履带负责前后，麦轮提供横移与辅助
    ///< Track: the track drives forward/backward, the wheels give lateral motion
  };
  /**
   * @brief 构造麦轮底盘控制对象，创建控制线程。
   *        Construct the mecanum chassis controller and create the control thread.
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   *                      Drive wheel motor 0.
   * @param motor_wheel_1 第 1 个驱动轮电机。
   *                      Drive wheel motor 1.
   * @param motor_wheel_2 第 2 个驱动轮电机。
   *                      Drive wheel motor 2.
   * @param motor_wheel_3 第 3 个驱动轮电机。
   *                      Drive wheel motor 3.
   * @param motor_steer_0 履带电机，可为 nullptr。
   *                      Track motor, may be nullptr.
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
   * @param pid_steer_angle_0 履带速度 PID 参数。
   *                          PID parameters of the track speed loop.
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
  Mecanum(

      Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
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
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,
      ChassisTopicNames topic_names = {})
      /*
       * 麦轮编号，箭头为轮子正方向
       *
       *                 ↑ y
       *        wheel0   │   wheel3
       *          ↑      │      ↓
       *                 │
       * ────────────────┼──────────────→ x
       *                 │
       *          ↑      │      ↓
       *        wheel1   │   wheel2
       */
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        track_motor_(motor_steer_0),
        pid_follow_(pid_follow),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1, pid_wheel_speed_2,
                         pid_wheel_speed_3},
        pid_track_speed_(pid_steer_angle_0),
        cmd_(cmd),
        power_control_(power_control),
        referee_(referee)
  {
    topic_names_ = topic_names;
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    UNUSED(pid_steer_speed_0);
    UNUSED(pid_steer_speed_1);
    UNUSED(pid_steer_speed_2);
    UNUSED(pid_steer_speed_3);
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
    track_motor_cmd_.mode = Motor::ControlMode::MODE_TORQUE;
    track_motor_cmd_.reduction_ratio = chassis_param.reduction_ratio;
    track_motor_cmd_.torque = 0.0f;
    track_motor_cmd_.position = 0.0f;
    track_motor_cmd_.velocity = 0.0f;
    track_motor_cmd_.kp = 0.0f;
    track_motor_cmd_.kd = 0.0f;

    thread_.Create(this, ThreadFunction, "MecanumChassisThread", task_stack_depth,
                   thread_priority);
    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum* mecanum, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->mutex_.Lock();
          mecanum->chassis_event_ = ChassisMode::RELAX;
          mecanum->mutex_.Unlock();
        },
        this);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum* mecanum, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->mutex_.Lock();
          mecanum->chassis_event_ = ChassisMode::RELAX;
          mecanum->mutex_.Unlock();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    void (*InitUi)(Mecanum*) = [](Mecanum* mecanum) { mecanum->InitUi(); };
    auto InitUi_ = LibXR::Timer::CreateTask(InitUi, this, 52);
    LibXR::Timer::Add(InitUi_);
    LibXR::Timer::Start(InitUi_);
  }

  /**
   * @brief 麦轮底盘控制线程函数。
   *        Control thread function of the mecanum chassis.
   *
   * @details 线程主循环：接收控制命令，执行运动学解算并输出控制量。
   *          Main loop: receives the control command, solves the kinematics and sends the
   *          outputs.
   *
   * @param mecanum Mecanum 对象指针。
   *                Pointer to the Mecanum object.
   */
  static void ThreadFunction(Mecanum* mecanum)
  {
    mecanum->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber(mecanum->topic_names_.chassis_cmd);
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        mecanum->topic_names_.chassis_ref);
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    mecanum->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    mecanum->mutex_.Unlock();

    while (true)
    {
      if (cmd_suber.Available())
      {
        mecanum->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available())
      {
        mecanum->referee_chassis_pack_ = referee_suber.GetData();
        mecanum->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available())
      {
        mecanum->current_yaw_ = LibXR::CycleValue<float>(yawmotor_angle_suber.GetData() -
                                                         mecanum->yawmotor_zero_);
        yawmotor_angle_suber.StartWaiting();
      }

      mecanum->mutex_.Lock();
      mecanum->Update();
      mecanum->UpdateTrack();
      mecanum->UpdateCMD();
      mecanum->SelfResolution();
      mecanum->InverseKinematicsSolution();
      mecanum->DynamicInverseSolution();
      mecanum->CalculateMotorCurrent();
      mecanum->CalculateTrackCurrent();
      mecanum->PowerControlUpdate();
      mecanum->mutex_.Unlock();
      mecanum->OutputToDynamics();
      mecanum->ControlTrack();

      mecanum->thread_.SleepUntil(last_time, 2);
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
  void UpdateTrack()
  {
    if (track_motor_ == nullptr)
    {
      track_linear_speed_ = 0.0f;
      return;
    }
    track_motor_->Update();
    track_motor_feedback_ = track_motor_->GetFeedback();
    /* 转子角速度换算为履带线速度 */
    track_linear_speed_ =
        track_motor_feedback_.omega / PARAM.reduction_ratio * TRACK_WHEEL_RADIUS_M;
  }

  /**
   * @brief 设置底盘模式并复位速度、履带与轮速 PID，由 Chassis 外壳调用。
   *        Set the chassis mode and reset the velocity, track and wheel-speed PIDs;
   *        called by the Chassis shell.
   *
   * @param mode 新模式，取 ChassisMode 的值。
   *             New mode, a value of ChassisMode.
   */
  void SetMode(uint32_t mode)
  {
    mutex_.Lock();
    chassis_event_ = static_cast<ChassisMode>(mode);
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    pid_track_speed_.Reset();
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
    float max_v = PARAM.wheel_radius * MECANUM_MOTOR_MAX_OMEGA;

    /* 先生成目标角速度 */
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        target_omega_ = 0.0f;
        break;

      case (ChassisMode::INDEPENDENT):
        target_omega_ = -max_v * cmd_data_.z / PARAM.wheel_to_center;
        break;

      case (ChassisMode::ROTOR):
        this->target_omega_ = static_cast<float>(max_v / PARAM.wheel_to_center);
        break;

      case (ChassisMode::FOLLOW):
        target_omega_ = -pid_follow_.Calculate(0.0f, -current_yaw_, dt_);
        break;

      case (ChassisMode::TRACK_START):
      {
        float max_omega = max_v / PARAM.wheel_to_center;
        /* 履带模式只保留小幅 FOLLOW 纠偏 */
        target_omega_ = -pid_follow_.Calculate(0.0f, -current_yaw_, dt_);
        target_omega_ =
            std::clamp(target_omega_, -max_omega * TRACK_FOLLOW_OMEGA_LIMIT_SCALE,
                       max_omega * TRACK_FOLLOW_OMEGA_LIMIT_SCALE);
      }
      break;

      default:
        break;
    }

    /* 再生成目标平移速度 */
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case (ChassisMode::ROTOR):
      case (ChassisMode::FOLLOW):
      {
        float beta = -current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ = (cos_beta * cmd_data_.x * max_v + sin_beta * cmd_data_.y * max_v);
        target_vy_ = (-sin_beta * cmd_data_.x * max_v + cos_beta * cmd_data_.y * max_v);
      }
      break;
      case (ChassisMode::TRACK_START):
      {
        float beta = -current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        /* 履带负责前后，麦轮保留横移能力 */
        float assist_vx = cmd_data_.x * max_v * TRACK_LATERAL_SPEED_SCALE;
        float assist_vy = GetTrackWheelAssistSpeed();
        target_vx_ = cos_beta * assist_vx + sin_beta * assist_vy;
        target_vy_ = -sin_beta * assist_vx + cos_beta * assist_vy;
      }
      break;
      case (ChassisMode::INDEPENDENT):
      {
        target_vx_ = cmd_data_.x * max_v;
        target_vy_ = cmd_data_.y * max_v;
      }
      break;
      default:
        break;
    }

    /* 小陀螺模式下根据平移输入与功率状态动态调整旋转速度 */
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
   * @brief 麦轮底盘正运动学解算。
   *        Forward kinematics of the mecanum chassis.
   *
   * @details 由四个麦轮的角速度解算底盘的运动状态。
   *          Solves the chassis motion from the angular velocities of the four wheels.
   */
  void SelfResolution()
  {
    now_vx_ = (motor_feedback_[0].omega / PARAM.reduction_ratio -
               motor_feedback_[1].omega / PARAM.reduction_ratio -
               motor_feedback_[2].omega / PARAM.reduction_ratio +
               motor_feedback_[3].omega / PARAM.reduction_ratio) *
              PARAM.wheel_radius / 4.0f;

    now_vy_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
               motor_feedback_[1].omega / PARAM.reduction_ratio -
               motor_feedback_[2].omega / PARAM.reduction_ratio -
               motor_feedback_[3].omega / PARAM.reduction_ratio) *
              PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
                  motor_feedback_[1].omega / PARAM.reduction_ratio +
                  motor_feedback_[2].omega / PARAM.reduction_ratio +
                  motor_feedback_[3].omega / PARAM.reduction_ratio) *
                 PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 麦轮底盘逆运动学解算。
   *        Inverse kinematics of the mecanum chassis.
   *
   * @details 由目标底盘速度 (vx, vy, ω) 计算四个麦轮的目标角速度。
   *          Computes the target angular velocity of the four wheels from the target
   *          chassis velocity (vx, vy, ω).
   */
  void InverseKinematicsSolution()
  {
    target_motor_omega_[0] =
        (target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[1] =
        (-target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[2] =
        (-target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[3] =
        (target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
  }

  /**
   * @brief 计算轮速 PID 输出并与前馈力矩合成。
   *        Compute the wheel-speed PID outputs and combine them with the feedforward
   *        torque.
   */
  void CalculateMotorCurrent()
  {
    if (chassis_event_ == ChassisMode::RELAX)
    {
      LostCtrl();
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        target_motor_current_[i] = pid_wheel_speed_[i].Calculate(
            target_motor_omega_[i], motor_feedback_[i].omega / PARAM.reduction_ratio,
            dt_);
      }
      /* 计算输出 */
      for (int i = 0; i < 4; i++)
      {
        output_[i] =
            target_motor_force_[i] * PARAM.wheel_radius + target_motor_current_[i];
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
    /* 采样当前反馈电流和转速供功率模型参数估计使用 */
    for (int i = 0; i < 4; i++)
    {
      motor_data_.rotorspeed_rpm_3508[i] = motor_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;
    }
    /* 第五路是履带电机反馈 */
    motor_data_.rotorspeed_rpm_3508[4] =
        track_motor_ == nullptr ? 0.0f : track_motor_feedback_.velocity;
    motor_data_.output_current_3508[4] =
        track_motor_ == nullptr ? 0.0f
                                : track_motor_feedback_.torque * M3508_NM_TO_LSB_RATIO;

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->CalculatePowerControlParam();

    float speed_error[5] = {};

    /* 写入五路期望电流供限功率使用 */
    for (int i = 0; i < 4; i++)
    {
      speed_error[i] =
          target_motor_omega_[i] - motor_feedback_[i].omega / PARAM.reduction_ratio;
      motor_data_.output_current_3508[i] =
          std::clamp(output_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }
    /* 履带线速度误差换算为主动轮角速度误差 */
    speed_error[4] = track_speed_error_ / TRACK_WHEEL_RADIUS_M;
    motor_data_.output_current_3508[4] = std::clamp(
        track_output_current_ * static_cast<float>(M3508_MAX_ABS_LSB),
        -static_cast<float>(M3508_MAX_ABS_LSB), static_cast<float>(M3508_MAX_ABS_LSB));

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508, speed_error);
    PowerControl::AllocationBias3508 allocation_bias{};
    if (track_motor_ != nullptr && chassis_event_ == ChassisMode::TRACK_START)
    {
      const float TRACK_CMD_MAG = GetTrackCommandMagnitude();
      const bool TRACK_ACTIVE = TRACK_CMD_MAG > TRACK_ACTIVE_SPEED_EPS_MPS;
      const bool TRACK_STALLED =
          fabsf(track_target_speed_) > TRACK_ACTIVE_SPEED_EPS_MPS &&
          fabsf(track_linear_speed_) <
              fabsf(track_target_speed_) * TRACK_STALL_SPEED_RATIO;
      float wheel_omega_abs_sum = 0.0f;
      for (int i = 0; i < 4; i++)
      {
        wheel_omega_abs_sum += fabsf(motor_feedback_[i].omega);
      }
      const bool WHEEL_FREE_SPIN =
          wheel_omega_abs_sum > TRACK_WHEEL_FREE_SPIN_OMEGA_RADPS * WHEEL_COUNT_FLOAT;
      const bool TRACK_NEEDS_PRIORITY =
          TRACK_ACTIVE && (fabsf(track_speed_error_) > TRACK_PRIORITY_ERROR_EPS_MPS ||
                           (TRACK_STALLED && WHEEL_FREE_SPIN));

      allocation_bias.enabled = true;
      allocation_bias.reserve_fraction = TRACK_NEEDS_PRIORITY
                                             ? TRACK_PRIORITY_RESERVE_FRACTION
                                             : TRACK_CRUISE_RESERVE_FRACTION;
      for (int i = 0; i < 4; i++)
      {
        allocation_bias.reserve_weight[i] = 0.0f;
        allocation_bias.allocation_weight_scale[i] =
            TRACK_NEEDS_PRIORITY ? TRACK_WHEEL_DEPRIORITY_SCALE
                                 : TRACK_WHEEL_ASSIST_PRIORITY_SCALE;
      }
      allocation_bias.reserve_weight[4] = 1.0f;
      allocation_bias.allocation_weight_scale[4] =
          TRACK_NEEDS_PRIORITY ? TRACK_PRIORITY_WEIGHT_SCALE : TRACK_CRUISE_WEIGHT_SCALE;
    }
    power_control_->SetAllocationBias3508(allocation_bias);

    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    /* 裁判系统离线或上限异常时回退到本地默认功率上限 */
    float max_power = static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f)
    {
      max_power = MECANUM_CHASSIS_MAX_POWER;
    }

    /* BOOST 模式按电容能量分档提升可用功率上限 */
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

    /* 受限程度估计为限幅后电流总量除以请求电流总量 */
    float req_current_abs_sum = 0.0f;
    float lim_current_abs_sum = 0.0f;
    for (int i = 0; i < 5; i++)
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

    /* 将裁判缓冲能量映射为缩放因子，裁判系统离线时保持 1.0 */
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

    /* 合成目标缩放并做一阶低通抑制跳变 */
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

    /* 仅在小陀螺模式保留缩放，其他模式复位为 1.0 */
    if (chassis_event_ != ChassisMode::ROTOR)
    {
      rotor_dynamic_scale_ = 1.0f;
    }
  }

  /**
   * @brief 麦轮底盘逆动力学解算。
   *        Inverse dynamics of the mecanum chassis.
   *
   * @details 以运动学正解得到的当前状态与目标状态做 PID，得到各轮的前馈力。
   *          Runs PIDs between the state from the forward kinematics and the target state
   *          to obtain the feedforward force of each wheel.
   */
  void DynamicInverseSolution()
  {
    float force_x = pid_velocity_x_.Calculate(target_vx_, now_vx_, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, now_vy_, dt_);
    float force_z = pid_omega_.Calculate(target_omega_, now_omega_, dt_);

    /* 按麦轮受力方向分配前馈力 */
    target_motor_force_[0] = (force_x + force_y + force_z) / 4;
    target_motor_force_[1] = (-force_x + force_y + force_z) / 4;
    target_motor_force_[2] = (-force_x - force_y + force_z) / 4;
    target_motor_force_[3] = (force_x - force_y + force_z) / 4;
  }

  /**
   * @brief 麦轮底盘动力学输出。
   *        Dynamics output of the mecanum chassis.
   *
   * @details 限幅后向四个麦轮下发力矩指令。
   *          Limits the torques and sends them to the four wheels.
   */
  void OutputToDynamics()
  {
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
  float GetTrackCommandMagnitude() const
  {
    /* 遥控 y 先缩放再开方，提高低速段的输入分辨率 */
    const float INPUT = cmd_data_.y * TRACK_INPUT_SCALE;
    return std::sqrt(std::abs(INPUT)) * TRACK_MAX_LINEAR_SPEED_MPS;
  }
  float GetTrackSetpointSpeed() const
  {
    /* 履带电机方向和底盘前进方向相反 */
    const float INPUT = cmd_data_.y * TRACK_INPUT_SCALE;
    const float SIGN = (INPUT < 0.0f) ? 1.0f : -1.0f;
    return SIGN * GetTrackCommandMagnitude();
  }
  float GetTrackWheelAssistSpeed() const
  {
    /* 麦轮辅助方向和底盘前进方向一致 */
    const float INPUT = cmd_data_.y * TRACK_INPUT_SCALE;
    const float SIGN = (INPUT < 0.0f) ? -1.0f : 1.0f;
    return SIGN * GetTrackCommandMagnitude() * TRACK_WHEEL_ASSIST_SCALE;
  }
  void CalculateTrackCurrent()
  {
    if (track_motor_ == nullptr || chassis_event_ != ChassisMode::TRACK_START)
    {
      track_target_speed_ = 0.0f;
      track_output_current_ = 0.0f;
      track_speed_error_ = 0.0f;
      return;
    }

    const float DESIRED_TRACK_SPEED = GetTrackSetpointSpeed();
    const float MAX_DELTA = TRACK_SPEED_RAMP_MPS2 * dt_;
    /* 目标速度加斜坡，限制履带目标速度的变化率 */
    track_target_speed_ +=
        std::clamp(DESIRED_TRACK_SPEED - track_target_speed_, -MAX_DELTA, MAX_DELTA);
    track_speed_error_ = track_target_speed_ - track_linear_speed_;
    track_output_current_ =
        pid_track_speed_.Calculate(track_target_speed_, track_linear_speed_, dt_);
  }
  void ControlTrack()
  {
    if (track_motor_ == nullptr)
    {
      return;
    }

    mutex_.Lock();
    const bool RELAX = chassis_event_ == ChassisMode::RELAX;
    float track_output_current = track_output_current_;
    const PowerControlData POWER_CONTROL_DATA = power_control_data_;
    mutex_.Unlock();

    if (RELAX)
    {
      track_motor_->Relax();
      return;
    }
    if (POWER_CONTROL_DATA.is_power_limited)
    {
      /* 限功率时使用第五路重新分配后的电流 */
      track_output_current = std::clamp(POWER_CONTROL_DATA.new_output_current_3508[4] /
                                            static_cast<float>(M3508_MAX_ABS_LSB),
                                        -1.0f, 1.0f);
    }

    /* 按麦轮相同的电流到输出轴扭矩关系下发 */
    track_motor_cmd_.torque =
        std::clamp(track_output_current * static_cast<float>(M3508_MAX_ABS_LSB) /
                       M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                   -6.0f, 6.0f);
    track_motor_cmd_.velocity = 0.0f;
    track_motor_->Control(track_motor_cmd_);
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
    if (track_motor_ != nullptr)
    {
      track_motor_->Relax();
    }
  }
  void InitUi()
  {
    const uint16_t id = referee_->GetRobotID();
    const uint16_t client = referee_->GetClientID(id);
    Referee::UIFigureOp ADD_OP = Referee::UIFigureOp::UI_OP_MODIFY;
    if (this->ui_dyn_step_ % 4 == 0)
    {
      ADD_OP = Referee::UIFigureOp::UI_OP_ADD;
      ui_dyn_step_ = 0;
    }

    switch (ui_step_)
    {
      case 0:
      {
        const char* mode_str = "RELX";
        switch (chassis_event_)
        {
          case ChassisMode::RELAX:
            mode_str = "RELX";
            current_mode_ = ChassisMode::RELAX;
            break;
          case ChassisMode::FOLLOW:
            mode_str = "FOLW";
            current_mode_ = ChassisMode::FOLLOW;
            break;
          case ChassisMode::ROTOR:
            mode_str = "ROTO";
            current_mode_ = ChassisMode::ROTOR;
            break;
          case ChassisMode::TRACK_START:
            mode_str = "TRAC";
            current_mode_ = ChassisMode::TRACK_START;
          default:
            mode_str = "RELX";
            current_mode_ = ChassisMode::RELAX;
            break;
        }
        if (current_mode_ != last_ui_mode_ or ADD_OP == Referee::UIFigureOp::UI_OP_ADD)
        {
          Referee::UICharacter char_fig{};
          referee_->FillCharacter(char_fig, "WM", ADD_OP, 1,
                                  Referee::UIColor::UI_COLOR_CYAN, 27, 2, 1345, 764,
                                  mode_str);
          referee_->SendUICharacter(id, client, char_fig);
          last_ui_mode_ = current_mode_;
        }
        break;
      }
      case 1:
      {
        if (ADD_OP == Referee::UIFigureOp::UI_OP_ADD)
        {
          Referee::UIFigure line_fig{};
          referee_->FillLine(line_fig, "WSL", ADD_OP, 1, Referee::UIColor::UI_COLOR_CYAN,
                             2, 0, 0, 400, 1000);
          referee_->SendUIFigure(id, client, line_fig);
        }
        break;
      }
      case 3:
      {
        if (ADD_OP == Referee::UIFigureOp::UI_OP_ADD)
        {
          Referee::UIFigure line_fig{};
          referee_->FillLine(line_fig, "WSR", ADD_OP, 1, Referee::UIColor::UI_COLOR_CYAN,
                             2, 1920, 0, 1520, 1000);
          referee_->SendUIFigure(id, client, line_fig);
        }
        break;
      }
      case 2:
      {
        Referee::UIFigure spfig{};
        Referee::UIColor cap_state;

        float cap_energy = power_control_->GetCapEnergy();
        if (cap_energy < 0.35f)
        {
          cap_state = Referee::UIColor::UI_COLOR_ORANGE;
          this->cnt_low++;
        }
        else
        {
          cap_state = Referee::UIColor::UI_COLOR_CYAN;
          this->cnt_high++;
        }
        this->cap_ener = cap_energy;
        referee_->FillLine(spfig, "MSP", ADD_OP, 1, cap_state, 20, 50, 550,
                           static_cast<uint16_t>(100 + 450 * cap_energy), 550);
        referee_->SendUIFigure(id, client, spfig);
        this->ui_dyn_step_ = (this->ui_dyn_step_ + 1) % 20;
      }
      default:
        break;
    }

    this->ui_step_ = (this->ui_step_ + 1) % 4;
  }

 private:
  /* 履带模式使用完整遥控行程 */
  static constexpr float TRACK_INPUT_SCALE = 1.0f;
  /* 麦轮辅助速度按履带目标速度 1:1 跟随 */
  static constexpr float TRACK_WHEEL_ASSIST_SCALE = 1.0f;
  /* 履带模式的 FOLLOW 纠偏角速度上限为普通最大角速度的 35% */
  static constexpr float TRACK_FOLLOW_OMEGA_LIMIT_SCALE = 0.35f;
  /* 履带目标线速度最大变化率, 1.2 表示每秒最多变化 1.2 m/s */
  static constexpr float TRACK_SPEED_RAMP_MPS2 = 1.2f;
  /* 履带主动轮半径 m */
  static constexpr float TRACK_WHEEL_RADIUS_M = 0.025f;
  static constexpr float TRACK_ACTIVE_SPEED_EPS_MPS = 0.05f;
  static constexpr float TRACK_PRIORITY_ERROR_EPS_MPS = 0.12f;
  static constexpr float TRACK_STALL_SPEED_RATIO = 0.35f;
  static constexpr float TRACK_WHEEL_FREE_SPIN_OMEGA_RADPS = 18.0f;
  static constexpr float TRACK_CRUISE_RESERVE_FRACTION = 0.55f;
  static constexpr float TRACK_PRIORITY_RESERVE_FRACTION = 0.8f;
  static constexpr float TRACK_LATERAL_SPEED_SCALE = 0.85f;
  static constexpr float TRACK_WHEEL_ASSIST_PRIORITY_SCALE = 0.45f;
  static constexpr float TRACK_WHEEL_DEPRIORITY_SCALE = 0.1f;
  static constexpr float TRACK_CRUISE_WEIGHT_SCALE = 1.35f;
  static constexpr float TRACK_PRIORITY_WEIGHT_SCALE = 2.5f;
  static constexpr float WHEEL_COUNT_FLOAT = 4.0f;

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

  float current_yaw_ = 0.0f;
  float yawmotor_zero_ = 0.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;
  Motor* track_motor_;

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

  LibXR::PID<float> pid_track_speed_;

  LibXR::PID<float> pid_steer_angle_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};

  LibXR::PID<float> pid_steer_speed_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};

  float track_linear_speed_ = 0.0f;
  float track_target_speed_ = 0.0f;
  float track_speed_error_ = 0.0f;
  float track_output_current_ = 0.0f;
  Motor::Feedback track_motor_feedback_{};
  Motor::MotorCmd track_motor_cmd_{};

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  PowerControl* power_control_;
  PowerControlData power_control_data_;

  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;
  Referee::ChassisPack referee_chassis_pack_{};

  ChassisTopicNames topic_names_{};
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  ChassisMode chassis_event_ = ChassisMode::RELAX;
  Referee* referee_;

  uint16_t ui_step_ = 0;
  uint16_t ui_dyn_step_ = 0;
  ChassisMode current_mode_ = ChassisMode::RELAX;
  ChassisMode last_ui_mode_ = ChassisMode::RELAX;
  LibXR::MillisecondTimestamp ui_update_time_ = 0;

  uint16_t cnt_high = 0;
  uint16_t cnt_low = 0;
  float cap_ener = 0.0f;
};
