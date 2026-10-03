#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "timebase.hpp"

#define M3508_NM_TO_LSB_RATIO \
  52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */
#define GM6020_NM_TO_LSB_RATIO \
  7370.0f /* 6020转子扭矩转化为电机控制单位的比例 */

#define HELM_CHASSIS_MAX_POWER 100

template <typename ChassisType>
class Chassis;
class Helm {
 public:
  struct ChassisParam {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float gravity_height = 0.0f;
    float reduction_ratio = 0.0f;
    float wheel_resistance = 0.0f;
    float error_compensation = 0.0f;
    float gravity = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float rotor_speed_scale = 1.0f;      /* 平移输入下的小陀螺缩放 */
    float rotor_omega_min_scale = 0.55f; /* 动态缩放下限 */
    float rotor_buffer_low_j = 35.0f;    /* 缓冲能量低阈值 */
    float rotor_buffer_high_j = 70.0f;   /* 缓冲能量高阈值 */
    float rotor_scale_lpf_alpha = 0.2f;  /* 动态缩放低通系数 */
  };
  enum class ChassisMode : uint8_t {
    RELAX,
    INDEPENDENT,
    ROTOR,
    FOLLOW,
  };

  /**
   * @brief 构造函数，初始化舵轮底盘控制对象
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令引用
   * @param motor_wheel_0 第0个驱动轮电机指针
   * @param motor_wheel_1 第1个驱动轮电机指针
   * @param motor_wheel_2 第2个驱动轮电机指针
   * @param motor_wheel_3 第3个驱动轮电机指针
   * @param motor_steer_0 第0个舵向电机指针
   * @param motor_steer_1 第1个舵向电机指针
   * @param motor_steer_2 第2个舵向电机指针
   * @param motor_steer_3 第3个舵向电机指针
   * @param task_stack_depth 控制线程栈深度
   * @param chassis_param 舵轮底盘参数
   * @param pid_follow 跟随云台角度PID参数
   * @param pid_velocity_x X方向速度PID参数
   * @param pid_velocity_y Y方向速度PID参数
   * @param pid_omega 角速度PID参数
   * @param pid_wheel_omega_0 轮子0角速度PID参数
   * @param pid_wheel_omega_1 轮子1角速度PID参数
   * @param pid_wheel_omega_2 轮子2角速度PID参数
   * @param pid_wheel_omega_3 轮子3角速度PID参数
   * @param pid_steer_angle_0 舵机0角度PID参数
   * @param pid_steer_angle_1 舵机1角度PID参数
   * @param pid_steer_angle_2 舵机2角度PID参数
   * @param pid_steer_angle_3 舵机3角度PID参数
   */
  Helm(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
       Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
       Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
       Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd,
       PowerControl* power_control, Referee* referee, uint32_t task_stack_depth,
       ChassisParam chassis_param, LibXR::PID<float>::Param pid_follow,
       LibXR::PID<float>::Param pid_velocity_x,
       LibXR::PID<float>::Param pid_velocity_y,
       LibXR::PID<float>::Param pid_omega,
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
       LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        motor_steer_0_(motor_steer_0),   /* wheel3   ▲ y  wheel2 */
        motor_steer_1_(motor_steer_1),   /*     ↑    │     ↑     */
        motor_steer_2_(motor_steer_2),   /*          │           */
        motor_steer_3_(motor_steer_3),   /* ─────────┼────────▶x */
        pid_follow_(pid_follow),         /*          │           */
        pid_velocity_x_(pid_velocity_x), /*     ↑    │     ↑     */
        pid_velocity_y_(pid_velocity_y), /* wheel0   │    wheel1 */
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1,
                         pid_wheel_speed_2, pid_wheel_speed_3},
        pid_steer_angle_{
            pid_steer_angle_0,
            pid_steer_angle_1,
            pid_steer_angle_2,
            pid_steer_angle_3,
        },
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1,
                         pid_steer_speed_2, pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(referee);

    for (int i = 0; i < 4; i++) {
      motor_wheel_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_wheel_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_wheel_cmd_[i].torque = 0.0f;
      motor_wheel_cmd_[i].position = 0.0f;
      motor_wheel_cmd_[i].velocity = 0.0f;
      motor_wheel_cmd_[i].kp = 0.0f;
      motor_wheel_cmd_[i].kd = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
      motor_steer_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_steer_cmd_[i].reduction_ratio = 1.0f;
      motor_steer_cmd_[i].torque = 0.0f;
      motor_steer_cmd_[i].position = 0.0f;
      motor_steer_cmd_[i].velocity = 0.0f;
      motor_steer_cmd_[i].kp = 0.0f;
      motor_steer_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "HelmChassisThread", task_stack_depth,
                   thread_priority);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Helm* helm, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          helm->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 舵轮底盘控制线程函数
   * @param helm Helm对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Helm* helm) {
    helm->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        "chassis_ref");
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    helm->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    helm->mutex_.Unlock();
    while (true) {
      if (cmd_suber.Available()) {
        helm->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available()) {
        helm->referee_chassis_pack_ = referee_suber.GetData();
        helm->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available()) {
        helm->current_yaw_ =
            LibXR::CycleValue<float>(yawmotor_angle_suber.GetData());
        helm->delta_yaw_ = -helm->current_yaw_;
        yawmotor_angle_suber.StartWaiting();
      }

      helm->mutex_.Lock();
      helm->Update();
      helm->UpdateCMD();
      helm->topic_delta_yaw_.Publish(helm->delta_yaw_);
      helm->Helmcontrol();
      helm->PowerControlUpdate();
      helm->mutex_.Unlock();
      helm->Output();
      helm->thread_.SleepUntil(last_time, 2);
    }
  }
  /**
   * @brief 更新电机状态
   * @details 获取当前时间戳并更新所有驱动轮电机的状态
   */
  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    for (int i = 0; i < 4; i++) {
      motor_wheel_[i]->Update();
      motor_steer_[i]->Update();
      motor_wheel_feedback_[i] = motor_wheel_[i]->GetFeedback();
      motor_steer_feedback_[i] = motor_steer_[i]->GetFeedback();
    }
  }

  /**
   * @brief 失去控制处理
   *
   */
  void LostCtrl() {
    for (int i = 0; i < 4; i++) {
      motor_wheel_[i]->Relax();
      motor_steer_[i]->Relax();
    }
  }

  /**
   * @brief 设置底盘模式 (由 Chassis 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    chassis_event_ = static_cast<ChassisMode>(mode);
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; i++) {
      pid_wheel_speed_[i].Reset();
      pid_steer_angle_[i].Reset();
      pid_steer_speed_[i].Reset();
    }
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 从CMD获取底盘控制指令,速控底盘
   */
  void UpdateCMD() {
    target_vx_ = cmd_data_.x;
    target_vy_ = cmd_data_.y;
    target_omega_ = cmd_data_.z;
  }

  /**
   * @brief 功率控制更新
   *
   */
  void PowerControlUpdate() {
    for (int i = 0; i < 4; i++) {
      motor_data_.rotorspeed_rpm_3508[i] = motor_wheel_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_wheel_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;

      motor_data_.rotorspeed_rpm_6020[i] = motor_steer_feedback_[i].velocity;
      motor_data_.output_current_6020[i] =
          motor_steer_feedback_[i].torque * GM6020_NM_TO_LSB_RATIO;
    }

    /* 第一次设置: 反馈数据用于 RLS 参数估计 */
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020);

    power_control_->CalculatePowerControlParam();

    /* 计算 3508 速度跟踪误差 */
    float speed_error_3508[4];
    for (int i = 0; i < 4; i++) {
      float actual_speed = motor_reverse_[i]
                               ? -motor_wheel_feedback_[i].velocity
                               : motor_wheel_feedback_[i].velocity;
      speed_error_3508[i] = target_speed_[i] - actual_speed;

      motor_data_.output_current_3508[i] = std::clamp(
          wheel_out_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
          -16384.0f, 16384.0f);
      motor_data_.output_current_6020[i] = std::clamp(
          steer_out_[i] * GM6020_NM_TO_LSB_RATIO, -16384.0f, 16384.0f);
    }

    /* 计算 6020 舵向速度跟踪误差 */
    float speed_error_6020[4];
    for (int i = 0; i < 4; i++) {
      speed_error_6020[i] =
          steer_angle_[i] - motor_steer_feedback_[i].velocity *
                                static_cast<float>(LibXR::TWO_PI) / 60.0f;
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508,
                                     speed_error_3508);
    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020,
                                     speed_error_6020);

    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    float max_power =
        static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f) {
      max_power = HELM_CHASSIS_MAX_POWER;
    }

    if (power_control_online && boost_mode) {
      float cap_energy = power_control_->GetCapEnergy();
      if (cap_energy > 0.8f) {
        max_power += 100.0f;
      } else if (cap_energy > 0.5f) {
        max_power += 70.0f;
      } else if (cap_energy > 0.25f) {
        max_power += 40.0f;
      }
    }

    power_control_->OutputLimit(max_power);
    power_control_data_ = power_control_->GetPowerControlData();
  }

  /**
   * @brief 速控底盘控制
   * @details 多模式控制底盘
   */
  // Body frame: +x right, +y forward; positive yaw is counterclockwise.
  void Helmcontrol() {
    /* 计算目标平移速度 */
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case (ChassisMode::INDEPENDENT): {
        target_vx_ = cmd_data_.x;
        target_vy_ = cmd_data_.y;
      } break;
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR): {
        float beta = 2*current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ = cos_beta * cmd_data_.x - sin_beta * cmd_data_.y;
        target_vy_ = sin_beta * cmd_data_.x + cos_beta * cmd_data_.y;
      } break;

      default:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
    }

    const float SQRT2 = 1.41421356237f;

    /* 计算 wz */
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_omega_ = 0.0f;
        break;
      case (ChassisMode::INDEPENDENT):
        /* z 是归一化旋转命令，不是 rad/s。 */
        target_omega_ = cmd_data_.z;
        break;
      case (ChassisMode::FOLLOW):
        target_omega_ = -pid_follow_.Calculate(0.0f, current_yaw_, dt_);
        break;
      case (ChassisMode::ROTOR):
        target_omega_ = cmd_data_.z;
        break;
      default:
        target_omega_ = 0.0f;
        break;
    }
    /* 旋转角速度必须在读取模式命令后换算。 */
    const float CHASSIS_OMEGA_RADPS = PARAM.wheel_to_center > 1.0e-6f
                                          ? target_omega_ * SQRT2 /
                                                PARAM.wheel_to_center
                                          : 0.0f;
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        for (int i = 0; i < 4; i++) {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0f;
        }
        break;
      case (ChassisMode::INDEPENDENT):
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR): {
        float max_target_speed = 0.0f;
        for (int i = 0; i < 4; i++) {
          const WheelPosition WHEEL_POSITION =
              GetWheelPosition(static_cast<uint8_t>(i));
          const float WHEEL_VX =
              target_vx_ - CHASSIS_OMEGA_RADPS * WHEEL_POSITION.y;
          const float WHEEL_VY =
              target_vy_ + CHASSIS_OMEGA_RADPS * WHEEL_POSITION.x;
          const float WHEEL_VECTOR_SPEED =
              sqrtf(WHEEL_VX * WHEEL_VX + WHEEL_VY * WHEEL_VY);
          if (WHEEL_VECTOR_SPEED < 0.05f) {
            /* 速度为零时保持当前舵角，避免静止状态下主动转舵。 */
            target_angle_[i] = LibXR::CycleValue<float>(
                motor_steer_feedback_[i].abs_angle - zero_[i]);
            target_speed_[i] = 0.0f;
          } else {
            /* 沿用原有舵角零位：+y 为 0，正角朝 +x。 */
            target_angle_[i] =
                static_cast<float>(M_PI_2) - atan2f(WHEEL_VY, WHEEL_VX);
            target_speed_[i] = motor_max_speed_ * WHEEL_VECTOR_SPEED;
            max_target_speed =
                fmaxf(max_target_speed, target_speed_[i]);
          }
        }
        /* 按最大轮速同比例缩放，保留四轮速度比例并限制至 RPM 上限。 */
        if (max_target_speed > motor_max_speed_) {
          const float SPEED_SCALE = motor_max_speed_ / max_target_speed;
          for (int i = 0; i < 4; i++) {
            target_speed_[i] *= SPEED_SCALE;
          }
        }
      } break;
      default:
        for (int i = 0; i < 4; i++) {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0f;
        }
        break;
    }
    /* 舵向差超过 90 度时反转轮电机, 减少舵向转动 */
    for (int i = 0; i < 4; i++) {
      if (fabs(
              LibXR::CycleValue(motor_steer_feedback_[i].abs_angle - zero_[i]) -
              target_angle_[i]) > M_PI_2) {
        motor_reverse_[i] = true;
      } else {
        motor_reverse_[i] = false;
      }
    }

    /* 最优角确定后按舵向误差抑制轮速，减少转舵时的侧向拖拽。 */
    for (int i = 0; i < 4; i++) {
      const LibXR::CycleValue<float> CURRENT_ANGLE(
          motor_steer_feedback_[i].abs_angle - zero_[i]);
      const LibXR::CycleValue<float> OPTIMAL_ANGLE(
          target_angle_[i] +
          (motor_reverse_[i] ? static_cast<float>(M_PI) : 0.0f));
      const float STEER_ERROR = OPTIMAL_ANGLE - CURRENT_ANGLE;
      const float ALIGNMENT = fmaxf(0.0f, cosf(STEER_ERROR));
      target_speed_[i] *= ALIGNMENT * ALIGNMENT * ALIGNMENT;
    }

    /* 根据反转状态计算轮电机和舵向电机输出 */
    for (int i = 0; i < 4; i++) {
      if (motor_reverse_[i]) {
        wheel_out_[i] = pid_wheel_speed_[i].Calculate(
            -target_speed_[i], motor_wheel_feedback_[i].velocity, dt_);

        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            LibXR::CycleValue<float>(target_angle_[i] +
                                     static_cast<float>(M_PI) + zero_[i]),
            motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity *
                static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      } else {
        wheel_out_[i] = pid_wheel_speed_[i].Calculate(
            target_speed_[i], motor_wheel_feedback_[i].velocity, dt_);
        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            target_angle_[i] + zero_[i], motor_steer_feedback_[i].abs_angle,
            dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity *
                static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      }
    }
  }

  /**
   * @brief 输出控制命令到电机
   *
   */
  void Output() {
    if (power_control_data_.is_power_limited) {
      for (int i = 0; i < 4; i++) {
        wheel_out_[i] =
            std::clamp(power_control_data_.new_output_current_3508[i] /
                           M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                       -4.4f, 4.4f);
        steer_out_[i] = static_cast<float>(
            std ::clamp(power_control_data_.new_output_current_6020[i] /
                            GM6020_NM_TO_LSB_RATIO * 1.0,
                        -2.5, 2.5));
      }
    }

    if (chassis_event_ == (ChassisMode::RELAX)) {
      LostCtrl();
    } else {
      for (int i = 0; i < 4; i++) {
        motor_wheel_cmd_[i].torque = std::clamp(wheel_out_[i], -4.4f, 4.4f);
        motor_steer_cmd_[i].torque = std::clamp(steer_out_[i], -2.5f, 2.5f);
      }
      for (int i = 0; i < 4; i++) {
        motor_wheel_[i]->Control(motor_wheel_cmd_[i]);
        motor_steer_[i]->Control(motor_steer_cmd_[i]);
      }
    }
  }

 private:
  const ChassisParam PARAM;

  struct WheelPosition {
    float x;
    float y;
  };

  WheelPosition GetWheelPosition(uint8_t wheel_index) const {
    const float COMPONENT = PARAM.wheel_to_center * 0.70710678118f;
    switch (wheel_index) {
      case 0:
        return {-COMPONENT, -COMPONENT};  // Left rear, quadrant III
      case 1:
        return {COMPONENT, -COMPONENT};   // Right rear, quadrant IV
      case 2:
        return {COMPONENT, COMPONENT};    // Right front, quadrant I
      case 3:
        return {-COMPONENT, COMPONENT};   // Left front, quadrant II
      default:
        return {0.0f, 0.0f};
    }
  }

  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;

  bool motor_reverse_[4]{false, false, false, false};
  LibXR::CycleValue<float> zero_[4] = {1.10063124, 2.05323339, 3.2244277,
                                       0.979446769};
  float current_yaw_ = 0.0f;
  float delta_yaw_ = 0.0f;

  /* 转子的转速 */
  float target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::CycleValue<float> target_angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  /* 输出的电流值 */
  float wheel_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_angle_[4] = {0.0, 0.0, 0.0, 0.0};

  float motor_max_speed_ = 9000.0;

  LibXR::CycleValue<float> main_direct_ = 0.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;
  Motor* motor_steer_0_;
  Motor* motor_steer_1_;
  Motor* motor_steer_2_;
  Motor* motor_steer_3_;

  // Wheel order: 0=LB (QIII), 1=RB (QIV), 2=RF (QI), 3=LF (QII).
  // Coordinates use +x right and +y forward; indices increase counterclockwise.
  Motor* motor_wheel_[4] = {motor_wheel_0_, motor_wheel_1_, motor_wheel_2_,
                            motor_wheel_3_};
  Motor* motor_steer_[4] = {motor_steer_0_, motor_steer_1_, motor_steer_2_,
                            motor_steer_3_};

  Motor::Feedback motor_wheel_feedback_[4]{};
  Motor::Feedback motor_steer_feedback_[4]{};

  LibXR::PID<float> pid_follow_;
  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_speed_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_angle_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_speed_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  Motor::MotorCmd motor_wheel_cmd_[4]{};
  Motor::MotorCmd motor_steer_cmd_[4]{};

  MotorData motor_data_ = {};
  PowerControl* power_control_;
  PowerControlData power_control_data_;

  Referee::ChassisPack referee_chassis_pack_{};
  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  LibXR::Topic topic_delta_yaw_ = LibXR::Topic::CreateTopic<float>("delta_yaw");
  ChassisMode chassis_event_ = ChassisMode::RELAX;
};
