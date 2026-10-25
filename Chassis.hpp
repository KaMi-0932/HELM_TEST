#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 底盘控制模块：麦轮、全向轮与舵轮底盘的速度闭环、功率限制与模式切换 / Chassis control Module with velocity loops, power limiting and mode switching for mecanum, omnidirectional and swerve (helm) chassis
depends:
- id: xrobot-org/BMI088
  ref: same-or-dev
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/PowerControl
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
- id: xrobot-org/MadgwickAHRS
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/SuperPower
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <type_traits>

#include "thread.hpp"

/// 功率控制数组中的电机数量上限
/// Maximum motor count of the power control arrays
static constexpr int CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT = 6;

/**
 * @brief 提交给 PowerControl 的电机数据，3508 与 6020 各一组。
 *        Motor data submitted to PowerControl, one set each for the 3508 and 6020 motors.
 */
struct MotorData
{
  float output_current_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  ///< 3508 电机的输出电流（电机控制单位）
  ///< Output current of the 3508 motors (motor control units)
  float rotorspeed_rpm_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  ///< 3508 电机的转子转速 (rpm)
  ///< Rotor speed of the 3508 motors (rpm)

  float output_current_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  ///< 6020 电机的输出电流（电机控制单位）
  ///< Output current of the 6020 motors (motor control units)
  float rotorspeed_rpm_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  ///< 6020 电机的转子转速 (rpm)
  ///< Rotor speed of the 6020 motors (rpm)
};

/**
 * @brief 底盘订阅的 Topic 名称。
 *        Names of the Topics subscribed by the chassis.
 */
struct ChassisTopicNames
{
  const char* chassis_cmd = "chassis_cmd";  ///< 底盘控制命令 Topic
  ///< Chassis command Topic
  const char* chassis_ref = "chassis_ref";  ///< 裁判系统底盘数据 Topic
  ///< Referee chassis data Topic
  const char* gimbal_euler = "gimbal_euler";  ///< 云台欧拉角 Topic，由全向轮底盘使用
  ///< Gimbal Euler angle Topic, used by Omni
};

#include "CMD.hpp"
#include "Helm.hpp"
#include "Mecanum.hpp"
#include "Motor.hpp"
#include "Omni.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "libxr_def.hpp"
#include "pid.hpp"

/**
 * @brief 底盘控制模块：模板参数选择麦轮、全向轮或舵轮实现。
 *        Chassis control Module; the template parameter selects the mecanum,
 *        omnidirectional-wheel or helm (swerve) implementation.
 *
 * @tparam ChassisType 底盘实现：Mecanum、Omni 或 Helm。
 *                     Chassis implementation: Mecanum, Omni or Helm.
 */
template <typename ChassisType>
class Chassis
{
 public:
  /// 所选底盘实现的模式枚举
  /// Mode enum of the selected chassis implementation
  using ChassisMode = typename ChassisType::ChassisMode;

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
    // Helm 竖直轴转动惯量 (kg*m^2)，其他底盘忽略。
    float moment_of_inertia = 0.0f;
  };

  /**
   * @brief 底盘配置参数。
   *        Chassis configuration parameters.
   */
  struct Param
  {
    ChassisParam chassis_param;  ///< 底盘几何、动力与小陀螺缩放参数
    ///< Chassis geometry, dynamics and spin-scaling parameters
    LibXR::PID<float>::Param pid_follow;  ///< 跟随云台的角度环 PID
    ///< Gimbal-following angle-loop PID
    LibXR::PID<float>::Param pid_velocity_x;  ///< x 方向速度环 PID
    ///< x velocity-loop PID
    LibXR::PID<float>::Param pid_velocity_y;  ///< y 方向速度环 PID
    ///< y velocity-loop PID
    LibXR::PID<float>::Param pid_omega;  ///< 角速度环 PID
    ///< Angular-velocity-loop PID
    LibXR::PID<float>::Param pid_wheel_speed_0;  ///< 轮 0 速度环 PID
    ///< Speed-loop PID of wheel 0
    LibXR::PID<float>::Param pid_wheel_speed_1;  ///< 轮 1 速度环 PID
    ///< Speed-loop PID of wheel 1
    LibXR::PID<float>::Param pid_wheel_speed_2;  ///< 轮 2 速度环 PID
    ///< Speed-loop PID of wheel 2
    LibXR::PID<float>::Param pid_wheel_speed_3;  ///< 轮 3 速度环 PID
    ///< Speed-loop PID of wheel 3
    LibXR::PID<float>::Param pid_steer_angle_0;  ///< 舵 0 角度环 PID，Mecanum 履带速度环
    ///< Angle-loop PID of steering motor 0; Mecanum: track speed loop
    LibXR::PID<float>::Param pid_steer_angle_1;  ///< 舵 1 角度环 PID
    ///< Angle-loop PID of steering motor 1
    LibXR::PID<float>::Param pid_steer_angle_2;  ///< 舵 2 角度环 PID
    ///< Angle-loop PID of steering motor 2
    LibXR::PID<float>::Param pid_steer_angle_3;  ///< 舵 3 角度环 PID
    ///< Angle-loop PID of steering motor 3
    LibXR::PID<float>::Param pid_steer_speed_0;  ///< 舵 0 速度环 PID
    ///< Speed-loop PID of steering motor 0
    LibXR::PID<float>::Param pid_steer_speed_1;  ///< 舵 1 速度环 PID
    ///< Speed-loop PID of steering motor 1
    LibXR::PID<float>::Param pid_steer_speed_2;  ///< 舵 2 速度环 PID
    ///< Speed-loop PID of steering motor 2
    LibXR::PID<float>::Param pid_steer_speed_3;  ///< 舵 3 速度环 PID
    ///< Speed-loop PID of steering motor 3
    LibXR::Thread::Priority thread_priority;  ///< 控制线程优先级
    ///< Control thread priority
    const char* chassis_cmd_topic_name;  ///< 订阅的底盘控制命令 Topic 名称
    ///< Name of the subscribed chassis command Topic
    const char* chassis_ref_topic_name;  ///< 订阅的裁判系统底盘数据 Topic 名称
    ///< Name of the subscribed referee chassis data Topic
    const char* gimbal_euler_topic_name;  ///< 订阅的云台欧拉角 Topic 名称（Omni 使用）
    ///< Name of the subscribed gimbal Euler angle Topic (used by Omni)
  };

  /**
   * @brief 构造 Chassis，创建所选底盘的控制线程并注册模式事件。
   *        Construct Chassis, create the control thread of the selected chassis and
   *        register the mode events.
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   *                      Drive wheel motor 0.
   * @param motor_wheel_1 第 1 个驱动轮电机。
   *                      Drive wheel motor 1.
   * @param motor_wheel_2 第 2 个驱动轮电机。
   *                      Drive wheel motor 2.
   * @param motor_wheel_3 第 3 个驱动轮电机。
   *                      Drive wheel motor 3.
   * @param motor_steer_0 舵向电机 0；Mecanum 为履带电机，可为 nullptr；Helm 须非空；
   *                      Omni 填 nullptr。
   *                      Steering motor 0; the track motor of Mecanum, may be nullptr;
   *                      non-null for Helm; nullptr for Omni.
   * @param motor_steer_1 舵向电机 1；Helm 须非空，其他底盘填 nullptr。
   *                      Steering motor 1; non-null for Helm, nullptr for the others.
   * @param motor_steer_2 舵向电机 2；Helm 须非空，其他底盘填 nullptr。
   *                      Steering motor 2; non-null for Helm, nullptr for the others.
   * @param motor_steer_3 舵向电机 3；Helm 须非空，其他底盘填 nullptr。
   *                      Steering motor 3; non-null for Helm, nullptr for the others.
   * @param cmd 控制命令模块实例，提供控制命令、控制模式与 CMD 事件。
   *            Control command Module instance, providing the control command, the
   *            control mode and the CMD events.
   * @param power_control 功率控制模块实例。
   *                      Power control Module instance.
   * @param referee 裁判系统模块实例。
   *                Referee Module instance.
   * @param task_stack_depth 控制线程栈深，默认 1536。
   *                         Control thread stack depth, default 1536.
   * @param param 配置参数，默认值见 Param 与 ChassisParam。
   *              Configuration parameters; see Param and ChassisParam for the defaults.
   */
  Chassis(Motor& motor_wheel_0, Motor& motor_wheel_1, Motor& motor_wheel_2,
          Motor& motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
          Motor* motor_steer_2, Motor* motor_steer_3, CMD& cmd,
          PowerControl& power_control, Referee& referee, uint32_t task_stack_depth = 1536,
          const Param& param = {.chassis_param = {},
                                .pid_follow = {},
                                .pid_velocity_x = {},
                                .pid_velocity_y = {},
                                .pid_omega = {},
                                .pid_wheel_speed_0 = {},
                                .pid_wheel_speed_1 = {},
                                .pid_wheel_speed_2 = {},
                                .pid_wheel_speed_3 = {},
                                .pid_steer_angle_0 = {},
                                .pid_steer_angle_1 = {},
                                .pid_steer_angle_2 = {},
                                .pid_steer_angle_3 = {},
                                .pid_steer_speed_0 = {},
                                .pid_steer_speed_1 = {},
                                .pid_steer_speed_2 = {},
                                .pid_steer_speed_3 = {},
                                .thread_priority = LibXR::Thread::Priority::HIGH,
                                .chassis_cmd_topic_name = "chassis_cmd",
                                .chassis_ref_topic_name = "chassis_ref",
                                .gimbal_euler_topic_name = "gimbal_euler"})
      : chassis_(
            &motor_wheel_0, &motor_wheel_1, &motor_wheel_2, &motor_wheel_3, motor_steer_0,
            motor_steer_1, motor_steer_2, motor_steer_3, &cmd, &power_control, &referee,
            task_stack_depth, MakeChassisParam(param.chassis_param), param.pid_follow,
            param.pid_velocity_x, param.pid_velocity_y, param.pid_omega,
            param.pid_wheel_speed_0, param.pid_wheel_speed_1, param.pid_wheel_speed_2,
            param.pid_wheel_speed_3, param.pid_steer_angle_0, param.pid_steer_angle_1,
            param.pid_steer_angle_2, param.pid_steer_angle_3, param.pid_steer_speed_0,
            param.pid_steer_speed_1, param.pid_steer_speed_2, param.pid_steer_speed_3,
            param.thread_priority,
            ChassisTopicNames{.chassis_cmd = param.chassis_cmd_topic_name,
                              .chassis_ref = param.chassis_ref_topic_name,
                              .gimbal_euler = param.gimbal_euler_topic_name}),
        referee_(&referee)
  {
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Chassis* chassis, uint32_t event_id)
        {
          UNUSED(in_isr);
          chassis->EventHandler(event_id);
        },
        this);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::RELAX), callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::INDEPENDENT), callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::ROTOR), callback);
    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::FOLLOW), callback);
    /* TRACK_START 仅 Mecanum 的 ChassisMode 提供，编译期判断后注册 */
    if constexpr (std::is_same<ChassisType, Mecanum>::value)
    {
      chassis_event_.Register(static_cast<uint32_t>(ChassisMode::TRACK_START), callback);
    }
  }

  /**
   * @brief 获取模式切换事件。
   *        Get the mode-switching event.
   *
   * @details 事件上注册了 ChassisMode 的各个值，激活对应的事件 ID 即切换底盘模式。
   *          Every value of ChassisMode is registered on the event; activating the
   *          corresponding event ID switches the chassis mode.
   *
   * @return 事件对象的引用。
   *         Reference to the event object.
   */
  LibXR::Event& GetEvent() { return chassis_event_; }

  /**
   * @brief 按事件 ID 设置底盘模式。
   *        Set the chassis mode from an event ID.
   *
   * @param event_id 事件 ID，取 ChassisMode 的值。
   *                 Event ID, a value of ChassisMode.
   */
  void EventHandler(uint32_t event_id) { chassis_.SetMode(event_id); }

 private:
  static typename ChassisType::ChassisParam MakeChassisParam(
      const ChassisParam& chassis_param)
  {
    auto type_param = typename ChassisType::ChassisParam{
        .wheel_radius = chassis_param.wheel_radius,
        .wheel_to_center = chassis_param.wheel_to_center,
        .gravity_height = chassis_param.gravity_height,
        .reduction_ratio = chassis_param.reduction_ratio,
        .wheel_resistance = chassis_param.wheel_resistance,
        .error_compensation = chassis_param.error_compensation,
        .gravity = chassis_param.gravity,
        .length = chassis_param.length,
        .width = chassis_param.width,
        .rotor_speed_scale = chassis_param.rotor_speed_scale,
        .rotor_omega_min_scale = chassis_param.rotor_omega_min_scale,
        .rotor_buffer_low_j = chassis_param.rotor_buffer_low_j,
        .rotor_buffer_high_j = chassis_param.rotor_buffer_high_j,
        .rotor_scale_lpf_alpha = chassis_param.rotor_scale_lpf_alpha};
    if constexpr (std::is_same_v<ChassisType, Helm>)
    {
      type_param.moment_of_inertia = chassis_param.moment_of_inertia;
    }
    return type_param;
  }

  ChassisType chassis_;
  LibXR::Event chassis_event_;
  Referee* referee_;
};
