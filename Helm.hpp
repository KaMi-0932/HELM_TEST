#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 舵轮底盘控制实现，由 Chassis 模板使用 / Helm (swerve) chassis controller used by the Chassis template
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
#include "HelmPowerAllocation.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#define M3508_NM_TO_LSB_RATIO 52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */
#define GM6020_NM_TO_LSB_RATIO 7370.0f /* 6020转子扭矩转化为电机控制单位的比例 */

#define HELM_CHASSIS_MAX_POWER 100

/* 固定规模的有界阻尼最小二乘，不分配堆内存。
 * matrix * torque 对应 [r*Fx, r*Fy, r*Mz/R]，三轴统一到 N*m 后等权跟踪。
 * 最小化 ||matrix*torque-requested||^2 + DAMPING*||torque||^2。
 * 下、上界由调用者扣除本周期轮速 PID 后的可用力矩确定。 */
namespace HelmForceAllocation
{
constexpr int WHEEL_COUNT = 4;
constexpr int WRENCH_DIM = 3;
constexpr float DAMPING = 0.01f;
constexpr int MAX_ITERATIONS = 32;

struct Result
{
  float torque_nm[WHEEL_COUNT] = {};
  float residual_norm = 0.0f;
  uint32_t iterations = 0;
  bool valid = false;
  bool converged = false;
  bool rank_deficient = false;
  bool saturated = false;
};

inline bool solve_positive_definite(const float matrix[WHEEL_COUNT][WHEEL_COUNT],
                                    const float rhs[WHEEL_COUNT], int count,
                                    float solution[WHEEL_COUNT])
{
  float factor[WHEEL_COUNT][WHEEL_COUNT] = {};
  float intermediate[WHEEL_COUNT] = {};
  for (int i = 0; i < count; ++i)
  {
    for (int j = 0; j <= i; ++j)
    {
      float value = matrix[i][j];
      for (int k = 0; k < j; ++k)
      {
        value -= factor[i][k] * factor[j][k];
      }
      if (i == j)
      {
        if (!std::isfinite(value) || value <= 1.0e-7f)
        {
          return false;
        }
        factor[i][j] = std::sqrt(value);
      }
      else
      {
        factor[i][j] = value / factor[j][j];
      }
    }
    float value = rhs[i];
    for (int j = 0; j < i; ++j)
    {
      value -= factor[i][j] * intermediate[j];
    }
    intermediate[i] = value / factor[i][i];
  }
  for (int i = count - 1; i >= 0; --i)
  {
    float value = intermediate[i];
    for (int j = i + 1; j < count; ++j)
    {
      value -= factor[j][i] * solution[j];
    }
    solution[i] = value / factor[i][i];
    if (!std::isfinite(solution[i]))
    {
      return false;
    }
  }
  return true;
}

inline Result solve(const float matrix[WRENCH_DIM][WHEEL_COUNT],
                    const float requested[WRENCH_DIM], const float lower[WHEEL_COUNT],
                    const float upper[WHEEL_COUNT])
{
  Result result;
  float normal[WHEEL_COUNT][WHEEL_COUNT] = {};
  float rhs[WHEEL_COUNT] = {};
  /* -1/1 为下/上界，0 为自由变量，2 为固定变量。 */
  int active[WHEEL_COUNT] = {};
  for (int row = 0; row < WRENCH_DIM; ++row)
  {
    if (!std::isfinite(requested[row]))
    {
      return result;
    }
    for (int i = 0; i < WHEEL_COUNT; ++i)
    {
      if (!std::isfinite(matrix[row][i]))
      {
        return result;
      }
    }
  }
  float tolerance = 1.0e-5f;
  for (int i = 0; i < WHEEL_COUNT; ++i)
  {
    if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] > upper[i])
    {
      return result;
    }
    result.torque_nm[i] = std::clamp(0.0f, lower[i], upper[i]);
    if (upper[i] - lower[i] <= 1.0e-6f)
    {
      active[i] = 2;
    }
    for (int row = 0; row < WRENCH_DIM; ++row)
    {
      rhs[i] += matrix[row][i] * requested[row];
      for (int j = 0; j < WHEEL_COUNT; ++j)
      {
        normal[i][j] += matrix[row][i] * matrix[row][j];
      }
    }
    normal[i][i] += DAMPING;
    if (!std::isfinite(rhs[i]))
    {
      return result;
    }
    tolerance = std::max(tolerance, 1.0e-5f * std::fabs(rhs[i]));
  }

  /* 秩不足常见于四轮平行；只作诊断，求解始终使用阻尼而不直接求逆。 */
  float gram[WRENCH_DIM][WRENCH_DIM] = {};
  for (int i = 0; i < WRENCH_DIM; ++i)
  {
    for (int j = 0; j < WRENCH_DIM; ++j)
    {
      for (int k = 0; k < WHEEL_COUNT; ++k)
      {
        gram[i][j] += matrix[i][k] * matrix[j][k];
      }
    }
  }
  const float DET = gram[0][0] * (gram[1][1] * gram[2][2] - gram[1][2] * gram[2][1]) -
                    gram[0][1] * (gram[1][0] * gram[2][2] - gram[1][2] * gram[2][0]) +
                    gram[0][2] * (gram[1][0] * gram[2][1] - gram[1][1] * gram[2][0]);
  const float SCALE = (gram[0][0] + gram[1][1] + gram[2][2]) / 3.0f;
  result.rank_deficient = SCALE <= 1.0e-6f || DET <= 1.0e-3f * SCALE * SCALE * SCALE;

  /* 有界二次规划的活动集法：饱和后重新分配；KKT 不满足时允许释放约束。 */
  for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration)
  {
    result.iterations = static_cast<uint32_t>(iteration + 1);
    int free_index[WHEEL_COUNT] = {};
    int free_count = 0;
    for (int i = 0; i < WHEEL_COUNT; ++i)
    {
      if (active[i] == 0)
      {
        free_index[free_count++] = i;
      }
    }
    float reduced[WHEEL_COUNT][WHEEL_COUNT] = {};
    float reduced_rhs[WHEEL_COUNT] = {};
    float solution[WHEEL_COUNT] = {};
    for (int i = 0; i < free_count; ++i)
    {
      const int INDEX = free_index[i];
      reduced_rhs[i] = rhs[INDEX];
      for (int j = 0; j < WHEEL_COUNT; ++j)
      {
        if (active[j] != 0)
        {
          reduced_rhs[i] -= normal[INDEX][j] * result.torque_nm[j];
        }
      }
      for (int j = 0; j < free_count; ++j)
      {
        reduced[i][j] = normal[INDEX][free_index[j]];
      }
    }
    if (!solve_positive_definite(reduced, reduced_rhs, free_count, solution))
    {
      return result;
    }
    float step = 1.0f;
    int hit_index = -1;
    int hit_bound = 0;
    for (int i = 0; i < free_count; ++i)
    {
      const int INDEX = free_index[i];
      const float CHANGE = solution[i] - result.torque_nm[INDEX];
      float candidate_step = 1.0f;
      int candidate_bound = 0;
      if (solution[i] < lower[INDEX] && CHANGE < 0.0f)
      {
        candidate_step = (lower[INDEX] - result.torque_nm[INDEX]) / CHANGE;
        candidate_bound = -1;
      }
      else if (solution[i] > upper[INDEX] && CHANGE > 0.0f)
      {
        candidate_step = (upper[INDEX] - result.torque_nm[INDEX]) / CHANGE;
        candidate_bound = 1;
      }
      if (candidate_bound != 0 && candidate_step < step)
      {
        step = std::max(0.0f, candidate_step);
        hit_index = INDEX;
        hit_bound = candidate_bound;
      }
    }
    for (int i = 0; i < free_count; ++i)
    {
      const int INDEX = free_index[i];
      result.torque_nm[INDEX] = std::clamp(
          result.torque_nm[INDEX] + step * (solution[i] - result.torque_nm[INDEX]),
          lower[INDEX], upper[INDEX]);
    }
    if (hit_index >= 0)
    {
      active[hit_index] = hit_bound;
      result.torque_nm[hit_index] = hit_bound < 0 ? lower[hit_index] : upper[hit_index];
      continue;
    }
    int release_index = -1;
    float largest_violation = tolerance;
    for (int i = 0; i < WHEEL_COUNT; ++i)
    {
      float gradient = -rhs[i];
      for (int j = 0; j < WHEEL_COUNT; ++j)
      {
        gradient += normal[i][j] * result.torque_nm[j];
      }
      const float VIOLATION = active[i] == -1 ? -gradient : active[i] == 1 ? gradient : 0;
      if (VIOLATION > largest_violation)
      {
        largest_violation = VIOLATION;
        release_index = i;
      }
    }
    if (release_index < 0)
    {
      result.converged = true;
      break;
    }
    active[release_index] = 0;
  }
  float residual_squared = 0.0f;
  for (int row = 0; row < WRENCH_DIM; ++row)
  {
    float error = -requested[row];
    for (int i = 0; i < WHEEL_COUNT; ++i)
    {
      error += matrix[row][i] * result.torque_nm[i];
    }
    residual_squared += error * error;
  }
  result.residual_norm = std::sqrt(residual_squared);
  result.valid = std::isfinite(result.residual_norm);
  for (int i = 0; i < WHEEL_COUNT; ++i)
  {
    result.saturated = result.saturated || active[i] != 0 ||
                       result.torque_nm[i] <= lower[i] + 1.0e-6f ||
                       result.torque_nm[i] >= upper[i] - 1.0e-6f;
    result.valid = result.valid && std::isfinite(result.torque_nm[i]);
  }
  return result;
}
}  // namespace HelmForceAllocation

struct ChassisVelocity
{
  float x = 0.0f;      // m/s, +x right
  float y = 0.0f;      // m/s, +y forward
  float omega = 0.0f;  // rad/s, counterclockwise positive
  bool valid = false;
};

template <typename ChassisType>
class Chassis;

/**
 * @brief 舵轮底盘控制实现，由 Chassis 模板使用。
 *        Helm (swerve) chassis controller used by the Chassis template.
 */
class Helm
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
    float wheel_to_center = 0.0f;  ///< 轮心距 (m)，由长宽计算；长宽均为 0 时兼容旧配置
    ///< Derived wheel-center radius (m); legacy square radius when both spans are zero
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
    float length = 0.0f;  ///< 前后轮心的完整间距 (m)，沿底盘 y 轴
    ///< Full front-to-rear wheel-center spacing along the chassis y axis (m)
    float width = 0.0f;  ///< 左右轮心的完整间距 (m)，沿底盘 x 轴
    ///< Full left-to-right wheel-center spacing along the chassis x axis (m)
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
    // 绕底盘竖直轴的转动惯量 (kg*m^2)，0 停用旋转力矩前馈。
    float moment_of_inertia = 0.0f;
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
   * @brief 构造舵轮底盘控制对象，创建控制线程。
   *        Construct the helm chassis controller and create the control thread.
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   *                      Drive wheel motor 0.
   * @param motor_wheel_1 第 1 个驱动轮电机。
   *                      Drive wheel motor 1.
   * @param motor_wheel_2 第 2 个驱动轮电机。
   *                      Drive wheel motor 2.
   * @param motor_wheel_3 第 3 个驱动轮电机。
   *                      Drive wheel motor 3.
   * @param motor_steer_0 第 0 个舵向电机，非空。
   *                      Steering motor 0, non-null.
   * @param motor_steer_1 第 1 个舵向电机，非空。
   *                      Steering motor 1, non-null.
   * @param motor_steer_2 第 2 个舵向电机，非空。
   *                      Steering motor 2, non-null.
   * @param motor_steer_3 第 3 个舵向电机，非空。
   *                      Steering motor 3, non-null.
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
   * @param pid_steer_angle_0 舵 0 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 0.
   * @param pid_steer_angle_1 舵 1 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 1.
   * @param pid_steer_angle_2 舵 2 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 2.
   * @param pid_steer_angle_3 舵 3 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 3.
   * @param pid_steer_speed_0 舵 0 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 0.
   * @param pid_steer_speed_1 舵 1 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 1.
   * @param pid_steer_speed_2 舵 2 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 2.
   * @param pid_steer_speed_3 舵 3 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 3.
   * @param thread_priority 控制线程优先级。
   *                        Control thread priority.
   * @param topic_names 订阅的 Topic 名称。
   *                    Names of the subscribed Topics.
   */
  Helm(Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
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
      : PARAM(NormalizeChassisGeometry(chassis_param)),
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
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1, pid_wheel_speed_2,
                         pid_wheel_speed_3},
        pid_steer_angle_{
            pid_steer_angle_0,
            pid_steer_angle_1,
            pid_steer_angle_2,
            pid_steer_angle_3,
        },
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1, pid_steer_speed_2,
                         pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control)
  {
    topic_names_ = topic_names;
    UNUSED(referee);

    /* 舵轮底盘控制线程无条件访问四个舵向电机 */
    ASSERT(motor_steer_0 != nullptr);
    ASSERT(motor_steer_1 != nullptr);
    ASSERT(motor_steer_2 != nullptr);
    ASSERT(motor_steer_3 != nullptr);

    for (int i = 0; i < 4; i++)
    {
      motor_wheel_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_wheel_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_wheel_cmd_[i].torque = 0.0f;
      motor_wheel_cmd_[i].position = 0.0f;
      motor_wheel_cmd_[i].velocity = 0.0f;
      motor_wheel_cmd_[i].kp = 0.0f;
      motor_wheel_cmd_[i].kd = 0.0f;
    }
    for (int i = 0; i < 4; i++)
    {
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
    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Helm* helm, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          helm->mutex_.Lock();
          helm->chassis_event_ = ChassisMode::RELAX;
          helm->mutex_.Unlock();
        },
        this);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Helm* helm, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          helm->mutex_.Lock();
          helm->chassis_event_ = ChassisMode::RELAX;
          helm->mutex_.Unlock();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

  }

  /**
   * @brief 舵轮底盘控制线程函数
   * @param helm Helm对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Helm* helm)
  {
    helm->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber(
        helm->topic_names_.chassis_cmd);
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        helm->topic_names_.chassis_ref);
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    helm->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    constexpr uint32_t CONTROL_PERIOD_MS = 2;
    auto last_time = LibXR::Timebase::GetMilliseconds();

    helm->mutex_.Unlock();
    while (true)
    {
      if (cmd_suber.Available())
      {
        helm->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available())
      {
        helm->referee_chassis_pack_ = referee_suber.GetData();
        helm->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available())
      {
        helm->current_yaw_ = LibXR::CycleValue<float>(yawmotor_angle_suber.GetData());
        helm->delta_yaw_ = -helm->current_yaw_;
        yawmotor_angle_suber.StartWaiting();
      }

      helm->mutex_.Lock();
      helm->Update();
      helm->UpdateCMD();
      helm->topic_delta_yaw_.Publish(helm->delta_yaw_);
      helm->SelfResolution();
      helm->InverseKinematicsSolution();
      helm->OptimizeWheelAngle();
      helm->CompensateWheelSpeed();
      helm->DynamicInverseSolution();
      helm->CalculateMotorCurrent();
      helm->PowerControlUpdate();
      helm->mutex_.Unlock();
      helm->Output();
      const auto NOW = LibXR::Timebase::GetMilliseconds();
      if ((NOW - last_time).ToMillisecond() >= CONTROL_PERIOD_MS)
      {
        /* 超时后让出 CPU，醒来再重建周期，避免持续追赶旧周期而饿死接收线程。 */
        LibXR::Thread::Sleep(1);
        last_time = LibXR::Timebase::GetMilliseconds();
      }
      else
      {
        helm->thread_.SleepUntil(last_time, CONTROL_PERIOD_MS);
      }
    }
  }
  /**
   * @brief 更新电机状态
   * @details 获取当前时间戳并更新所有驱动轮电机的状态
   */
  void Update()
  {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    feedback_valid_ = true;
    for (int i = 0; i < 4; i++)
    {
      const auto WHEEL_STATUS = motor_wheel_[i]->Update();
      const auto STEER_STATUS = motor_steer_[i]->Update();
      motor_wheel_feedback_[i] = motor_wheel_[i]->GetFeedback();
      motor_steer_feedback_[i] = motor_steer_[i]->GetFeedback();
      feedback_valid_ = feedback_valid_ && WHEEL_STATUS == LibXR::ErrorCode::OK &&
                        STEER_STATUS == LibXR::ErrorCode::OK &&
                        motor_wheel_feedback_[i].state != 0 &&
                        motor_steer_feedback_[i].state != 0;
    }
  }

  /**
   * @brief 失去控制处理
   *
   */
  void LostCtrl()
  {
    for (int i = 0; i < 4; i++)
    {
      motor_wheel_[i]->Relax();
      motor_steer_[i]->Relax();
    }
  }

  /**
   * @brief 设置底盘模式 (由 Chassis 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode)
  {
    allocation_braking_priority_ = false;
    mutex_.Lock();
    chassis_event_ = static_cast<ChassisMode>(mode);
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; ++i)
    {
      allocation_previous_nm_[i] = 0.0f;
    }
    for (int i = 0; i < 4; i++)
    {
      pid_wheel_speed_[i].Reset();
      pid_steer_angle_[i].Reset();
      pid_steer_speed_[i].Reset();
    }
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 将归一化 CMD 输入转换为底盘目标速度，单位为 m/s 和 rad/s
   */
  void UpdateCMD()
  {
    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    const float MAX_CHASSIS_OMEGA = GetMaxChassisOmega();

    /* 先生成目标角速度。 */
    switch (chassis_event_)
    {
      case ChassisMode::RELAX:
        target_omega_ = 0.0f;
        break;
      case ChassisMode::INDEPENDENT:
      case ChassisMode::ROTOR:
        target_omega_ = 0.2 * MAX_CHASSIS_OMEGA;
        break;
      case ChassisMode::FOLLOW:
        target_omega_ =
            -pid_follow_.Calculate(0.0f, current_yaw_, dt_) * MAX_CHASSIS_OMEGA;
        break;
      default:
        target_omega_ = 0.0f;
        break;
    }

    /* 再生成目标平移速度，保持原有云台坐标变换。 */
    switch (chassis_event_)
    {
      case ChassisMode::RELAX:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case ChassisMode::INDEPENDENT:
        target_vx_ = cmd_data_.x * MAX_WHEEL_LINEAR_SPEED;
        target_vy_ = cmd_data_.y * MAX_WHEEL_LINEAR_SPEED;
        break;
      case ChassisMode::FOLLOW:
      case ChassisMode::ROTOR:
      {
        const float BETA = 2*current_yaw_;
        const float COS_BETA = cosf(BETA);
        const float SIN_BETA = sinf(BETA);
        target_vx_ =
            (COS_BETA * cmd_data_.x - SIN_BETA * cmd_data_.y) * MAX_WHEEL_LINEAR_SPEED;
        target_vy_ =
            (SIN_BETA * cmd_data_.x + COS_BETA * cmd_data_.y) * MAX_WHEEL_LINEAR_SPEED;
      }
      break;
      default:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
    }
  }

  /**
   * @brief 功率控制更新
   *
   */
  void PowerControlUpdate()
  {
    for (int i = 0; i < 4; i++)
    {
      motor_data_.rotorspeed_rpm_3508[i] = motor_wheel_feedback_[i].velocity;
      motor_data_.output_current_3508[i] = motor_wheel_feedback_[i].torque *
                                           motor_wheel_[i]->GetFeedbackTorquePolarity() *
                                           M3508_NM_TO_LSB_RATIO;

      motor_data_.rotorspeed_rpm_6020[i] = motor_steer_feedback_[i].velocity;
      motor_data_.output_current_6020[i] = motor_steer_feedback_[i].torque *
                                           motor_steer_[i]->GetFeedbackTorquePolarity() *
                                           GM6020_NM_TO_LSB_RATIO;
    }

    /* 第一次设置: 反馈数据用于 RLS 参数估计 */
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020);

    power_control_->CalculatePowerControlParam();

    /* Feedback current and RPM use the same logical direction, including braking. */
    const auto FEEDBACK_POWER =
        feedback_valid_
            ? power_control_->EstimatePower(
                  motor_data_.output_current_3508, motor_data_.rotorspeed_rpm_3508,
                  motor_data_.output_current_6020, motor_data_.rotorspeed_rpm_6020)
            : PowerControl::PowerEstimate{};
    wheel_feedback_power_w_ = FEEDBACK_POWER.valid ? FEEDBACK_POWER.wheel_power_w : 0.0f;

    /* Select the budget before allocation, using the same margin/model as the guard. */
    const float MAX_POWER = GetMaxChassisPower();
    AllocateWheelForces(power_control_->GetDrivePowerModel(),
                        PowerControl::GetDrivePowerBudget(MAX_POWER));

    /* 计算 3508 速度跟踪误差 */
    float speed_error_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
    for (int i = 0; i < 4; i++)
    {
      float actual_speed = motor_reverse_[i] ? -motor_wheel_feedback_[i].velocity
                                             : motor_wheel_feedback_[i].velocity;
      speed_error_3508[i] = target_motor_rpm_[i] - actual_speed;

      motor_data_.output_current_3508[i] =
          std::clamp(wheel_out_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
      motor_data_.output_current_6020[i] =
          std::clamp(steer_out_[i] * GM6020_NM_TO_LSB_RATIO, -16384.0f, 16384.0f);
    }

    /* 计算 6020 舵向速度跟踪误差 */
    float speed_error_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
    for (int i = 0; i < 4; i++)
    {
      speed_error_6020[i] = steer_angle_[i] - motor_steer_feedback_[i].velocity *
                                                  static_cast<float>(LibXR::TWO_PI) /
                                                  60.0f;
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508, speed_error_3508);
    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020, speed_error_6020);

    power_control_->OutputLimit(MAX_POWER);
    power_control_data_ = power_control_->GetPowerControlData();
    UpdateAllocationFeedback();
  }

  /**
   * @brief 舵轮底盘逆动力学解算
   * @details 车体速度 PID 输出加速度，生成整车目标合力/力矩。
   *          功率模型更新后，按实际舵角联合分配最终驱动力矩。
   */
  void DynamicInverseSolution()
  {
    target_ax_ = 0.0f;
    target_ay_ = 0.0f;
    target_alpha_ = 0.0f;
    allocation_target_valid_ = false;
    for (int axis = 0; axis < 3; ++axis)
    {
      requested_wrench_[axis] = 0.0f;
    }
    for (int i = 0; i < 4; i++)
    {
      target_motor_torque_ff_[i] = 0.0f;
    }

    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    const float MAX_CHASSIS_OMEGA = GetMaxChassisOmega();
    if (chassis_event_ == ChassisMode::RELAX ||

        !measured_chassis_velocity_.valid || !std::isfinite(dt_) ||
        !std::isfinite(target_vx_) || !std::isfinite(target_vy_) ||
        !std::isfinite(target_omega_) || dt_ <= 0.0f ||
        !std::isfinite(MAX_WHEEL_LINEAR_SPEED) || MAX_WHEEL_LINEAR_SPEED <= 0.0f ||
        !std::isfinite(MAX_CHASSIS_OMEGA) || MAX_CHASSIS_OMEGA <= 0.0f ||
        !std::isfinite(PARAM.gravity) || PARAM.gravity <= 0.0f ||
        !std::isfinite(PARAM.moment_of_inertia) || PARAM.moment_of_inertia < 0.0f)
    {
      pid_velocity_x_.Reset();
      pid_velocity_y_.Reset();
      pid_omega_.Reset();
      return;
    }

    /* 零速目标时仍闭环底盘速度，以模型力矩制动残余运动。 */
    outer_integral_before_[0] = pid_velocity_x_.GetIntegralError();
    outer_integral_before_[1] = pid_velocity_y_.GetIntegralError();
    outer_integral_before_[2] = pid_omega_.GetIntegralError();
    target_ax_ = pid_velocity_x_.Calculate(target_vx_, measured_chassis_velocity_.x, dt_);
    target_ay_ = pid_velocity_y_.Calculate(target_vy_, measured_chassis_velocity_.y, dt_);
    if (PARAM.moment_of_inertia > 0.0f)
    {
      target_alpha_ =
          pid_omega_.Calculate(target_omega_, measured_chassis_velocity_.omega, dt_);
    }
    else
    {
      pid_omega_.Reset();
    }
    if (!std::isfinite(target_ax_) || !std::isfinite(target_ay_) ||
        !std::isfinite(target_alpha_))
    {
      target_ax_ = 0.0f;
      target_ay_ = 0.0f;
      target_alpha_ = 0.0f;
      pid_velocity_x_.Reset();
      pid_velocity_y_.Reset();
      pid_omega_.Reset();
      return;
    }

    constexpr float GRAVITY_ACCELERATION = 9.80665f;
    const float MASS = PARAM.gravity / GRAVITY_ACCELERATION;
    requested_wrench_[0] = MASS * target_ax_;
    requested_wrench_[1] = MASS * target_ay_;
    requested_wrench_[2] = PARAM.moment_of_inertia * target_alpha_;
    allocation_target_valid_ = std::isfinite(requested_wrench_[0]) &&
                               std::isfinite(requested_wrench_[1]) &&
                               std::isfinite(requested_wrench_[2]);
    if (!allocation_target_valid_)
    {
      for (int axis = 0; axis < 3; ++axis)
      {
        requested_wrench_[axis] = 0.0f;
      }
    }
  }

  /**
   * @brief 舵轮底盘逆运动学解算
   * @details 根据底盘目标速度计算各轮目标舵角与电机转子转速
   */
  void InverseKinematicsSolution()
  {
    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    if (MAX_WHEEL_LINEAR_SPEED <= 1.0e-6f)
    {
      for (int i = 0; i < 4; i++)
      {
        target_motor_rpm_[i] = 0.0f;
        target_steer_angle_[i] = 0.0f;
        steer_angle_held_[i] = false;
      }
      return;
    }
    const float MOTOR_RPM_PER_MPS = motor_max_speed_ / MAX_WHEEL_LINEAR_SPEED;
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        for (int i = 0; i < 4; i++)
        {
          target_motor_rpm_[i] = 0.0f;
          target_steer_angle_[i] = 0.0f;
          steer_angle_held_[i] = false;
        }
        break;
      case (ChassisMode::INDEPENDENT):
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR):
      {
        float max_target_speed = 0.0f;
        for (int i = 0; i < 4; i++)
        {
          const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
          const float WHEEL_VX = target_vx_ - target_omega_ * WHEEL_POSITION.y;
          const float WHEEL_VY = target_vy_ + target_omega_ * WHEEL_POSITION.x;
          const float WHEEL_VECTOR_SPEED =
              sqrtf(WHEEL_VX * WHEEL_VX + WHEEL_VY * WHEEL_VY);
          if (WHEEL_VECTOR_SPEED <= 1.0e-6f)
          {
            /* 仅轮速度近零时锁存舵角，避免零向量的目标舵角无定义。 */
            if (!steer_angle_held_[i])
            {
              target_steer_angle_[i] =
                  LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
              steer_angle_held_[i] = true;
            }
            target_motor_rpm_[i] = 0.0f;
          }
          else
          {
            steer_angle_held_[i] = false;
            /* 沿用原有舵角零位：+y 为 0，正角朝 +x。 */
            target_steer_angle_[i] =
                static_cast<float>(M_PI_2) - atan2f(WHEEL_VY, WHEEL_VX);
            target_motor_rpm_[i] = WHEEL_VECTOR_SPEED * MOTOR_RPM_PER_MPS;
            max_target_speed = fmaxf(max_target_speed, target_motor_rpm_[i]);
          }
        }
        /* 按最大轮速同比例缩放，保留四轮速度比例并限制至 RPM 上限。 */
        if (max_target_speed > motor_max_speed_)
        {
          const float SPEED_SCALE = motor_max_speed_ / max_target_speed;
          for (int i = 0; i < 4; i++)
          {
            target_motor_rpm_[i] *= SPEED_SCALE;
          }
        }
      }
      break;
      default:
        for (int i = 0; i < 4; i++)
        {
          target_motor_rpm_[i] = 0.0f;
          target_steer_angle_[i] = 0.0f;
          steer_angle_held_[i] = false;
        }
        break;
    }
  }

  /**
   * @brief 最优舵角选择
   * @details 舵向差超过 90 度时反转驱动轮，减少舵向转动
   */
  void OptimizeWheelAngle()
  {
    for (int i = 0; i < 4; i++)
    {
      if (steer_angle_held_[i])
      {
        /* 静止时保持锁存的物理舵角，不再做 180 度等价翻转。 */
        motor_reverse_[i] = false;
        continue;
      }
      if (fabs(LibXR::CycleValue(motor_steer_feedback_[i].abs_angle - zero_[i]) -
               target_steer_angle_[i]) > M_PI_2)
      {
        motor_reverse_[i] = true;
      }
      else
      {
        motor_reverse_[i] = false;
      }
    }
  }

  /**
   * @brief 舵向误差对应的驱动轮速度补偿
   */
  void CompensateWheelSpeed()
  {
    /* 最优角确定后按舵向误差抑制轮速，减少转舵时的侧向拖拽。 */
    for (int i = 0; i < 4; i++)
    {
      const LibXR::CycleValue<float> CURRENT_ANGLE(motor_steer_feedback_[i].abs_angle -
                                                   zero_[i]);
      const LibXR::CycleValue<float> OPTIMAL_ANGLE(
          target_steer_angle_[i] + (motor_reverse_[i] ? static_cast<float>(M_PI) : 0.0f));
      const float STEER_ERROR = OPTIMAL_ANGLE - CURRENT_ANGLE;
      const float ALIGNMENT = fmaxf(0.0f, cosf(STEER_ERROR));
      target_motor_rpm_[i] *= ALIGNMENT * ALIGNMENT * ALIGNMENT;
    }
  }

  /**
   * @brief 计算舵向和驱动轮 PID 输出
   */
  void CalculateMotorCurrent()
  {
    /* 轮速 PID 跟踪观测底盘速度在当前实际舵向上的投影。 */
    const float MAX_WHEEL_LINEAR_SPEED = GetMaxWheelLinearSpeed();
    const float MOTOR_RPM_PER_MPS = measured_chassis_velocity_.valid &&
                                            std::isfinite(MAX_WHEEL_LINEAR_SPEED) &&
                                            MAX_WHEEL_LINEAR_SPEED > 1.0e-6f
                                        ? motor_max_speed_ / MAX_WHEEL_LINEAR_SPEED
                                        : 0.0f;
    /* 轮速目标的符号由实际舵向决定，反转状态只影响舵向控制。 */
    for (int i = 0; i < 4; i++)
    {
      float observed_motor_rpm = 0.0f;
      if (MOTOR_RPM_PER_MPS > 0.0f)
      {
        const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
        const float STEER_ANGLE =
            LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
        const float WHEEL_VX = measured_chassis_velocity_.x -
                               measured_chassis_velocity_.omega * WHEEL_POSITION.y;
        const float WHEEL_VY = measured_chassis_velocity_.y +
                               measured_chassis_velocity_.omega * WHEEL_POSITION.x;
        observed_motor_rpm =
            (WHEEL_VX * sinf(STEER_ANGLE) + WHEEL_VY * cosf(STEER_ANGLE)) *
            MOTOR_RPM_PER_MPS;
        if (!std::isfinite(observed_motor_rpm))
        {
          observed_motor_rpm = 0.0f;
        }
      }
      wheel_out_[i] = pid_wheel_speed_[i].Calculate(
          observed_motor_rpm, motor_wheel_feedback_[i].velocity, dt_);
      if (motor_reverse_[i])
      {
        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            LibXR::CycleValue<float>(target_steer_angle_[i] + static_cast<float>(M_PI) +
                                     zero_[i]),
            motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity * static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      }
      else
      {
        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            target_steer_angle_[i] + zero_[i], motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity * static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      }
    }
    /* The wheel PID output is a reference for the later total-torque allocation. */
  }

  /**
   * @brief 输出控制命令到电机
   *
   */
  void Output()
  {
    float output_current_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
    if (power_control_data_.is_power_limited)
    {
      /* 只替换驱动轮输出，舵向力矩仍使用舵向 PID 的结果。 */
      for (int i = 0; i < 4; i++)
      {
        wheel_out_[i] = std::clamp(power_control_data_.new_output_current_3508[i] /
                                       M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                                   -4.4f, 4.4f);
      }
    }

    if (chassis_event_ == (ChassisMode::RELAX))
    {
      LostCtrl();
      for (int i = 0; i < 4; ++i)
      {
        allocation_previous_nm_[i] = 0.0f;
      }
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        motor_wheel_cmd_[i].torque = std::clamp(wheel_out_[i], -4.4f, 4.4f);
        allocation_previous_nm_[i] = motor_wheel_cmd_[i].torque;
        motor_steer_cmd_[i].torque = std::clamp(steer_out_[i], -2.5f, 2.5f);
        output_current_3508[i] =
            std::clamp(motor_wheel_cmd_[i].torque * M3508_NM_TO_LSB_RATIO /
                           motor_wheel_cmd_[i].reduction_ratio,
                       -16384.0f, 16384.0f);
      }
      for (int i = 0; i < 4; i++)
      {
        motor_wheel_[i]->Control(motor_wheel_cmd_[i]);
        motor_steer_[i]->Control(motor_steer_cmd_[i]);
      }
    }

    /* Use the final Helm command after all limits, not the pre-limit request.
     * RELAX uses zero current; the model's RPM-squared loss can remain nonzero. */
    const auto OUTPUT_POWER =
        feedback_valid_
            ? power_control_->EstimatePower(
                  output_current_3508, motor_data_.rotorspeed_rpm_3508,
                  motor_data_.output_current_6020, motor_data_.rotorspeed_rpm_6020)
            : PowerControl::PowerEstimate{};
    wheel_output_power_estimated_w_ =
        OUTPUT_POWER.valid ? OUTPUT_POWER.wheel_power_w : 0.0f;
  }

 private:
  float GetMaxChassisPower()
  {
    const auto NOW_MS = LibXR::Timebase::GetMilliseconds();
    const bool REFEREE_ONLINE = (NOW_MS - referee_last_rx_time_).ToSecondf() <= 1.0f;
    float max_power = static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!REFEREE_ONLINE || !std::isfinite(max_power) || max_power <= 1.0f)
    {
      max_power = HELM_CHASSIS_MAX_POWER;
    }
    if (power_control_->IsOnline() && cmd_data_.self_define == CMD::ChasStat::BOOST)
    {
      const float CAP_ENERGY = power_control_->GetCapEnergy();
      if (CAP_ENERGY > 0.8f)
      {
        max_power += 100.0f;
      }
      else if (CAP_ENERGY > 0.5f)
      {
        max_power += 70.0f;
      }
      else if (CAP_ENERGY > 0.25f)
      {
        max_power += 40.0f;
      }
    }
    return max_power;
  }

  static bool AllocationDeadlineExceeded(void* context)
  {
    const auto* helm = static_cast<const Helm*>(context);
    return (LibXR::Timebase::GetMicroseconds() - helm->allocation_start_time_)
               .ToMicrosecond() >= ALLOCATION_TIME_BUDGET_US;
  }

  /* Row order selects descending task weights, rather than fixed watt quotas. */
  void AllocateWheelForces(const PowerControl::DrivePowerModel& model,
                           float available_power_w)
  {
    allocation_converged_ = false;
    allocation_budget_feasible_ = false;
    allocation_used_fallback_ = false;
    allocation_time_limited_ = false;
    allocation_solver_time_us_ = 0;
    allocation_solver_iterations_ = 0;
    allocation_solver_status_ = 0;
    allocation_available_power_w_ = available_power_w;
    allocation_model_power_w_ = 0.0f;
    allocation_upper_power_w_ = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
      const float CURRENT_PER_NM = M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio;
      const float LIMIT = std::isfinite(CURRENT_PER_NM) && CURRENT_PER_NM > 0.0f
                              ? std::min(4.4f, 16384.0f / CURRENT_PER_NM)
                              : 0.0f;
      wheel_out_[i] =
          std::isfinite(wheel_out_[i]) ? std::clamp(wheel_out_[i], -LIMIT, LIMIT) : 0.0f;
      wheel_pid_torque_nm_[i] = wheel_out_[i];
      target_motor_torque_ff_[i] = 0.0f;
    }
    if (chassis_event_ == ChassisMode::RELAX || !allocation_target_valid_)
    {
      allocation_braking_priority_ = false;
      return;
    }
    if (!model.valid || !feedback_valid_ || !std::isfinite(available_power_w) ||
        available_power_w < 0.0f || !std::isfinite(PARAM.wheel_radius) ||
        PARAM.wheel_radius <= 1.0e-6f || !std::isfinite(PARAM.wheel_to_center) ||
        PARAM.wheel_to_center <= 1.0e-6f || !std::isfinite(PARAM.reduction_ratio) ||
        PARAM.reduction_ratio <= 1.0e-6f)
    {
      ++allocation_solve_count_;
      allocation_solver_status_ = -100;
      allocation_used_fallback_ = true;
      ++allocation_fallback_count_;
      AllocateWheelForcesFallback();
      return;
    }

    allocation_problem_ = {};
    auto& problem = allocation_problem_;
    problem.current_per_nm = M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio;
    problem.kt = model.kt;
    problem.k1 = model.k1;
    problem.k2 = model.k2;
    problem.available_power_w = available_power_w;
    const float TORQUE_LIMIT = std::min(4.4f, 16384.0f / problem.current_per_nm);
    const float SPEED =
        std::hypot(measured_chassis_velocity_.x, measured_chassis_velocity_.y);
    const float TARGET_SPEED = std::hypot(target_vx_, target_vy_);
    float direction_x = 0.0f;
    float direction_y = 1.0f;
    if (SPEED > 0.05f)
    {
      direction_x = measured_chassis_velocity_.x / SPEED;
      direction_y = measured_chassis_velocity_.y / SPEED;
    }
    else if (TARGET_SPEED > 0.05f)
    {
      direction_x = target_vx_ / TARGET_SPEED;
      direction_y = target_vy_ / TARGET_SPEED;
    }
    const float FORCE_PARALLEL =
        direction_x * requested_wrench_[0] + direction_y * requested_wrench_[1];
    const float FORCE_LATERAL =
        -direction_y * requested_wrench_[0] + direction_x * requested_wrench_[1];
    /* Deceleration/stop/reversal cannot be starved by a sustained spin request. */
    if (allocation_braking_priority_)
    {
      allocation_braking_priority_ = SPEED > 0.03f && FORCE_PARALLEL < -0.05f;
    }
    else
    {
      allocation_braking_priority_ = SPEED > 0.06f && FORCE_PARALLEL < -0.15f;
    }
    const int YAW_ROW = allocation_braking_priority_ ? 1 : 0;
    const int LATERAL_ROW = allocation_braking_priority_ ? 2 : 1;
    const int PARALLEL_ROW = allocation_braking_priority_ ? 0 : 2;
    problem.requested[YAW_ROW] =
        PARAM.wheel_radius * requested_wrench_[2] / PARAM.wheel_to_center;
    problem.requested[LATERAL_ROW] = PARAM.wheel_radius * FORCE_LATERAL;
    problem.requested[PARALLEL_ROW] = PARAM.wheel_radius * FORCE_PARALLEL;
    const bool ZERO_COMMAND = fabsf(target_vx_) <= 1.0e-6f &&
                              fabsf(target_vy_) <= 1.0e-6f &&
                              fabsf(target_omega_) <= 1.0e-6f;
    for (int i = 0; i < 4; ++i)
    {
      const WheelPosition POSITION = GetWheelPosition(static_cast<uint8_t>(i));
      const float ANGLE =
          LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
      const float DRIVE_X = sinf(ANGLE);
      const float DRIVE_Y = cosf(ANGLE);
      problem.matrix[YAW_ROW][i] =
          (POSITION.x * DRIVE_Y - POSITION.y * DRIVE_X) / PARAM.wheel_to_center;
      problem.matrix[LATERAL_ROW][i] = -direction_y * DRIVE_X + direction_x * DRIVE_Y;
      problem.matrix[PARALLEL_ROW][i] = direction_x * DRIVE_X + direction_y * DRIVE_Y;
      problem.lower_nm[i] = -TORQUE_LIMIT;
      problem.upper_nm[i] = TORQUE_LIMIT;
      problem.pid_reference_nm[i] = wheel_pid_torque_nm_[i];
      problem.previous_nm[i] = allocation_previous_nm_[i];
      problem.rpm[i] = motor_wheel_feedback_[i].velocity;
      if (steer_angle_held_[i] && !ZERO_COMMAND)
      {
        problem.lower_nm[i] = problem.upper_nm[i] = wheel_pid_torque_nm_[i];
      }
    }
    ++allocation_solve_count_;
    allocation_start_time_ = LibXR::Timebase::GetMicroseconds();
    const auto RESULT = HelmPowerAllocation::solve(problem, allocation_workspace_,
                                                   AllocationDeadlineExceeded, this);
    allocation_solver_time_us_ = static_cast<uint32_t>(
        (LibXR::Timebase::GetMicroseconds() - allocation_start_time_).ToMicrosecond());
    allocation_solver_iterations_ = RESULT.iterations;
    allocation_solver_status_ = RESULT.status;
    allocation_converged_ = RESULT.converged;
    allocation_budget_feasible_ = RESULT.budget_feasible;
    allocation_time_limited_ = RESULT.time_limited;
    allocation_model_power_w_ = RESULT.estimated_power_w;
    allocation_upper_power_w_ = RESULT.upper_power_w;
    if (!RESULT.valid || !RESULT.converged)
    {
      allocation_used_fallback_ = true;
      ++allocation_fallback_count_;
      AllocateWheelForcesFallback();
      return;
    }
    for (int i = 0; i < 4; ++i)
    {
      wheel_out_[i] = RESULT.torque_nm[i];
      target_motor_torque_ff_[i] = wheel_out_[i] - wheel_pid_torque_nm_[i];
    }
  }

  void UpdateAllocationFeedback()
  {
    for (int axis = 0; axis < 3; ++axis)
    {
      allocation_achieved_wrench_[axis] = 0.0f;
    }
    if (!allocation_target_valid_ || chassis_event_ == ChassisMode::RELAX ||
        !std::isfinite(PARAM.wheel_radius) || PARAM.wheel_radius <= 1.0e-6f ||
        !std::isfinite(PARAM.wheel_to_center) || PARAM.wheel_to_center <= 1.0e-6f ||
        !std::isfinite(PARAM.reduction_ratio) || PARAM.reduction_ratio <= 1.0e-6f)
    {
      return;
    }
    const float TORQUE_LIMIT =
        std::min(4.4f, 16384.0f * PARAM.reduction_ratio / M3508_NM_TO_LSB_RATIO);
    const float POWER_RESERVE = std::min(0.05f, 0.001f * allocation_available_power_w_);
    const float POWER_TOLERANCE =
        std::max(1.0e-3f, 1.0e-5f * allocation_available_power_w_);
    bool hard_limited =
        power_control_data_.is_power_limited ||
        (allocation_converged_ &&
         allocation_upper_power_w_ >=
             allocation_available_power_w_ - POWER_RESERVE - POWER_TOLERANCE);
    const bool ZERO_COMMAND = fabsf(target_vx_) <= 1.0e-6f &&
                              fabsf(target_vy_) <= 1.0e-6f &&
                              fabsf(target_omega_) <= 1.0e-6f;
    /* Project onto the attainable wrench space separately from the weighted QP.
     * Its small regularization error must not freeze an otherwise free PID. */
    float basis[3][3] = {};
    unsigned rank = 0;
    float uncontrollable[3] = {requested_wrench_[0], requested_wrench_[1],
                               requested_wrench_[2] / PARAM.wheel_to_center};
    for (int i = 0; i < 4; ++i)
    {
      const float TORQUE = power_control_data_.is_power_limited
                               ? power_control_data_.new_output_current_3508[i] /
                                     M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio
                               : wheel_out_[i];
      const float FORCE = TORQUE / PARAM.wheel_radius;
      const float ANGLE =
          LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
      const float FX = FORCE * sinf(ANGLE);
      const float FY = FORCE * cosf(ANGLE);
      const WheelPosition POSITION = GetWheelPosition(static_cast<uint8_t>(i));
      allocation_achieved_wrench_[0] += FX;
      allocation_achieved_wrench_[1] += FY;
      allocation_achieved_wrench_[2] += POSITION.x * FY - POSITION.y * FX;
      hard_limited =
          hard_limited || fabsf(TORQUE) >= TORQUE_LIMIT - 1.0e-4f * TORQUE_LIMIT;
      float column[3] = {
          sinf(ANGLE), cosf(ANGLE),
          (POSITION.x * cosf(ANGLE) - POSITION.y * sinf(ANGLE)) / PARAM.wheel_to_center};
      if (steer_angle_held_[i] && !ZERO_COMMAND)
      {
        for (unsigned axis = 0; axis < 3; ++axis)
        {
          uncontrollable[axis] -= column[axis] * FORCE;
        }
        continue;
      }
      /* Reorthogonalize once to keep nearly parallel wheel columns stable. */
      for (unsigned pass = 0; pass < 2; ++pass)
      {
        for (unsigned j = 0; j < rank; ++j)
        {
          float dot = 0.0f;
          for (unsigned axis = 0; axis < 3; ++axis)
          {
            dot += column[axis] * basis[j][axis];
          }
          for (unsigned axis = 0; axis < 3; ++axis)
          {
            column[axis] -= dot * basis[j][axis];
          }
        }
      }
      const float NORM_SQUARED =
          column[0] * column[0] + column[1] * column[1] + column[2] * column[2];
      if (rank < 3 && NORM_SQUARED > 1.0e-8f)
      {
        const float INVERSE_NORM = 1.0f / sqrtf(NORM_SQUARED);
        for (unsigned axis = 0; axis < 3; ++axis)
        {
          basis[rank][axis] = column[axis] * INVERSE_NORM;
        }
        ++rank;
      }
    }
    for (unsigned j = 0; j < rank; ++j)
    {
      float dot = 0.0f;
      for (unsigned axis = 0; axis < 3; ++axis)
      {
        dot += uncontrollable[axis] * basis[j][axis];
      }
      for (unsigned axis = 0; axis < 3; ++axis)
      {
        uncontrollable[axis] -= dot * basis[j][axis];
      }
    }
    uncontrollable[2] *= PARAM.wheel_to_center;
    LibXR::PID<float>* pid[3] = {&pid_velocity_x_, &pid_velocity_y_, &pid_omega_};
    for (int axis = 0; axis < 3; ++axis)
    {
      const float DEFICIT = requested_wrench_[axis] - allocation_achieved_wrench_[axis];
      const float I_INCREMENT =
          pid[axis]->I() * (pid[axis]->GetIntegralError() - outer_integral_before_[axis]);
      const float TOLERANCE = std::max(1.0e-3f, 1.0e-3f * fabsf(requested_wrench_[axis]));
      const bool GEOMETRICALLY_BLOCKED = fabsf(uncontrollable[axis]) > TOLERANCE &&
                                         uncontrollable[axis] * I_INCREMENT > 0.0f;
      if ((hard_limited || GEOMETRICALLY_BLOCKED) && fabsf(DEFICIT) > TOLERANCE &&
          DEFICIT * I_INCREMENT > 0.0f)
      {
        pid[axis]->SetIntegralError(outer_integral_before_[axis]);
      }
    }
  }

  static ChassisParam NormalizeChassisGeometry(ChassisParam param)
  {
    /* 仅长宽均未填写时沿用旧正方形布局，保留原轮心距。 */
    if (param.length == 0.0f && param.width == 0.0f &&
        std::isfinite(param.wheel_to_center) && param.wheel_to_center > 0.0f)
    {
      param.length = param.width = param.wheel_to_center * 1.41421356237f;
      return param;
    }
    if (std::isfinite(param.length) && param.length > 0.0f &&
        std::isfinite(param.width) && param.width > 0.0f)
    {
      param.wheel_to_center = hypotf(param.width * 0.5f, param.length * 0.5f);
    }
    else
    {
      /* 不完整或非法尺寸交给现有几何有效性检查处理。 */
      param.length = param.width = param.wheel_to_center = 0.0f;
    }
    return param;
  }

  const ChassisParam PARAM;

  void AllocateWheelForcesFallback()
  {
    const float TORQUE_LIMIT =
        std::isfinite(PARAM.reduction_ratio) && PARAM.reduction_ratio > 1.0e-6f
            ? std::min(4.4f, 16384.0f * PARAM.reduction_ratio / M3508_NM_TO_LSB_RATIO)
            : 0.0f;
    for (int i = 0; i < 4; ++i)
    {
      /* 进入分配器的轮速 PID 修正先遵守物理输出上限。 */
      wheel_out_[i] = std::isfinite(wheel_out_[i])
                          ? std::clamp(wheel_out_[i], -TORQUE_LIMIT, TORQUE_LIMIT)
                          : 0.0f;
      target_motor_torque_ff_[i] = 0.0f;
    }
    if (!allocation_target_valid_ || !std::isfinite(PARAM.wheel_radius) ||
        PARAM.wheel_radius <= 1.0e-6f || !std::isfinite(PARAM.wheel_to_center) ||
        PARAM.wheel_to_center <= 1.0e-6f || TORQUE_LIMIT <= 0.0f)
    {
      return;
    }
    float matrix[3][4] = {};
    float lower[4] = {};
    float upper[4] = {};
    float requested[3] = {
        PARAM.wheel_radius * requested_wrench_[0],
        PARAM.wheel_radius * requested_wrench_[1],
        PARAM.wheel_radius * requested_wrench_[2] / PARAM.wheel_to_center};
    const bool ZERO_CHASSIS_COMMAND = fabsf(target_vx_) <= 1.0e-6f &&
                                      fabsf(target_vy_) <= 1.0e-6f &&
                                      fabsf(target_omega_) <= 1.0e-6f;
    for (int i = 0; i < 4; ++i)
    {
      const WheelPosition POSITION = GetWheelPosition(static_cast<uint8_t>(i));
      const float ANGLE =
          LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
      /* 沿实际舵向直接建立合力矩阵，不重复应用 motor_reverse 或 cos^3。 */
      matrix[0][i] = sinf(ANGLE);
      matrix[1][i] = cosf(ANGLE);
      matrix[2][i] =
          (POSITION.x * matrix[1][i] - POSITION.y * matrix[0][i]) / PARAM.wheel_to_center;
      for (int axis = 0; axis < 3; ++axis)
      {
        requested[axis] -= matrix[axis][i] * wheel_out_[i];
      }
      lower[i] = -TORQUE_LIMIT - wheel_out_[i];
      upper[i] = TORQUE_LIMIT - wheel_out_[i];
      /* 保留零轮速锁舵的语义，停车时仍允许四轮共同制动。 */
      if (steer_angle_held_[i] && !ZERO_CHASSIS_COMMAND)
      {
        lower[i] = upper[i] = 0.0f;
      }
    }
    const auto RESULT = HelmForceAllocation::solve(matrix, requested, lower, upper);
    if (RESULT.valid && RESULT.converged)
    {
      for (int i = 0; i < 4; ++i)
      {
        target_motor_torque_ff_[i] = RESULT.torque_nm[i];
        wheel_out_[i] =
            std::clamp(wheel_out_[i] + RESULT.torque_nm[i], -TORQUE_LIMIT, TORQUE_LIMIT);
      }
    }
    /* 总功率仍交给现有 PowerControl 作为最终限制，不将舵向功率计入预算。 */
  }

  struct WheelPosition
  {
    float x;
    float y;
  };

  WheelPosition GetWheelPosition(uint8_t wheel_index) const
  {
    const float HALF_WIDTH = PARAM.width * 0.5f;
    const float HALF_LENGTH = PARAM.length * 0.5f;
    switch (wheel_index)
    {
      case 0:
        return {-HALF_WIDTH, -HALF_LENGTH};  // Left rear, quadrant III
      case 1:
        return {HALF_WIDTH, -HALF_LENGTH};  // Right rear, quadrant IV
      case 2:
        return {HALF_WIDTH, HALF_LENGTH};  // Right front, quadrant I
      case 3:
        return {-HALF_WIDTH, HALF_LENGTH};  // Left front, quadrant II
      default:
        return {0.0f, 0.0f};
    }
  }

  float GetMaxWheelLinearSpeed() const
  {
    if (PARAM.wheel_radius <= 1.0e-6f || PARAM.reduction_ratio <= 1.0e-6f ||
        motor_max_speed_ <= 1.0e-6f)
    {
      return 0.0f;
    }
    return motor_max_speed_ * static_cast<float>(LibXR::TWO_PI) * PARAM.wheel_radius /
           (60.0f * PARAM.reduction_ratio);
  }

  float GetMaxChassisOmega() const
  {
    /* 纯旋转时轮心线速度为 |omega|*R，R 已是轮心到车体中心的距离。 */
    return PARAM.wheel_to_center > 1.0e-6f
               ? GetMaxWheelLinearSpeed() / PARAM.wheel_to_center
               : 0.0f;
  }

  /**
   * @brief 舵轮底盘正运动学解算
   * @details 根据四轮转速和舵角解算当前底盘速度
   */
  void SelfResolution()
  {
    measured_chassis_velocity_ = {};
    if (!feedback_valid_ || PARAM.wheel_to_center <= 0.0f || PARAM.wheel_radius <= 0.0f ||
        PARAM.reduction_ratio <= 0.0f)
    {
      return;
    }

    const float RPM_TO_LINEAR_SPEED = static_cast<float>(LibXR::TWO_PI) *
                                      PARAM.wheel_radius /
                                      (60.0f * PARAM.reduction_ratio);
    float sum_vx = 0.0f;
    float sum_vy = 0.0f;
    float sum_rotation = 0.0f;
    float sum_radius_squared = 0.0f;
    for (int i = 0; i < 4; i++)
    {
      const WheelPosition WHEEL_POSITION = GetWheelPosition(static_cast<uint8_t>(i));
      /* 舵向零位沿 +y，正角朝 +x；电机 reverse 已在反馈解码中处理。 */
      const float STEER_ANGLE =
          LibXR::CycleValue<float>(motor_steer_feedback_[i].abs_angle - zero_[i]);
      const float WHEEL_SPEED = motor_wheel_feedback_[i].velocity * RPM_TO_LINEAR_SPEED;
      if (!std::isfinite(STEER_ANGLE) || !std::isfinite(WHEEL_SPEED))
      {
        return;
      }
      const float WHEEL_VX = WHEEL_SPEED * sinf(STEER_ANGLE);
      const float WHEEL_VY = WHEEL_SPEED * cosf(STEER_ANGLE);
      sum_vx += WHEEL_VX;
      sum_vy += WHEEL_VY;
      sum_rotation += WHEEL_POSITION.x * WHEEL_VY - WHEEL_POSITION.y * WHEEL_VX;
      sum_radius_squared +=
          WHEEL_POSITION.x * WHEEL_POSITION.x + WHEEL_POSITION.y * WHEEL_POSITION.y;
    }
    if (sum_radius_squared <= 1.0e-12f)
    {
      return;
    }
    measured_chassis_velocity_.x = sum_vx / 4.0f;
    measured_chassis_velocity_.y = sum_vy / 4.0f;
    measured_chassis_velocity_.omega = sum_rotation / sum_radius_squared;
    measured_chassis_velocity_.valid = std::isfinite(measured_chassis_velocity_.x) &&
                                       std::isfinite(measured_chassis_velocity_.y) &&
                                       std::isfinite(measured_chassis_velocity_.omega);
    if (!measured_chassis_velocity_.valid)
    {
      measured_chassis_velocity_ = {};
    }
  }

  ChassisVelocity measured_chassis_velocity_{};
  bool feedback_valid_ = false;
  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;
  float target_ax_ = 0.0f;    /* m/s^2 */
  float target_ay_ = 0.0f;    /* m/s^2 */
  float target_alpha_ = 0.0f; /* rad/s^2 */
  float requested_wrench_[3] = {}; /* Fx(N), Fy(N), Mz(N*m) */
  bool allocation_target_valid_ = false;
  float target_motor_torque_ff_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  /* Per-instance workspace: no heap allocation or large control-thread stack arrays. */
  HelmPowerAllocation::Workspace allocation_workspace_{};
  HelmPowerAllocation::Problem allocation_problem_{};
  static constexpr uint32_t ALLOCATION_TIME_BUDGET_US = 1000;
  LibXR::MicrosecondTimestamp allocation_start_time_ = 0;
  float wheel_pid_torque_nm_[4] = {};
  float allocation_previous_nm_[4] = {};
  float outer_integral_before_[3] = {};
  float allocation_achieved_wrench_[3] = {};
  float allocation_available_power_w_ = 0.0f;
  float allocation_model_power_w_ = 0.0f;
  float allocation_upper_power_w_ = 0.0f;
  uint32_t allocation_solver_time_us_ = 0;
  uint32_t allocation_solver_iterations_ = 0;
  int allocation_solver_status_ = 0;
  uint32_t allocation_solve_count_ = 0;
  uint32_t allocation_fallback_count_ = 0;
  bool allocation_converged_ = false;
  bool allocation_budget_feasible_ = false;
  bool allocation_used_fallback_ = false;
  bool allocation_time_limited_ = false;
  bool allocation_braking_priority_ = false;

  bool motor_reverse_[4]{false, false, false, false};
  LibXR::CycleValue<float> zero_[4] = {1.10063124, 2.05323339, 3.2244277, 0.979446769};
  float current_yaw_ = 0.0f;
  float delta_yaw_ = 0.0f;

  /* 驱动电机转子目标转速 RPM，舵角以 +y 为零位、单位 rad。 */
  float target_motor_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::CycleValue<float> target_steer_angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool steer_angle_held_[4] = {false, false, false, false};

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

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  Motor::MotorCmd motor_wheel_cmd_[4]{};
  Motor::MotorCmd motor_steer_cmd_[4]{};

  MotorData motor_data_ = {};
  /* Positive drive-wheel model power only, excluding steering and static loss.
   * Both estimates are in W, independent of supercapacitor availability. */
  float wheel_feedback_power_w_ = 0.0f;
  float wheel_output_power_estimated_w_ = 0.0f;
  PowerControl* power_control_;
  PowerControlData power_control_data_;

  Referee::ChassisPack referee_chassis_pack_{};
  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;
  ChassisTopicNames topic_names_{};
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  LibXR::Topic topic_delta_yaw_ = LibXR::Topic::CreateTopic<float>("delta_yaw");
  ChassisMode chassis_event_ = ChassisMode::RELAX;
};
