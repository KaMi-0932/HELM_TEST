#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 底盘功率控制模块：按功率上限重新分配各电机的输出电流，支持全向轮、麦轮与舵轮底盘 / Chassis power control Module that redistributes the motor output currents under a power limit, for omni-wheel, mecanum and steering-wheel chassis
depends:
- id: QDU-Robomaster/SuperPower
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "PowerLimit.hpp"
#include "RLS.hpp"
#include "SuperPower.hpp"
#include "message.hpp"
#include "thread.hpp"

/// 速度误差总和高于该值时，功率全部按误差分配
/// Above this speed error sum, power is allocated entirely by error
#define ERROR_POWERDISTRIBUTION_SET 20
/// 速度误差总和低于该值时，功率全部按需求比例分配
/// Below this speed error sum, power is allocated entirely by demand ratio
#define POP_POWERDISTRIBUTION 15
/// 底盘限功率余量 (W)
/// Chassis power limit margin (W)
#define CHASSIS_POWER_LIMIT_MARGIN_W 4.0f

/**
 * @brief 计算单个电机的模型预测功率，不含静态损耗。
 *        Compute the model-predicted power of one motor, excluding the static loss.
 *
 * @param current 电机电流。
 *                Motor current.
 * @param rpm 转子转速 (rpm)。
 *            Rotor speed (rpm).
 * @param kt 机械功率系数。
 *           Mechanical power coefficient.
 * @param k1 电流平方项系数。
 *           Coefficient of the squared current term.
 * @param k2 转速平方项系数。
 *           Coefficient of the squared speed term.
 * @return 预测功率 (W)。
 *         Predicted power (W).
 */
inline float calculate_motor_model_power(float current, float rpm, float kt, float k1,
                                         float k2)
{
  return (kt * current * rpm) + (k1 * current * current) + (k2 * rpm * rpm);
}

/**
 * @brief 由目标功率反解电流，结果限幅到 ±16384。
 *        Solve the current from a target power, clamped to ±16384.
 *
 * @param target_power 目标功率 (W)。
 *                     Target power (W).
 * @param rpm 转子转速 (rpm)。
 *            Rotor speed (rpm).
 * @param kt 机械功率系数。
 *           Mechanical power coefficient.
 * @param k1 电流平方项系数。
 *           Coefficient of the squared current term.
 * @param k2 转速平方项系数。
 *           Coefficient of the squared speed term.
 * @param original_current 原始期望电流，其符号决定取二次方程的哪个根。
 *                         Original desired current, whose sign selects the root of the
 *                         quadratic equation.
 * @return 反解得到的电流。
 *         Solved current.
 */
inline float solve_current_for_power(float target_power, float rpm, float kt, float k1,
                                     float k2, float original_current)
{
  float a = k1;
  float b = kt * rpm;
  float c = k2 * rpm * rpm - target_power;
  float delta = b * b - 4.0f * a * c;

  float x3 = -b / (2.0f * a);

  float final_current = 0;

  if (delta < 1e-9f)
  {
    final_current = x3;
  }
  else
  {
    float sqrt_delta = sqrtf(delta);
    float x1 = (-b + sqrt_delta) / (2.0f * a);
    float x2 = (-b - sqrt_delta) / (2.0f * a);

    if (original_current >= 0)
    {
      final_current = x1;
    }
    else
    {
      final_current = x2;
    }
  }

  return std::clamp(final_current, -16384.0f, 16384.0f);
}

/// 最大电机数量
/// Maximum number of motors
static constexpr int POWER_CONTROL_MAX_MOTOR_COUNT = 6;

/**
 * @brief 功率限制后的输出。
 *        Output after the power limit.
 */
struct PowerControlData
{
  /// 3508 功率限制后的电流
  /// Currents of the 3508 motors after the power limit
  float new_output_current_3508[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  /// 6020 功率限制后的电流
  /// Currents of the 6020 motors after the power limit
  float new_output_current_6020[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  /// 是否触发了功率限制
  /// Whether the power limit was applied
  bool is_power_limited = false;
};

/**
 * @brief 底盘功率控制模块：按功率上限重新分配各电机的输出电流，支持全向轮、麦轮与舵轮
 *        底盘。
 *        Chassis power control Module that redistributes the motor output currents
 *        under a power limit, for omni-wheel, mecanum and steering-wheel chassis.
 */
class PowerControl
{
 public:
  /// 最大电机数量
  /// Maximum number of motors
  static constexpr int MAX_MOTOR_COUNT = POWER_CONTROL_MAX_MOTOR_COUNT;

  struct DrivePowerModel
  {
    float kt = 0.0f;
    float k1 = 0.0f;
    float k2 = 0.0f;
    bool valid = false;
  };

  /**
   * @brief 3508 组的分配偏置：先按保底池分配，再分剩余功率。
   *        Allocation bias of the 3508 group: distribute a reserve pool first, then
   *        the remaining power.
   */
  struct AllocationBias3508
  {
    /// 是否启用
    /// Whether enabled
    bool enabled = false;
    /// 可用功率中作为保底池的比例，范围 [0, 1]
    /// Fraction of the available power used as the reserve pool, in [0, 1]
    float reserve_fraction = 0.0f;
    /// 保底池分配权重
    /// Reserve pool allocation weights
    float reserve_weight[MAX_MOTOR_COUNT] = {};
    /// 剩余功率权重的缩放，未配置或不大于 0 时为 1
    /// Scale of the remaining-power weights, 1 when unset or not greater than 0
    float allocation_weight_scale[MAX_MOTOR_COUNT] = {};
  };

  /**
   * @brief 构造 PowerControl。
   *        Construct PowerControl.
   *
   * @param super_power SuperPower 实例，提供实测底盘功率与超级电容在线状态。
   *                    SuperPower instance that provides the measured chassis power
   *                    and the supercapacitor online state.
   * @param is_helm true 为舵轮底盘，仅限制 3508；false 为全向轮或麦轮底盘。
   *                True for a steering-wheel chassis, which limits only the 3508; false
   *                for an omni-wheel or mecanum chassis.
   * @param chassis_static_power_loss 底盘静态功耗 (W)。
   *                                  Chassis static power loss (W).
   * @param motor_count_3508 3508 电机数量，最大 `MAX_MOTOR_COUNT`。
   *                         Number of 3508 motors, at most `MAX_MOTOR_COUNT`.
   * @param motor_count_6020 6020 电机数量，最大 `MAX_MOTOR_COUNT`。
   *                         Number of 6020 motors, at most `MAX_MOTOR_COUNT`.
   */
  PowerControl(SuperPower& super_power, bool is_helm = false,
               float chassis_static_power_loss = 0.0f, int motor_count_3508 = 4,
               int motor_count_6020 = 4)
      : superpower_(&super_power),
        is_helm_(is_helm),
        rls_(1e-5f, 0.99999f),
        k3_chassis_(chassis_static_power_loss),
        motor_count_3508_(motor_count_3508 > MAX_MOTOR_COUNT ? MAX_MOTOR_COUNT
                                                             : motor_count_3508),
        motor_count_6020_(motor_count_6020 > MAX_MOTOR_COUNT ? MAX_MOTOR_COUNT
                                                             : motor_count_6020)
  {
    params_3508_(0, 0) = 2.0e-07f;
    params_3508_(1, 0) = 3.0e-07f;
    k1_3508_ = params_3508_(0, 0);
    k2_3508_ = params_3508_(1, 0);
  }

  /**
   * @brief 写入 3508 电机本周期的期望电流、转子转速和可选的速度跟踪误差。
   *        Write the desired current, rotor speed and optional speed tracking error of
   *        the 3508 motors for this cycle.
   *
   * @param output_current 期望电流数组，长度为 3508 电机数量。
   *                       Desired current array with one entry per 3508 motor.
   * @param rotorspeed_rpm 转子转速数组 (rpm)，长度为 3508 电机数量。
   *                       Rotor speed array (rpm) with one entry per 3508 motor.
   * @param speed_error 速度跟踪误差数组，取绝对值使用；为空时保持上一次的值。
   *                    Speed tracking error array, used as absolute values; the previous
   *                    values are kept when null.
   */
  void SetMotorData3508(float* output_current, float* rotorspeed_rpm,
                        float* speed_error = nullptr)
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    for (int i = 0; i < motor_count_3508_; i++)
    {
      output_current_3508_[i] = output_current[i];
      rotorspeed_rpm_3508_[i] = rotorspeed_rpm[i];
      if (speed_error)
      {
        speed_error_3508_[i] = fabsf(speed_error[i]);
      }
    }
  }

  /**
   * @brief 写入 6020 电机本周期的期望电流、转子转速和可选的速度跟踪误差。
   *        Write the desired current, rotor speed and optional speed tracking error of
   *        the 6020 motors for this cycle.
   *
   * @param output_current 期望电流数组，长度为 6020 电机数量。
   *                       Desired current array with one entry per 6020 motor.
   * @param rotorspeed_rpm 转子转速数组 (rpm)，长度为 6020 电机数量。
   *                       Rotor speed array (rpm) with one entry per 6020 motor.
   * @param speed_error 速度跟踪误差数组，取绝对值使用；为空时保持上一次的值。
   *                    Speed tracking error array, used as absolute values; the previous
   *                    values are kept when null.
   */
  void SetMotorData6020(float* output_current, float* rotorspeed_rpm,
                        float* speed_error = nullptr)
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    for (int i = 0; i < motor_count_6020_; i++)
    {
      output_current_6020_[i] = output_current[i];
      rotorspeed_rpm_6020_[i] = rotorspeed_rpm[i];
      if (speed_error)
      {
        speed_error_6020_[i] = fabsf(speed_error[i]);
      }
    }
  }

  /**
   * @brief 设置 3508 组的分配偏置，仅全向轮与麦轮路径使用。
   *        Set the allocation bias of the 3508 group, used by the omni-wheel and
   *        mecanum path.
   *
   * @param bias 分配偏置。
   *             Allocation bias.
   */
  void SetAllocationBias3508(const AllocationBias3508& bias)
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    allocation_bias_3508_.enabled = bias.enabled;
    allocation_bias_3508_.reserve_fraction =
        std::clamp(bias.reserve_fraction, 0.0f, 1.0f);
    for (int i = 0; i < MAX_MOTOR_COUNT; i++)
    {
      allocation_bias_3508_.reserve_weight[i] = std::max(0.0f, bias.reserve_weight[i]);
      const float WEIGHT_SCALE = bias.allocation_weight_scale[i];
      /* 未配置时保持 1.0 */
      allocation_bias_3508_.allocation_weight_scale[i] =
          WEIGHT_SCALE > 0.0f ? WEIGHT_SCALE : 1.0f;
    }
  }

  /**
   * @brief 读取实测底盘功率，并用 RLS 在线辨识 3508 功率模型的 `k1`、`k2`。
   *        Read the measured chassis power and identify `k1` and `k2` of the 3508 power
   *        model online with RLS.
   */
  void CalculatePowerControlParam()
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    /* 用实测功率修正 3508 功率模型参数 */
    measured_power_ = superpower_->GetChassisPower();
    samples_3508_(0, 0) = 0;
    samples_3508_(1, 0) = 0;
    bool online = superpower_->IsOnline();

    float mechanical_power = 0;

    for (int i = 0; i < motor_count_3508_; i++)
    {
      samples_3508_(0, 0) += output_current_3508_[i] * output_current_3508_[i];
      samples_3508_(1, 0) += rotorspeed_rpm_3508_[i] * rotorspeed_rpm_3508_[i];
      mechanical_power += kt_3508_ * output_current_3508_[i] * rotorspeed_rpm_3508_[i];
    }

    float residual = measured_power_ - mechanical_power - k3_chassis_;

    if (is_helm_)
    {
      float power_6020 = 0;
      for (int i = 0; i < motor_count_6020_; i++)
      {
        power_6020 +=
            calculate_motor_model_power(output_current_6020_[i], rotorspeed_rpm_6020_[i],
                                        kt_6020_, k1_6020_, k2_6020_);
      }
      residual -= power_6020;
    }

    if (residual > 0 && online && measured_power_ > 5.0f)
    {
      params_3508_ = rls_.Update(samples_3508_, residual);
      k1_3508_ = static_cast<float>(fmax(params_3508_(0, 0), 1.0e-07f));
      k2_3508_ = static_cast<float>(fmax(params_3508_(1, 0), 1.0e-07f));
    }
  }

  /**
   * @brief 读取同一时刻的 3508 驱动轮功率模型参数。
   *        Read a consistent snapshot of the 3508 drive-wheel power model.
   */
  DrivePowerModel GetDrivePowerModel()
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    DrivePowerModel model{kt_3508_, k1_3508_, k2_3508_, false};
    model.valid = std::isfinite(model.kt) && std::isfinite(model.k1) &&
                  std::isfinite(model.k2) && model.kt >= 0.0f && model.k1 >= 0.0f &&
                  model.k2 >= 0.0f;
    return model;
  }

  /**
   * @brief 返回扣除余量后的驱动轮功率预算，不含舵向与静态损耗。
   *        Get the drive-only power budget after margin, excluding steering and
   *        static loss. Non-finite limits produce a zero budget.
   */
  static float GetDrivePowerBudget(float max_power)
  {
    return std::isfinite(max_power)
               ? std::max(0.0f, max_power - CHASSIS_POWER_LIMIT_MARGIN_W)
               : 0.0f;
  }

  /**
   * @brief 按功率上限计算限制后的输出电流，结果由 `GetPowerControlData()` 取回。
   *        Compute the limited output currents under the power limit; the result is
   *        fetched with `GetPowerControlData()`.
   *
   * @param max_power 底盘功率上限 (W)。
   *                  Chassis power limit (W).
   */
  void OutputLimit(float max_power)
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (is_helm_)
    {
      OutputLimitHelm(max_power);
    }
    else
    {
      OutputLimitOmni(max_power);
    }
  }

  /**
   * @brief 取回功率限制后的输出。
   *        Fetch the output after the power limit.
   *
   * @return 功率限制后的输出。
   *         Output after the power limit.
   */
  PowerControlData GetPowerControlData()
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    return powercontrol_data_;
  }

  /**
   * @brief 获取最近一次 `CalculatePowerControlParam()` 读到的实测功率。
   *        Get the measured power read by the latest `CalculatePowerControlParam()`.
   *
   * @return 实测功率 (W)。
   *         Measured power (W).
   */
  float GetMeasuredPower() const { return measured_power_; }

  /**
   * @brief 转发 `SuperPower::GetCapEnergy()`。
   *        Forward `SuperPower::GetCapEnergy()`.
   *
   * @return 超级电容能量。
   *         Supercapacitor energy.
   */
  float GetCapEnergy() { return superpower_->GetCapEnergy(); }

  /**
   * @brief 转发 `SuperPower::IsOnline()`。
   *        Forward `SuperPower::IsOnline()`.
   *
   * @return 超级电容在线为 true。
   *         True when the supercapacitor is online.
   */
  bool IsOnline() { return superpower_->IsOnline(); }

  struct PowerEstimate
  {
    float wheel_power_w = 0.0f;
    float steer_power_w = 0.0f;
    float total_power_w = 0.0f;
    bool valid = false;
  };

  /* 无需超电的功率模型估算；忽略负功率，与限功率的消耗统计一致。
   * 总功率包含驱动轮、舵向电机和配置的静态损耗，均非实测功率。 */
  PowerEstimate EstimatePower(const float* current_3508, const float* rpm_3508,
                              const float* current_6020, const float* rpm_6020)
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    PowerEstimate estimate{};
    if ((motor_count_3508_ > 0 && (!current_3508 || !rpm_3508)) ||
        (is_helm_ && motor_count_6020_ > 0 && (!current_6020 || !rpm_6020)) ||
        !std::isfinite(k3_chassis_))
    {
      return estimate;
    }
    for (int i = 0; i < motor_count_3508_; i++)
    {
      if (!std::isfinite(current_3508[i]) || !std::isfinite(rpm_3508[i]))
      {
        return {};
      }
      const float POWER = calculate_motor_model_power(current_3508[i], rpm_3508[i],
                                                      kt_3508_, k1_3508_, k2_3508_);
      if (!std::isfinite(POWER))
      {
        return {};
      }
      estimate.wheel_power_w += std::max(0.0f, POWER);
    }
    if (is_helm_)
    {
      for (int i = 0; i < motor_count_6020_; i++)
      {
        if (!std::isfinite(current_6020[i]) || !std::isfinite(rpm_6020[i]))
        {
          return {};
        }
        const float POWER = calculate_motor_model_power(current_6020[i], rpm_6020[i],
                                                        kt_6020_, k1_6020_, k2_6020_);
        if (!std::isfinite(POWER))
        {
          return {};
        }
        estimate.steer_power_w += std::max(0.0f, POWER);
      }
    }
    estimate.total_power_w =
        estimate.wheel_power_w + estimate.steer_power_w + std::max(0.0f, k3_chassis_);
    estimate.valid = std::isfinite(estimate.total_power_w);
    return estimate.valid ? estimate : PowerEstimate{};
  }

 private:
  void OutputLimitOmni(float max_power)
  {
    float required_power_3508_sum = 0.0f;
    float available_power = max_power - k3_chassis_ - CHASSIS_POWER_LIMIT_MARGIN_W;

    /* 正功率参与分配, 负功率等效增加可用功率 */
    sum_error_ = 0.0f;
    for (int i = 0; i < motor_count_3508_; i++)
    {
      motor_power_3508_[i] = calculate_motor_model_power(
          output_current_3508_[i], rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_, k2_3508_);

      if (motor_power_3508_[i] > 0)
      {
        required_power_3508_sum += motor_power_3508_[i];
        sum_error_ += speed_error_3508_[i];
      }
      else
      {
        available_power -= motor_power_3508_[i];
      }
    }

    if (required_power_3508_sum > available_power)
    {
      powercontrol_data_.is_power_limited = true;

      /* 计算误差置信度: sum_error 越大, 越倾向按误差分配功率 */
      if (sum_error_ > ERROR_POWERDISTRIBUTION_SET)
      {
        error_confidence_ = 1.0f;
      }
      else if (sum_error_ > POP_POWERDISTRIBUTION)
      {
        error_confidence_ = std::clamp(
            (sum_error_ - static_cast<float>(POP_POWERDISTRIBUTION)) /
                static_cast<float>(ERROR_POWERDISTRIBUTION_SET - POP_POWERDISTRIBUTION),
            0.0f, 1.0f);
      }
      else
      {
        error_confidence_ = 0.0f;
      }

      if (!allocation_bias_3508_.enabled)
      {
        /* 默认按误差/需求混合权重分配 */
        for (int i = 0; i < motor_count_3508_; i++)
        {
          if (motor_power_3508_[i] > 0 && required_power_3508_sum > 1e-6f)
          {
            /* 误差权重: 按速度跟踪误差大小分配 */
            float power_weight_error =
                (sum_error_ > 1e-6f) ? (speed_error_3508_[i] / sum_error_) : 0.0f;
            /* 比例权重: 按功率需求比例分配 */
            float power_weight_prop = motor_power_3508_[i] / required_power_3508_sum;
            /* 混合权重 */
            float power_weight = error_confidence_ * power_weight_error +
                                 (1.0f - error_confidence_) * power_weight_prop;
            float power_quota = available_power * power_weight;

            powercontrol_data_.new_output_current_3508[i] =
                solve_current_for_power(power_quota, rotorspeed_rpm_3508_[i], kt_3508_,
                                        k1_3508_, k2_3508_, output_current_3508_[i]);
          }
          else
          {
            powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
          }
        }
      }
      else
      {
        /* 偏置路径: 先保底, 再分剩余功率 */
        float reserve_power_3508[MAX_MOTOR_COUNT] = {};
        float residual_power_3508[MAX_MOTOR_COUNT] = {};
        float weighted_residual_3508[MAX_MOTOR_COUNT] = {};
        const float RESERVE_POWER_POOL =
            std::max(0.0f, available_power) * allocation_bias_3508_.reserve_fraction;
        float reserve_weight_sum = 0.0f;
        float reserved_power_sum = 0.0f;

        for (int i = 0; i < motor_count_3508_; i++)
        {
          if (motor_power_3508_[i] > 0.0f)
          {
            reserve_weight_sum += allocation_bias_3508_.reserve_weight[i];
          }
        }

        if (RESERVE_POWER_POOL > 1e-6f && reserve_weight_sum > 1e-6f)
        {
          /* 保底池只分给正功率电机 */
          for (int i = 0; i < motor_count_3508_; i++)
          {
            if (motor_power_3508_[i] <= 0.0f)
            {
              continue;
            }
            const float RESERVE_QUOTA = RESERVE_POWER_POOL *
                                        allocation_bias_3508_.reserve_weight[i] /
                                        reserve_weight_sum;
            reserve_power_3508[i] = std::min(motor_power_3508_[i], RESERVE_QUOTA);
            reserved_power_sum += reserve_power_3508[i];
          }
        }

        /* 剩余功率再走普通分配 */
        const float REMAINING_AVAILABLE_POWER =
            std::max(0.0f, available_power - reserved_power_sum);
        float residual_required_power_sum = 0.0f;
        for (int i = 0; i < motor_count_3508_; i++)
        {
          residual_power_3508[i] =
              std::max(0.0f, motor_power_3508_[i] - reserve_power_3508[i]);
          residual_required_power_sum += residual_power_3508[i];
        }

        /* 剩余功率权重仍基于误差/需求, 再叠加偏置缩放 */
        float weighted_residual_sum = 0.0f;
        if (residual_required_power_sum > 1e-6f)
        {
          for (int i = 0; i < motor_count_3508_; i++)
          {
            if (residual_power_3508[i] <= 0.0f)
            {
              continue;
            }
            const float POWER_WEIGHT_ERROR =
                (sum_error_ > 1e-6f) ? (speed_error_3508_[i] / sum_error_) : 0.0f;
            const float POWER_WEIGHT_PROP =
                residual_power_3508[i] / residual_required_power_sum;
            const float POWER_WEIGHT = error_confidence_ * POWER_WEIGHT_ERROR +
                                       (1.0f - error_confidence_) * POWER_WEIGHT_PROP;
            weighted_residual_3508[i] =
                POWER_WEIGHT * allocation_bias_3508_.allocation_weight_scale[i];
            weighted_residual_sum += weighted_residual_3508[i];
          }
        }

        for (int i = 0; i < motor_count_3508_; i++)
        {
          if (motor_power_3508_[i] > 0.0f && required_power_3508_sum > 1e-6f)
          {
            float residual_quota = 0.0f;
            if (residual_power_3508[i] > 0.0f && REMAINING_AVAILABLE_POWER > 1e-6f)
            {
              if (weighted_residual_sum > 1e-6f)
              {
                residual_quota = REMAINING_AVAILABLE_POWER * weighted_residual_3508[i] /
                                 weighted_residual_sum;
              }
              else if (residual_required_power_sum > 1e-6f)
              {
                residual_quota = REMAINING_AVAILABLE_POWER * residual_power_3508[i] /
                                 residual_required_power_sum;
              }
            }
            /* 单路电机的最终配额 = 保底配额 + 剩余池配额, 但不超过自身请求 */
            const float POWER_QUOTA =
                std::min(motor_power_3508_[i], reserve_power_3508[i] + residual_quota);
            powercontrol_data_.new_output_current_3508[i] =
                solve_current_for_power(POWER_QUOTA, rotorspeed_rpm_3508_[i], kt_3508_,
                                        k1_3508_, k2_3508_, output_current_3508_[i]);
          }
          else
          {
            powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
          }
        }
      }
    }
    else
    {
      powercontrol_data_.is_power_limited = false;
      for (int i = 0; i < motor_count_3508_; i++)
      {
        powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
      }
    }
  }

  void OutputLimitHelm(float max_power)
  {
    /* 最大功率只约束 3508 驱动轮，保留模型误差与控制延迟余量。 */
    const float AVAILABLE_POWER = GetDrivePowerBudget(max_power);

    /* 6020 舵向电机不参与限幅，始终保留原电流命令。 */
    for (int i = 0; i < motor_count_6020_; i++)
    {
      powercontrol_data_.new_output_current_6020[i] = output_current_6020_[i];
    }

    const auto LIMITED = HelmPowerLimit::limit_power_proportionally(
        output_current_3508_, rotorspeed_rpm_3508_, motor_count_3508_, AVAILABLE_POWER,
        kt_3508_, k1_3508_, k2_3508_);
    powercontrol_data_.is_power_limited = LIMITED.is_power_limited;
    for (int i = 0; i < motor_count_3508_; i++)
    {
      motor_power_3508_[i] = LIMITED.requested_power[i];
      powercontrol_data_.new_output_current_3508[i] = LIMITED.current[i];
    }
  }

 private:
  LibXR::Mutex mutex_;
  SuperPower* superpower_;
  bool is_helm_;
  RLS<2> rls_; /* 在线辨识 3508 二次损耗模型参数 */
  PowerControlData powercontrol_data_;
  float k3_chassis_; /* 底盘静态功耗 */

  float error_confidence_ = 0.0f; /* 误差置信度 */
  float sum_error_ = 0.0f;

  float speed_error_3508_[MAX_MOTOR_COUNT] = {}; /* 3508速度跟踪误差 */
  float speed_error_6020_[MAX_MOTOR_COUNT] = {}; /* 6020速度跟踪误差 */
  AllocationBias3508 allocation_bias_3508_{};    /* 3508 组动态分配偏置 */

  int motor_count_3508_; /* 3508电机数目 */
  int motor_count_6020_; /* 6020电机数目 */

  float kt_3508_ = 1.99688994e-6f;
  float k1_3508_ = 0;
  float k2_3508_ = 0;
  Eigen::Matrix<float, 2, 1> samples_3508_; /* RLS 输入: [i^2_sum, rpm^2_sum] */
  Eigen::Matrix<float, 2, 1> params_3508_;  /* RLS 输出: [k1, k2] */

  float output_current_3508_[MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_3508_[MAX_MOTOR_COUNT] = {};
  float motor_power_3508_[MAX_MOTOR_COUNT] = {};

  float kt_6020_ = 1.42074505e-5f;
  float k1_6020_ = 6.4276e-7f;
  float k2_6020_ = 1.0e-10f;

  float output_current_6020_[MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_6020_[MAX_MOTOR_COUNT] = {};

  float measured_power_ = 0.0f;
};
