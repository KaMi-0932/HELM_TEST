#pragma once

#include <algorithm>
#include <cmath>

// Keep the Helm limiter independent of the existing omni-wheel allocation solver.
namespace HelmPowerLimit
{
static constexpr int POWER_CONTROL_MAX_MOTOR_COUNT = 6;

/* 电流单位为电机控制 LSB，转速为 rpm，模型功率为 W。 */
inline float calculate_motor_model_power(float current, float rpm, float kt, float k1,
                                         float k2)
{
  return kt * current * rpm + k1 * current * current + k2 * rpm * rpm;
}

/* 只在零电流至原请求电流之间寻找模型功率最低点，保留请求方向。 */
inline float minimum_model_power_current(float original_current, float rpm, float kt,
                                         float k1)
{
  const float REQUEST = std::clamp(original_current, -16384.0f, 16384.0f);
  if (k1 > 0.0f)
  {
    return std::clamp(-kt * rpm / (2.0f * k1), std::min(0.0f, REQUEST),
                      std::max(0.0f, REQUEST));
  }
  return kt * rpm * REQUEST < 0.0f ? REQUEST : 0.0f;
}

/* 取允许区间内最接近原请求的解；配额不可行时取区间内最低功率点。
 * 原请求制动时也保留其电流方向，不因选择二次方程的另一根而反向。 */
inline float solve_current_for_power(float target_power, float rpm, float kt, float k1,
                                     float k2, float original_current)
{
  if (!std::isfinite(target_power) || !std::isfinite(rpm) ||
      !std::isfinite(original_current) || !std::isfinite(kt) || !std::isfinite(k1) ||
      !std::isfinite(k2) || kt < 0.0f || k1 < 0.0f || k2 < 0.0f)
  {
    return 0.0f;
  }
  const float REQUEST = std::clamp(original_current, -16384.0f, 16384.0f);
  const float TARGET = std::max(0.0f, target_power);
  const float REQUESTED_POWER = calculate_motor_model_power(REQUEST, rpm, kt, k1, k2);
  if (!std::isfinite(REQUESTED_POWER))
  {
    return 0.0f;
  }
  if (REQUESTED_POWER <= TARGET)
  {
    return REQUEST;
  }

  const float MINIMUM_CURRENT = minimum_model_power_current(REQUEST, rpm, kt, k1);
  const float MINIMUM_POWER =
      calculate_motor_model_power(MINIMUM_CURRENT, rpm, kt, k1, k2);
  if (MINIMUM_POWER >= TARGET)
  {
    return MINIMUM_CURRENT;
  }

  /* 在请求方向的电流幅值坐标上解方程，取较大的根。 */
  const float DIRECTION = REQUEST < 0.0f ? -1.0f : 1.0f;
  const float B = kt * rpm * DIRECTION;
  const float C = k2 * rpm * rpm - TARGET;
  float magnitude = 0.0f;
  if (k1 == 0.0f)
  {
    magnitude = -C / B;
  }
  else
  {
    const float SQRT_DELTA = std::sqrt(std::max(0.0f, B * B - 4.0f * k1 * C));
    /* B >= 0 时使用等价式，避免相近数相减损失精度。 */
    magnitude =
        B >= 0.0f ? -2.0f * C / (B + SQRT_DELTA) : (-B + SQRT_DELTA) / (2.0f * k1);
  }
  return DIRECTION *
         std::clamp(magnitude, std::fabs(MINIMUM_CURRENT), std::fabs(REQUEST));
}

struct ProportionalPowerLimitResult
{
  float current[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float requested_power[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float minimum_power_sum = 0.0f;
  /* 非法输入默认零输出，并保持限幅状态，避免调用方透传请求。 */
  bool is_power_limited = true;
  bool budget_feasible = false;
};

/* 按请求正功率比例分配。某轮配额达到最低可行功率后，固定该轮，
 * 剩余预算继续按其余轮的请求比例分配，避免转速损耗造成配额不可行。 */
inline ProportionalPowerLimitResult limit_power_proportionally(
    const float* requested_current, const float* rpm, int motor_count,
    float available_power, float kt, float k1, float k2)
{
  ProportionalPowerLimitResult result;
  if (motor_count < 0 || motor_count > POWER_CONTROL_MAX_MOTOR_COUNT ||
      (motor_count > 0 && (!requested_current || !rpm)) ||
      !std::isfinite(available_power) || !std::isfinite(kt) || !std::isfinite(k1) ||
      !std::isfinite(k2) || kt < 0.0f || k1 < 0.0f || k2 < 0.0f)
  {
    return result;
  }

  const float AVAILABLE_POWER = std::max(0.0f, available_power);
  float minimum_current[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float minimum_power[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float required_power_sum = 0.0f;
  bool request_clamped = false;
  for (int i = 0; i < motor_count; ++i)
  {
    if (!std::isfinite(requested_current[i]) || !std::isfinite(rpm[i]))
    {
      return ProportionalPowerLimitResult{};
    }
    result.current[i] = std::clamp(requested_current[i], -16384.0f, 16384.0f);
    request_clamped = request_clamped || result.current[i] != requested_current[i];
    result.requested_power[i] =
        calculate_motor_model_power(result.current[i], rpm[i], kt, k1, k2);
    if (!std::isfinite(result.requested_power[i]))
    {
      return ProportionalPowerLimitResult{};
    }
    minimum_current[i] = result.current[i];
    if (result.requested_power[i] > 0.0f)
    {
      required_power_sum += result.requested_power[i];
      minimum_current[i] = minimum_model_power_current(result.current[i], rpm[i], kt, k1);
      minimum_power[i] = std::max(
          0.0f, calculate_motor_model_power(minimum_current[i], rpm[i], kt, k1, k2));
      result.minimum_power_sum += minimum_power[i];
    }
  }

  if (!std::isfinite(required_power_sum) || !std::isfinite(result.minimum_power_sum))
  {
    return ProportionalPowerLimitResult{};
  }
  result.budget_feasible = result.minimum_power_sum <= AVAILABLE_POWER;
  result.is_power_limited = request_clamped || required_power_sum > AVAILABLE_POWER;
  if (required_power_sum <= AVAILABLE_POWER)
  {
    return result;
  }
  if (!result.budget_feasible)
  {
    for (int i = 0; i < motor_count; ++i)
    {
      result.current[i] = minimum_current[i];
    }
    return result;
  }

  bool fixed[POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float fixed_power_sum = 0.0f;
  float remaining_request_sum = required_power_sum;
  float power_scale = 0.0f;
  for (int iteration = 0; iteration < motor_count; ++iteration)
  {
    power_scale =
        remaining_request_sum > 0.0f
            ? std::clamp((AVAILABLE_POWER - fixed_power_sum) / remaining_request_sum,
                         0.0f, 1.0f)
            : 0.0f;
    bool fixed_another = false;
    for (int i = 0; i < motor_count; ++i)
    {
      if (!fixed[i] && result.requested_power[i] > 0.0f &&
          power_scale * result.requested_power[i] < minimum_power[i])
      {
        fixed[i] = true;
        fixed_power_sum += minimum_power[i];
        remaining_request_sum -= result.requested_power[i];
        fixed_another = true;
      }
    }
    if (!fixed_another)
    {
      break;
    }
  }

  for (int i = 0; i < motor_count; ++i)
  {
    if (result.requested_power[i] > 0.0f)
    {
      const float POWER_QUOTA =
          fixed[i] ? minimum_power[i] : power_scale * result.requested_power[i];
      result.current[i] =
          solve_current_for_power(POWER_QUOTA, rpm[i], kt, k1, k2, result.current[i]);
    }
  }
  return result;
}

}  // namespace HelmPowerLimit
