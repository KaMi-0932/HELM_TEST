#pragma once

#include <cmath>

namespace HelmLateralForce
{
struct BodyAcceleration
{
  float x = 0.0f;
  float y = 0.0f;
  bool valid = false;
};

/* x 向右、y 向前、正 yaw 绕 +z。车体系速度导数换算到惯性加速度：
 * a = dv_body/dt + omega x v_body。补偿力由调用者乘车体质量得到。 */
inline BodyAcceleration compensate_body_acceleration(float acceleration_x,
                                                     float acceleration_y,
                                                     float velocity_x,
                                                     float velocity_y,
                                                     float yaw_rate)
{
  if (!std::isfinite(acceleration_x) || !std::isfinite(acceleration_y) ||
      !std::isfinite(velocity_x) || !std::isfinite(velocity_y) ||
      !std::isfinite(yaw_rate))
  {
    return {};
  }

  const float COMPENSATED_X = acceleration_x - yaw_rate * velocity_y;
  const float COMPENSATED_Y = acceleration_y + yaw_rate * velocity_x;
  if (!std::isfinite(COMPENSATED_X) || !std::isfinite(COMPENSATED_Y))
  {
    return {};
  }
  return {.x = COMPENSATED_X, .y = COMPENSATED_Y, .valid = true};
}
}  // namespace HelmLateralForce
