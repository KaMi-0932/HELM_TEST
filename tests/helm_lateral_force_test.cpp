#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "../HelmLateralForce.hpp"

static void require(bool condition, const char* description)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", description);
    std::exit(EXIT_FAILURE);
  }
}

static bool near(float actual, float expected)
{
  return std::fabs(actual - expected) < 1.0e-5f;
}

int main()
{
  const auto STRAIGHT =
      HelmLateralForce::compensate_body_acceleration(1.0f, -2.0f, 0.0f, 3.0f, 0.0f);
  require(STRAIGHT.valid && near(STRAIGHT.x, 1.0f) && near(STRAIGHT.y, -2.0f),
          "straight motion leaves PID accelerations unchanged");

  const auto LEFT_TURN =
      HelmLateralForce::compensate_body_acceleration(0.0f, 0.0f, 0.0f, 2.0f, 1.5f);
  require(LEFT_TURN.valid && near(LEFT_TURN.x, -3.0f) && near(LEFT_TURN.y, 0.0f),
          "forward motion and positive yaw require leftward force");

  const auto RIGHT_TURN =
      HelmLateralForce::compensate_body_acceleration(0.0f, 0.0f, 0.0f, 2.0f, -1.5f);
  require(RIGHT_TURN.valid && near(RIGHT_TURN.x, 3.0f),
          "reversing yaw reverses the lateral compensation");

  const auto SIDEWAYS =
      HelmLateralForce::compensate_body_acceleration(0.5f, 0.0f, 2.0f, 0.0f, 1.5f);
  require(SIDEWAYS.valid && near(SIDEWAYS.x, 0.5f) && near(SIDEWAYS.y, 3.0f),
          "sideways motion adds the matching forward force");

  const auto SPIN =
      HelmLateralForce::compensate_body_acceleration(0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
  require(SPIN.valid && near(SPIN.x, 0.0f) && near(SPIN.y, 0.0f),
          "pure spin adds no translational force");

  const auto INVALID = HelmLateralForce::compensate_body_acceleration(
      0.0f, 0.0f, 0.0f, std::numeric_limits<float>::infinity(), 1.0f);
  require(!INVALID.valid, "non-finite velocity is rejected");

  const auto OVERFLOW = HelmLateralForce::compensate_body_acceleration(
      0.0f, 0.0f, std::numeric_limits<float>::max(), 0.0f,
      std::numeric_limits<float>::max());
  require(!OVERFLOW.valid, "overflowed compensation is rejected");

  std::puts("Helm lateral-force compensation tests passed.");
}
