#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>

#include "../PowerLimit.hpp"

using namespace HelmPowerLimit;

static constexpr float KT = 1.99688994e-6f;
static constexpr float K1 = 2.0e-7f;
static constexpr float K2 = 3.0e-7f;

static void require(bool condition, const char* description)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", description);
    std::exit(EXIT_FAILURE);
  }
}

/* 用 double 独立检查输出模型正功率之和，避免仅比较实现自身的统计值。 */
static double positive_power_sum(const float* current, const float* rpm, int count,
                                 float kt = KT, float k1 = K1, float k2 = K2)
{
  double total = 0.0;
  for (int i = 0; i < count; ++i)
  {
    const double CURRENT = current[i];
    const double RPM = rpm[i];
    const double POWER = kt * CURRENT * RPM + k1 * CURRENT * CURRENT + k2 * RPM * RPM;
    total += std::max(0.0, POWER);
  }
  return total;
}

static void check_current_bounds(const float* request,
                                 const ProportionalPowerLimitResult& result, int count)
{
  for (int i = 0; i < count; ++i)
  {
    require(std::isfinite(result.current[i]), "finite current");
    require(std::fabs(result.current[i]) <= std::fabs(request[i]) + 0.001f,
            "limiter cannot increase requested magnitude");
    require(result.current[i] * request[i] >= 0.0f,
            "limiter cannot reverse requested current");
  }
}

static void test_switching_regression()
{
  const float REQUEST[4] = {1834.0f, 262.0f, 262.0f, 262.0f};
  const float RPM[4] = {7000.0f, 7000.0f, 7000.0f, 7000.0f};
  const auto LIMITED = limit_power_proportionally(REQUEST, RPM, 4, 96.0f, KT, K1, K2);
  require(LIMITED.is_power_limited && LIMITED.budget_feasible,
          "original switching example must be limited and feasible");
  check_current_bounds(REQUEST, LIMITED, 4);
  require(positive_power_sum(LIMITED.current, RPM, 4) <= 96.002,
          "original switching example respects power budget");
  for (int i = 0; i < 4; ++i)
  {
    require(std::fabs(LIMITED.current[i] - REQUEST[i]) < 10.0f,
            "slight overload only slightly changes each current");
  }
  std::printf("96 W regression currents: %.3f %.3f %.3f %.3f LSB\n", LIMITED.current[0],
              LIMITED.current[1], LIMITED.current[2], LIMITED.current[3]);

  const float TOTAL = static_cast<float>(positive_power_sum(REQUEST, RPM, 4));
  const auto BELOW =
      limit_power_proportionally(REQUEST, RPM, 4, TOTAL + 0.01f, KT, K1, K2);
  const auto ABOVE =
      limit_power_proportionally(REQUEST, RPM, 4, TOTAL - 0.01f, KT, K1, K2);
  require(!BELOW.is_power_limited && ABOVE.is_power_limited,
          "exercise both sides of switching threshold");
  for (int i = 0; i < 4; ++i)
  {
    require(BELOW.current[i] == REQUEST[i], "unlimited exact passthrough");
    require(std::fabs(ABOVE.current[i] - BELOW.current[i]) < 2.0f,
            "continuous current near power threshold");
  }

  float negative_request[4];
  float negative_rpm[4];
  for (int i = 0; i < 4; ++i)
  {
    negative_request[i] = -REQUEST[i];
    negative_rpm[i] = -RPM[i];
  }
  const auto REVERSED =
      limit_power_proportionally(negative_request, negative_rpm, 4, 96.0f, KT, K1, K2);
  for (int i = 0; i < 4; ++i)
  {
    require(std::fabs(REVERSED.current[i] + LIMITED.current[i]) < 0.001f,
            "positive and negative rotation behave symmetrically");
  }
}

static void test_floors_and_braking()
{
  const float REQUEST[4] = {6000.0f, 100.0f, 100.0f, 100.0f};
  const float RPM[4] = {7000.0f, 7000.0f, 7000.0f, 7000.0f};
  const auto FLOOR_LIMITED =
      limit_power_proportionally(REQUEST, RPM, 4, 65.0f, KT, K1, K2);
  require(FLOOR_LIMITED.budget_feasible, "floor constrained allocation feasible");
  check_current_bounds(REQUEST, FLOOR_LIMITED, 4);
  require(positive_power_sum(FLOOR_LIMITED.current, RPM, 4) <= 65.002,
          "reserve minimum powers before redistributing budget");
  require(positive_power_sum(FLOOR_LIMITED.current, RPM, 4) >= 64.998,
          "use available budget when feasible");

  const float HIGH_RPM[4] = {9000.0f, 9000.0f, 9000.0f, 9000.0f};
  const auto INFEASIBLE =
      limit_power_proportionally(REQUEST, HIGH_RPM, 4, 96.0f, KT, K1, K2);
  require(INFEASIBLE.is_power_limited && !INFEASIBLE.budget_feasible,
          "speed loss above budget explicitly marked infeasible");
  for (int i = 0; i < 4; ++i)
  {
    require(INFEASIBLE.current[i] == 0.0f,
            "infeasible motoring budget gives zero, never forced braking");
  }

  const float MIXED_REQUEST[4] = {6000.0f, -3000.0f, -16384.0f, 0.0f};
  const float MIXED_RPM[4] = {7000.0f, 7000.0f, 100.0f, 0.0f};
  const auto MIXED =
      limit_power_proportionally(MIXED_REQUEST, MIXED_RPM, 4, 40.0f, KT, K1, K2);
  check_current_bounds(MIXED_REQUEST, MIXED, 4);
  require(MIXED.budget_feasible, "mixed drive and braking case feasible");
  require(MIXED.current[1] == MIXED_REQUEST[1],
          "negative model power braking passes through unchanged");
  require(MIXED.current[2] < 0.0f, "positive power braking retains direction");
  require(positive_power_sum(MIXED.current, MIXED_RPM, 4) <= 40.002,
          "mixed drive and braking respects positive power budget");

  require(solve_current_for_power(0.0f, 7000.0f, KT, K1, K2, 262.0f) == 0.0f,
          "low quota cannot synthesize braking from a drive request");
  require(solve_current_for_power(100.0f, 7000.0f, KT, K1, K2, 262.0f) == 262.0f,
          "large quota cannot amplify requested current");
  require(solve_current_for_power(0.0f, 0.0f, KT, K1, K2, 1000.0f) == 0.0f,
          "zero power at standstill gives zero current");
  require(std::fabs(solve_current_for_power(1.0f, 1000.0f, KT, 0.0f, 0.0f, 1000.0f) -
                    1.0f / (KT * 1000.0f)) < 0.001f,
          "linear model is handled without division by zero");
}

static void test_random_invariants()
{
  std::mt19937 random(20261015);
  std::uniform_real_distribution<float> current_distribution(-16384.0f, 16384.0f);
  std::uniform_real_distribution<float> rpm_distribution(-9500.0f, 9500.0f);
  std::uniform_real_distribution<float> budget_distribution(0.0f, 250.0f);
  std::uniform_real_distribution<float> k1_distribution(1.0e-7f, 1.0e-6f);
  std::uniform_real_distribution<float> k2_distribution(1.0e-7f, 5.0e-7f);
  for (int iteration = 0; iteration < 4000; ++iteration)
  {
    const int COUNT = 1 + iteration % POWER_CONTROL_MAX_MOTOR_COUNT;
    const float BUDGET = budget_distribution(random);
    const float COPPER_LOSS = k1_distribution(random);
    const float SPEED_LOSS = k2_distribution(random);
    float request[POWER_CONTROL_MAX_MOTOR_COUNT];
    float rpm[POWER_CONTROL_MAX_MOTOR_COUNT];
    for (int i = 0; i < COUNT; ++i)
    {
      request[i] = current_distribution(random);
      rpm[i] = rpm_distribution(random);
    }
    const auto RESULT = limit_power_proportionally(request, rpm, COUNT, BUDGET, KT,
                                                   COPPER_LOSS, SPEED_LOSS);
    check_current_bounds(request, RESULT, COUNT);
    const double OUTPUT_POWER =
        positive_power_sum(RESULT.current, rpm, COUNT, KT, COPPER_LOSS, SPEED_LOSS);
    if (RESULT.budget_feasible)
    {
      require(OUTPUT_POWER <= BUDGET + 0.002,
              "feasible randomized budget must bound output power");
    }
    else
    {
      require(RESULT.is_power_limited && RESULT.minimum_power_sum > BUDGET,
              "infeasible randomized budget explicitly reported");
      require(std::fabs(OUTPUT_POWER - RESULT.minimum_power_sum) < 0.002,
              "infeasible output minimizes achievable positive model power");
    }
    for (int i = 0; i < COUNT; ++i)
    {
      const double ORIGINAL_POWER =
          KT * static_cast<double>(request[i]) * rpm[i] +
          COPPER_LOSS * static_cast<double>(request[i]) * request[i] +
          SPEED_LOSS * static_cast<double>(rpm[i]) * rpm[i];
      if (ORIGINAL_POWER < -0.001)
      {
        require(RESULT.current[i] == request[i],
                "randomized negative power braking stays intact");
      }
    }
  }
}

static void test_invalid_input()
{
  const float INVALID_RPM[1] = {std::numeric_limits<float>::quiet_NaN()};
  const float REQUEST[1] = {1000.0f};
  const auto INVALID =
      limit_power_proportionally(REQUEST, INVALID_RPM, 1, 96.0f, KT, K1, K2);
  require(
      INVALID.is_power_limited && !INVALID.budget_feasible && INVALID.current[0] == 0.0f,
      "invalid feedback cannot bypass limiter");
}

int main()
{
  test_switching_regression();
  test_floors_and_braking();
  test_random_invariants();
  test_invalid_input();
  std::puts("Power limiter regression and 4000 invariant cases passed.");
}
