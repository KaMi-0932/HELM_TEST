#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

#include "../../PowerControl/PowerLimit.hpp"
#include "../HelmPowerAllocation.hpp"

namespace
{
using HelmPowerAllocation::Problem;
using HelmPowerAllocation::Result;
using HelmPowerAllocation::Workspace;

constexpr float TOLERANCE = 3.0e-3f;
constexpr float CURRENT_PER_NM = 8192.0f;
constexpr float CURRENT_SQUARED = CURRENT_PER_NM * CURRENT_PER_NM;

void require(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

void near(float actual, float expected, const char* message, float tolerance = TOLERANCE)
{
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance)
  {
    std::fprintf(stderr, "FAIL: %s: actual %.9g, expected %.9g\n", message,
                 static_cast<double>(actual), static_cast<double>(expected));
    std::exit(1);
  }
}

Problem base_problem()
{
  Problem problem{};
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.lower_nm[wheel] = -2.0f;
    problem.upper_nm[wheel] = 2.0f;
  }
  problem.current_per_nm = CURRENT_PER_NM;
  problem.kt = 0.0f;
  problem.k1 = 1.0f / CURRENT_SQUARED;
  problem.k2 = 0.0f;
  problem.available_power_w = 16.0f;
  return problem;
}

float actual_power(const Problem& problem, const Result& result)
{
  float total = 0.0f;
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    const float CURRENT = problem.current_per_nm * result.torque_nm[wheel];
    const float RPM = problem.rpm[wheel];
    const float POWER = problem.kt * CURRENT * RPM + problem.k1 * CURRENT * CURRENT +
                        problem.k2 * RPM * RPM;
    total += std::fmax(0.0f, POWER);
  }
  return total;
}

void check_solution(const Problem& problem, const Result& result)
{
  require(result.valid, "finite valid result");
  require(result.converged, "solver converged");
  require(result.budget_feasible, "power budget feasible");
  require(!result.time_limited, "completed without abort");
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    require(result.torque_nm[wheel] >= problem.lower_nm[wheel] - 1.0e-5f &&
                result.torque_nm[wheel] <= problem.upper_nm[wheel] + 1.0e-5f,
            "final torque respects its bounds");
  }
  near(result.estimated_power_w, actual_power(problem, result),
       "reported physical power matches independent model", 1.0e-2f);
  require(result.estimated_power_w <= result.upper_power_w + 1.0e-2f,
          "piecewise envelope does not underestimate physical model");
  require(result.upper_power_w <= problem.available_power_w + 1.0e-2f,
          "conservative envelope remains within budget");
  for (int row = 0; row < 3; ++row)
  {
    float achieved = 0.0f;
    for (int wheel = 0; wheel < 4; ++wheel)
    {
      achieved += problem.matrix[row][wheel] * result.torque_nm[wheel];
    }
    near(result.achieved[row], achieved, "reported achieved wrench", 1.0e-2f);
  }
}

void pure_straight()
{
  auto problem = base_problem();
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.matrix[2][wheel] = 1.0f;
  }
  problem.requested[2] = 4.0f;
  Workspace workspace{};
  const auto RESULT = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, RESULT);
  near(RESULT.achieved[2], 4.0f, "straight demand tracked with regularization", 0.2f);
}

void pure_rotation()
{
  auto problem = base_problem();
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.matrix[0][wheel] = (wheel % 2 == 0) ? -1.0f : 1.0f;
    problem.matrix[2][wheel] = 1.0f;
  }
  problem.requested[0] = 4.0f;
  Workspace workspace{};
  const auto RESULT = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, RESULT);
  near(RESULT.achieved[0], 4.0f, "pure rotation demand reached");
  near(RESULT.achieved[2], 0.0f, "pure rotation has no translation");
}

void weighted_priorities()
{
  auto problem = base_problem();
  for (int row = 0; row < 3; ++row)
  {
    problem.matrix[row][row] = 1.0f;
    problem.requested[row] = 1.0f;
  }
  Workspace workspace{};
  problem.available_power_w = 1.0f;
  auto result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  require(result.achieved[0] > 0.85f && result.achieved[1] < 0.5f &&
              std::fabs(result.achieved[2]) < 0.05f,
          "yaw receives more tracking effort than both translations");

  problem.available_power_w = 2.0f;
  result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  require(result.achieved[0] > 0.98f && result.achieved[1] > 0.85f &&
              result.achieved[2] < 0.45f,
          "longitudinal gives way to yaw and lateral");

  problem.available_power_w = 3.0f;
  result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  for (int row = 0; row < 3; ++row)
  {
    near(result.achieved[row], 1.0f, "all tasks tracked when feasible", 0.02f);
  }
}

void task_scaling_and_unreachable_yaw()
{
  auto problem = base_problem();
  problem.matrix[0][0] = 1.0e-3f;
  problem.matrix[1][1] = 1.0e3f;
  problem.matrix[2][2] = 1.0f;
  problem.requested[0] = 0.01f;
  problem.requested[1] = 1000.0f;
  problem.requested[2] = 1.0f;
  problem.available_power_w = 8.0f;
  Workspace workspace{};
  const auto RESULT = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, RESULT);
  require(std::fabs(RESULT.torque_nm[0]) < 0.5f,
          "near-singular yaw authority does not create excessive torque");
  near(RESULT.torque_nm[1], 1.0f, "independent row scales do not alter priority");
  near(RESULT.torque_nm[2], 1.0f, "lower priority still uses remaining budget", 0.03f);
}

void large_request_scaling_regression()
{
  auto problem = base_problem();
  problem.current_per_nm = 52437.5f / 14.0f;
  problem.kt = 1.99688994e-6f;
  problem.k1 = 2.0e-7f;
  problem.k2 = 3.0e-7f;
  problem.available_power_w = 96.0f;
  const float TORQUE_LIMIT = 16384.0f / problem.current_per_nm;
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.lower_nm[wheel] = -TORQUE_LIMIT;
    problem.upper_nm[wheel] = TORQUE_LIMIT;
    problem.matrix[0][wheel] = (wheel == 0 || wheel == 3) ? -0.70710678118f
                                                                      : 0.70710678118f;
    problem.matrix[2][wheel] = 1.0f;
  }
  Workspace reused{};
  constexpr float REQUESTS[] = {10.0f, 17.5f, 25.0f, 40.0f, 80.0f};
  for (float yaw_sign : {-1.0f, 1.0f})
  {
    // Hold every other task fixed while crossing the old |request|-based
    // scaling boundary. This is a specific regression, not a general claim
    // of axis-wise monotonicity for every coupled constrained allocation.
    problem.requested[0] = yaw_sign * 2.0f;
    for (float direction : {-1.0f, 1.0f})
    {
      for (float& rpm : problem.rpm)
      {
        rpm = direction * 6000.0f;
      }
      float previous_achieved = -std::numeric_limits<float>::infinity();
      for (float request : REQUESTS)
      {
        problem.requested[2] = direction * request;
        const auto RESULT = HelmPowerAllocation::solve(problem, reused);
        check_solution(problem, RESULT);
        const float ACHIEVED = direction * RESULT.achieved[2];
        require(ACHIEVED >= previous_achieved - 2.0e-3f,
                "larger longitudinal demand does not weaken this fixed-yaw fixture");
        previous_achieved = ACHIEVED;

        Workspace cold{};
        const auto COLD = HelmPowerAllocation::solve(problem, cold);
        check_solution(problem, COLD);
        for (int wheel = 0; wheel < 4; ++wheel)
        {
          near(RESULT.torque_nm[wheel], COLD.torque_nm[wheel],
               "large requests agree after workspace sign and demand changes");
        }
      }
    }
  }
}

void fixed_wheels_and_zero_span()
{
  auto problem = base_problem();
  problem.matrix[0][0] = 1.0f;
  problem.matrix[0][1] = 1.0f;
  problem.requested[0] = 2.0f;
  problem.lower_nm[0] = problem.upper_nm[0] = 0.5f;
  problem.lower_nm[2] = problem.upper_nm[2] = 0.0f;
  Workspace workspace{};
  auto result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  near(result.torque_nm[0], 0.5f, "fixed torque preserved");
  near(result.torque_nm[1], 1.5f, "other wheel fills remaining yaw demand", 0.01f);
  near(result.torque_nm[2], 0.0f, "fixed zero torque preserved");
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.lower_nm[wheel] = problem.upper_nm[wheel] = 0.0f;
  }
  result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  near(result.achieved[0], 0.0f, "all-fixed infeasible task stays finite");
}

void singular_and_zero_budget()
{
  auto problem = base_problem();
  problem.requested[0] = 10.0f;
  problem.requested[1] = -10.0f;
  problem.requested[2] = 5.0f;
  Workspace workspace{};
  auto result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  near(result.estimated_power_w, 0.0f, "unreachable wrench does not waste power");
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    near(result.torque_nm[wheel], 0.0f, "singular zero matrix produces zero torque");
  }
  problem.matrix[0][0] = 1.0f;
  problem.available_power_w = 0.0f;
  result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  near(result.estimated_power_w, 0.0f, "zero budget produces no model consumption");
  near(result.achieved[0], 0.0f, "zero budget does not invent yaw ability");
}

void braking_is_not_a_power_credit()
{
  auto problem = base_problem();
  problem.kt = 2.0f / CURRENT_PER_NM;
  problem.k1 = 0.1f / CURRENT_SQUARED;
  problem.k2 = 0.25f;
  problem.lower_nm[0] = problem.upper_nm[0] = -1.0f;
  problem.lower_nm[1] = problem.upper_nm[1] = 1.0f;
  problem.lower_nm[2] = problem.upper_nm[2] = 0.0f;
  problem.lower_nm[3] = problem.upper_nm[3] = 0.0f;
  problem.rpm[0] = problem.rpm[1] = 2.0f;
  problem.available_power_w = 5.2f;
  Workspace workspace{};
  auto result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  near(result.estimated_power_w, 5.1f, "negative wheel power does not offset positive");
  problem.available_power_w = 5.0f;
  result = HelmPowerAllocation::solve(problem, workspace);
  require(!result.budget_feasible, "fixed positive power exceeding budget is infeasible");

  problem.rpm[0] = problem.rpm[1] = -2.0f;
  problem.available_power_w = 5.2f;
  result = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, result);
  near(result.estimated_power_w, 5.1f, "negative rpm preserves physical power polarity");
}

void speed_loss_can_make_budget_infeasible()
{
  auto problem = base_problem();
  problem.kt = 0.0f;
  problem.k2 = 1.0f;
  problem.available_power_w = 3.9f;
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.rpm[wheel] = 1.0f;
  }
  Workspace workspace{};
  const auto RESULT = HelmPowerAllocation::solve(problem, workspace);
  require(!RESULT.budget_feasible, "unavoidable speed loss exceeds budget");
}

void invalid_inputs()
{
  const auto ORIGINAL = base_problem();
  Workspace workspace{};
  auto problem = ORIGINAL;
  problem.matrix[0][0] = std::numeric_limits<float>::quiet_NaN();
  require(!HelmPowerAllocation::solve(problem, workspace).valid, "NaN matrix rejected");
  problem = ORIGINAL;
  problem.requested[1] = std::numeric_limits<float>::infinity();
  require(!HelmPowerAllocation::solve(problem, workspace).valid,
          "infinite demand rejected");
  problem = ORIGINAL;
  problem.lower_nm[0] = 3.0f;
  require(!HelmPowerAllocation::solve(problem, workspace).valid,
          "reversed bounds rejected");
  problem = ORIGINAL;
  problem.current_per_nm = 0.0f;
  require(!HelmPowerAllocation::solve(problem, workspace).valid,
          "invalid current conversion rejected");
  problem = ORIGINAL;
  problem.k1 = -1.0f;
  require(!HelmPowerAllocation::solve(problem, workspace).valid,
          "nonconvex copper-loss coefficient rejected");
  problem = ORIGINAL;
  problem.available_power_w = -1.0f;
  require(!HelmPowerAllocation::solve(problem, workspace).valid,
          "negative power budget rejected");
}

bool abort_now(void*) { return true; }

void wheel_pid_and_final_power_guard()
{
  auto problem = base_problem();
  Workspace workspace{};
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.matrix[0][wheel] = (wheel % 2 == 0) ? -1.0f : 1.0f;
    problem.matrix[2][wheel] = 1.0f;
  }
  problem.pid_reference_nm[0] = problem.pid_reference_nm[3] = 1.0f;
  problem.pid_reference_nm[1] = problem.pid_reference_nm[2] = -1.0f;
  const auto CORRECTION = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, CORRECTION);
  require(CORRECTION.torque_nm[0] > 0.9f,
          "wheel PID retains useful differential correction");
  near(CORRECTION.achieved[0], 0.0f, "PID preference preserves yaw", 0.005f);
  near(CORRECTION.achieved[2], 0.0f, "PID preference preserves translation", 0.03f);

  problem.current_per_nm = 52437.5f / 14.0f;
  problem.kt = 1.99688994e-6f;
  problem.k1 = 2.0e-7f;
  problem.k2 = 3.0e-7f;
  problem.available_power_w = 96.0f;
  problem.requested[0] = 2.0f;
  problem.requested[2] = 12.0f;
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    problem.lower_nm[wheel] = -4.3f;
    problem.upper_nm[wheel] = 4.3f;
    problem.rpm[wheel] = 5000.0f;
    problem.pid_reference_nm[wheel] = 3.0f;
  }
  const auto LIMITED = HelmPowerAllocation::solve(problem, workspace);
  check_solution(problem, LIMITED);
  require(LIMITED.upper_power_w > 95.0f, "exercise an active shared power constraint");
  float current[4]{};
  for (int wheel = 0; wheel < 4; ++wheel)
  {
    current[wheel] = problem.current_per_nm * LIMITED.torque_nm[wheel];
  }
  const auto GUARD = HelmPowerLimit::limit_power_proportionally(
      current, problem.rpm, 4, problem.available_power_w, problem.kt, problem.k1,
      problem.k2);
  require(!GUARD.is_power_limited, "final guard preserves the allocated priority result");
  require(GUARD.budget_feasible, "PID plus allocated torque fits drive-only budget");
}

void abort_and_workspace_reuse()
{
  auto problem = base_problem();
  problem.matrix[0][0] = 1.0f;
  problem.requested[0] = 1.0f;
  Workspace workspace{};
  const auto ABORTED = HelmPowerAllocation::solve(problem, workspace, abort_now, nullptr);
  require(ABORTED.time_limited, "abort callback marks deadline state");
  require(!ABORTED.converged, "aborted solve is not reported converged");
  for (int iteration = 0; iteration < 12; ++iteration)
  {
    problem.requested[0] = (iteration % 2 == 0) ? 1.0f : -1.0f;
    const auto RESULT = HelmPowerAllocation::solve(problem, workspace);
    check_solution(problem, RESULT);
    near(RESULT.achieved[0], problem.requested[0],
         "workspace reset after abort and changing sign");
  }
}
}  // namespace

int main()
{
  std::puts("pure_straight");
  pure_straight();
  std::puts("pure_rotation");
  pure_rotation();
  std::puts("weighted_priorities");
  weighted_priorities();
  std::puts("task_scaling_and_unreachable_yaw");
  task_scaling_and_unreachable_yaw();
  std::puts("large_request_scaling_regression");
  large_request_scaling_regression();
  std::puts("fixed_wheels_and_zero_span");
  fixed_wheels_and_zero_span();
  std::puts("singular_and_zero_budget");
  singular_and_zero_budget();
  std::puts("braking_is_not_a_power_credit");
  braking_is_not_a_power_credit();
  std::puts("speed_loss_can_make_budget_infeasible");
  speed_loss_can_make_budget_infeasible();
  std::puts("invalid_inputs");
  invalid_inputs();
  std::puts("abort_and_workspace_reuse");
  abort_and_workspace_reuse();
  std::puts("wheel_pid_and_final_power_guard");
  wheel_pid_and_final_power_guard();
  std::puts("helm_power_allocation_test: all tests passed");
  return 0;
}
