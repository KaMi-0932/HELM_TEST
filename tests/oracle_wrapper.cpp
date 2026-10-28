#include "../HelmPowerAllocation.hpp"

#ifdef _WIN32
#define HELM_ORACLE_EXPORT __declspec(dllexport)
#else
#define HELM_ORACLE_EXPORT __attribute__((visibility("default")))
#endif

static void solve_with_workspace(const float* input, float* output, int* status,
                                 HelmPowerAllocation::Workspace& workspace)
{
  HelmPowerAllocation::Problem problem{};
  unsigned cursor = 0;
  for (auto& row : problem.matrix)
    for (auto& value : row) value = input[cursor++];
  for (auto& value : problem.requested) value = input[cursor++];
  for (auto& value : problem.lower_nm) value = input[cursor++];
  for (auto& value : problem.upper_nm) value = input[cursor++];
  for (auto& value : problem.pid_reference_nm) value = input[cursor++];
  for (auto& value : problem.previous_nm) value = input[cursor++];
  for (auto& value : problem.rpm) value = input[cursor++];
  problem.current_per_nm = input[cursor++];
  problem.kt = input[cursor++];
  problem.k1 = input[cursor++];
  problem.k2 = input[cursor++];
  problem.available_power_w = input[cursor++];
  const auto result = HelmPowerAllocation::solve(problem, workspace);
  cursor = 0;
  for (float value : result.torque_nm) output[cursor++] = value;
  for (float value : result.achieved) output[cursor++] = value;
  for (float value : result.error) output[cursor++] = value;
  output[cursor++] = result.estimated_power_w;
  output[cursor++] = result.upper_power_w;
  // Preserve the library's pre-clamp/pre-reserve-repair torque for checking the
  // bounded postprocessing independently. This is test-only output.
  const float torque_scale =
      HelmPowerAllocation::MAX_CURRENT_LSB / problem.current_per_nm;
  for (unsigned wheel = 0; wheel < HelmPowerAllocation::WHEEL_COUNT; ++wheel)
    output[cursor++] = workspace.qp.solution[wheel] * torque_scale;
  status[0] = static_cast<int>(result.iterations);
  status[1] = result.valid;
  status[2] = result.converged;
  status[3] = result.budget_feasible;
  status[4] = result.time_limited;
  status[5] = result.status;
}

extern "C" HELM_ORACLE_EXPORT void solve_flat(const float* input, float* output,
                                              int* status)
{
  HelmPowerAllocation::Workspace workspace{};
  solve_with_workspace(input, output, status, workspace);
}

extern "C" HELM_ORACLE_EXPORT void solve_flat_reused(const float* input, float* output,
                                                     int* status)
{
  // Test-only persistent instance exercises every changing matrix/bound after
  // the preceding random solve. Firmware owns its workspace as a Helm member.
  static HelmPowerAllocation::Workspace workspace{};
  solve_with_workspace(input, output, status, workspace);
}
