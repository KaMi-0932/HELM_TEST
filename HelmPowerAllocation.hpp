#pragma once

#include <algorithm>
#include <cmath>

#include "HelmDaqpStorage.hpp"

// Single-precision, fixed-memory weighted QP. All constraints act on FINAL
// wheel torque, including the wheel-speed PID correction. Model power is
// conservatively bounded by twelve chords; this is not a sensor measurement.
namespace HelmPowerAllocation
{
constexpr unsigned WHEEL_COUNT = 4;
constexpr unsigned TASK_COUNT = 3;
constexpr unsigned POWER_SEGMENTS = 12;
constexpr unsigned VARIABLE_COUNT = 8;
constexpr unsigned CONSTRAINT_COUNT = 57;
constexpr unsigned MAX_ITERATIONS = 128;
constexpr float MAX_CURRENT_LSB = 16384.0f;
constexpr float TASK_WEIGHT[TASK_COUNT] = {100.0f, 10.0f, 1.0f};
constexpr float PID_WEIGHT = 0.01f;
constexpr float PREVIOUS_WEIGHT = 0.0005f;
constexpr float POWER_WEIGHT = 0.001f;

// TEMP DIAG: stage markers that survive a hang, readable in the debugger.
// diag_stage: 0 = not entered yet, 1 = inside qp.Update(), 2 = update returned,
//             3 = inside qp.Solve(), 4 = solve returned, 5 = solve() finished,
//             100 = input rejected. diag_calls counts solve() entries;
// diag_n_prox / diag_exitflag record the DAQP dispatch of the last attempt.
// diag_seq marks the step of the Helm control loop that is currently running.
// diag_rx_count / diag_rx_x / diag_rx_y record what the chassis command
// subscriber actually delivered. Remove with the whole TEMP DIAG block later.
inline volatile unsigned diag_stage = 0;
inline volatile unsigned diag_seq = 0;
inline volatile unsigned diag_calls = 0;
inline volatile int diag_n_prox = -1;
inline volatile int diag_exitflag = 0;
inline volatile unsigned diag_rx_count = 0;
inline volatile float diag_rx_x = 0.0f;
inline volatile float diag_rx_y = 0.0f;

struct Problem
{
  // Caller orders these rows from highest to lowest tracking weight.
  float matrix[TASK_COUNT][WHEEL_COUNT]{};
  float requested[TASK_COUNT]{};
  float lower_nm[WHEEL_COUNT]{}, upper_nm[WHEEL_COUNT]{};
  float pid_reference_nm[WHEEL_COUNT]{}, previous_nm[WHEEL_COUNT]{};
  float rpm[WHEEL_COUNT]{};
  float current_per_nm = 0.0f;
  float kt = 0.0f, k1 = 0.0f, k2 = 0.0f;
  float available_power_w = 0.0f;
};

struct Result
{
  float torque_nm[WHEEL_COUNT]{};
  float achieved[TASK_COUNT]{}, error[TASK_COUNT]{};
  float estimated_power_w = 0.0f, upper_power_w = 0.0f;
  unsigned iterations = 0;
  int status = 0;
  bool valid = false;
  bool converged = false;
  bool budget_feasible = false;
  bool time_limited = false;
};

struct Workspace
{
  DaqpStorage qp{};
  float scale[TASK_COUNT]{};
  float quadratic_a[WHEEL_COUNT]{}, quadratic_b[WHEEL_COUNT]{};
  float quadratic_c[WHEEL_COUNT]{};
};

namespace Detail
{
inline float upper_power(float u, float a, float b, float c)
{
  float power = 0.0f;
  for (unsigned segment = 0; segment < POWER_SEGMENTS; ++segment)
  {
    const float LEFT = -1.0f + 2.0f * segment / POWER_SEGMENTS;
    const float RIGHT = -1.0f + 2.0f * (segment + 1) / POWER_SEGMENTS;
    power = std::fmax(power, (a * (LEFT + RIGHT) + b) * u + c - a * LEFT * RIGHT);
  }
  return power;
}

inline float upper_power_sum(const Problem& problem, const Workspace& work,
                             const float* torque)
{
  float sum = 0.0f;
  for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
  {
    sum += upper_power(torque[wheel] / (MAX_CURRENT_LSB / problem.current_per_nm),
                       work.quadratic_a[wheel], work.quadratic_b[wheel],
                       work.quadratic_c[wheel]);
  }
  return sum;
}

// Restore the numerical reserve after a small QP primal residual. Convex
// blending with the minimum-power vector preserves every wheel bound.
// Large physical-budget errors are rejected rather than repaired.
inline bool restore_power_reserve(const Problem& problem, const Workspace& work,
                                  float* torque)
{
  const float POWER = upper_power_sum(problem, work, torque);
  if (!std::isfinite(POWER) || POWER > problem.available_power_w + 1.0e-3f)
  {
    return false;
  }
  const float RESERVE = std::fmin(0.05f, 0.001f * problem.available_power_w);
  const float BUDGET = problem.available_power_w - RESERVE;
  if (POWER <= BUDGET)
  {
    return true;
  }
  const float TORQUE_SCALE = MAX_CURRENT_LSB / problem.current_per_nm;
  float minimum[WHEEL_COUNT]{};
  for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
  {
    minimum[wheel] = problem.lower_nm[wheel];
    float best = upper_power(minimum[wheel] / TORQUE_SCALE, work.quadratic_a[wheel],
                             work.quadratic_b[wheel], work.quadratic_c[wheel]);
    // A convex piecewise-linear envelope is minimized at a bound or a junction.
    // Clamping junctions also covers the upper wheel bound.
    for (unsigned junction = 0; junction <= POWER_SEGMENTS; ++junction)
    {
      const float CANDIDATE =
          std::clamp(TORQUE_SCALE * (-1.0f + 2.0f * junction / POWER_SEGMENTS),
                     problem.lower_nm[wheel], problem.upper_nm[wheel]);
      const float VALUE = upper_power(CANDIDATE / TORQUE_SCALE, work.quadratic_a[wheel],
                                      work.quadratic_b[wheel], work.quadratic_c[wheel]);
      if (VALUE < best)
      {
        best = VALUE;
        minimum[wheel] = CANDIDATE;
      }
    }
  }
  if (upper_power_sum(problem, work, minimum) > BUDGET + 1.0e-5f)
  {
    return false;
  }
  float low = 0.0f, high = 1.0f;
  for (unsigned iteration = 0; iteration < 24; ++iteration)
  {
    const float MID = 0.5f * (low + high);
    float candidate[WHEEL_COUNT]{};
    for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
      candidate[wheel] =
          std::clamp(minimum[wheel] + MID * (torque[wheel] - minimum[wheel]),
                     problem.lower_nm[wheel], problem.upper_nm[wheel]);
    }
    if (upper_power_sum(problem, work, candidate) <= BUDGET)
    {
      low = MID;
    }
    else
    {
      high = MID;
    }
  }
  for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
  {
    torque[wheel] = std::clamp(minimum[wheel] + low * (torque[wheel] - minimum[wheel]),
                               problem.lower_nm[wheel], problem.upper_nm[wheel]);
  }
  return upper_power_sum(problem, work, torque) <= BUDGET + 2.0e-4f;
}

inline bool initialize(const Problem& problem, Workspace& work)
{
  if (!std::isfinite(problem.current_per_nm) || problem.current_per_nm <= 0.0f ||
      !std::isfinite(problem.available_power_w) || problem.available_power_w < 0.0f ||
      !std::isfinite(problem.kt) || problem.kt < 0.0f || !std::isfinite(problem.k1) ||
      problem.k1 < 0.0f || !std::isfinite(problem.k2) || problem.k2 < 0.0f)
  {
    return false;
  }
  const float TORQUE_SCALE = MAX_CURRENT_LSB / problem.current_per_nm;
  const float POWER_SCALE = std::fmax(1.0f, problem.available_power_w);
  if (!std::isfinite(TORQUE_SCALE))
  {
    return false;
  }
  auto& qp = work.qp;
  std::fill_n(qp.hessian, VARIABLE_COUNT * VARIABLE_COUNT, 0.0f);
  std::fill_n(qp.linear, VARIABLE_COUNT, 0.0f);
  std::fill_n(qp.matrix, (CONSTRAINT_COUNT - VARIABLE_COUNT) * VARIABLE_COUNT, 0.0f);
  std::fill_n(qp.lower, CONSTRAINT_COUNT, -DAQP_INF);
  std::fill_n(qp.upper, CONSTRAINT_COUNT, DAQP_INF);
  for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
  {
    if (!std::isfinite(problem.lower_nm[wheel]) ||
        !std::isfinite(problem.upper_nm[wheel]) ||
        problem.lower_nm[wheel] > problem.upper_nm[wheel] ||
        problem.lower_nm[wheel] < -TORQUE_SCALE - 1.0e-5f ||
        problem.upper_nm[wheel] > TORQUE_SCALE + 1.0e-5f ||
        !std::isfinite(problem.pid_reference_nm[wheel]) ||
        !std::isfinite(problem.previous_nm[wheel]) || !std::isfinite(problem.rpm[wheel]))
    {
      return false;
    }
    qp.lower[wheel] = std::fmax(-1.0f, problem.lower_nm[wheel] / TORQUE_SCALE);
    qp.upper[wheel] = std::fmin(1.0f, problem.upper_nm[wheel] / TORQUE_SCALE);
    qp.lower[4 + wheel] = 0.0f;
    qp.upper[4 + wheel] = problem.available_power_w / POWER_SCALE;
    qp.hessian[wheel * VARIABLE_COUNT + wheel] = PID_WEIGHT + PREVIOUS_WEIGHT;
    qp.hessian[(4 + wheel) * VARIABLE_COUNT + 4 + wheel] = POWER_WEIGHT;
    qp.linear[wheel] = -(PID_WEIGHT * problem.pid_reference_nm[wheel] +
                         PREVIOUS_WEIGHT * problem.previous_nm[wheel]) /
                       TORQUE_SCALE;
    const float A = problem.k1 * MAX_CURRENT_LSB * MAX_CURRENT_LSB;
    const float B = problem.kt * MAX_CURRENT_LSB * problem.rpm[wheel];
    const float C = problem.k2 * problem.rpm[wheel] * problem.rpm[wheel];
    if (!std::isfinite(A) || !std::isfinite(B) || !std::isfinite(C))
    {
      return false;
    }
    work.quadratic_a[wheel] = A;
    work.quadratic_b[wheel] = B;
    work.quadratic_c[wheel] = C;
    for (unsigned segment = 0; segment < POWER_SEGMENTS; ++segment)
    {
      const float LEFT = -1.0f + 2.0f * segment / POWER_SEGMENTS;
      const float RIGHT = -1.0f + 2.0f * (segment + 1) / POWER_SEGMENTS;
      const unsigned ROW = POWER_SEGMENTS * wheel + segment;
      qp.matrix[ROW * VARIABLE_COUNT + wheel] = (A * (LEFT + RIGHT) + B) / POWER_SCALE;
      qp.matrix[ROW * VARIABLE_COUNT + 4 + wheel] = -1.0f;
      qp.upper[VARIABLE_COUNT + ROW] = -(C - A * LEFT * RIGHT) / POWER_SCALE;
    }
    qp.matrix[48 * VARIABLE_COUNT + 4 + wheel] = 1.0f;
  }
  // Small numerical reserve, in addition to the existing shared chassis margin.
  const float RESERVE = std::fmin(0.05f, 0.001f * problem.available_power_w);
  qp.upper[56] = (problem.available_power_w - RESERVE) / POWER_SCALE;
  for (unsigned task = 0; task < TASK_COUNT; ++task)
  {
    if (!std::isfinite(problem.requested[task]))
    {
      return false;
    }
    float span = 0.0f;
    for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
      if (!std::isfinite(problem.matrix[task][wheel]))
      {
        return false;
      }
      span += std::fabs(problem.matrix[task][wheel]) * TORQUE_SCALE;
    }
    const float SCALE =
        std::fmax(1.0f, std::fmax(span, std::fabs(problem.requested[task])));
    if (!std::isfinite(SCALE))
    {
      return false;
    }
    work.scale[task] = SCALE;
    for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
      const float VALUE = problem.matrix[task][wheel] * TORQUE_SCALE / SCALE;
      qp.linear[wheel] -= TASK_WEIGHT[task] * VALUE * problem.requested[task] / SCALE;
      for (unsigned other = 0; other < WHEEL_COUNT; ++other)
      {
        qp.hessian[wheel * VARIABLE_COUNT + other] += TASK_WEIGHT[task] * VALUE *
                                                      problem.matrix[task][other] *
                                                      TORQUE_SCALE / SCALE;
      }
    }
  }
  qp.settings.primal_tol = 1.0e-7f;
  qp.settings.dual_tol = 1.0e-6f;
  qp.settings.zero_tol = 1.0e-8f;
  qp.settings.pivot_tol = 1.0e-7f;
  qp.settings.iter_limit = MAX_ITERATIONS + 1;
  return qp.Update() >= 0;
}

inline bool extract(const Problem& problem, const Workspace& work, Result& result)
{
  const float TORQUE_SCALE = MAX_CURRENT_LSB / problem.current_per_nm;
  for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
  {
    const float RAW = work.qp.solution[wheel] * TORQUE_SCALE;
    if (!std::isfinite(RAW) || RAW < problem.lower_nm[wheel] - 1.0e-4f * TORQUE_SCALE ||
        RAW > problem.upper_nm[wheel] + 1.0e-4f * TORQUE_SCALE)
    {
      return false;
    }
    const float TORQUE =
        std::clamp(RAW, problem.lower_nm[wheel], problem.upper_nm[wheel]);
    result.torque_nm[wheel] = TORQUE;
  }
  if (!restore_power_reserve(problem, work, result.torque_nm))
  {
    return false;
  }
  for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
  {
    const float TORQUE = result.torque_nm[wheel];
    const float U = TORQUE / TORQUE_SCALE;
    const float A = work.quadratic_a[wheel], B = work.quadratic_b[wheel];
    const float C = work.quadratic_c[wheel];
    result.estimated_power_w += std::fmax(0.0f, (A * U + B) * U + C);
    result.upper_power_w += upper_power(U, A, B, C);
  }
  for (unsigned task = 0; task < TASK_COUNT; ++task)
  {
    for (unsigned wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
      result.achieved[task] += problem.matrix[task][wheel] * result.torque_nm[wheel];
    }
    result.error[task] = problem.requested[task] - result.achieved[task];
    if (!std::isfinite(result.achieved[task]))
    {
      return false;
    }
  }
  return std::isfinite(result.upper_power_w) &&
         result.upper_power_w <= problem.available_power_w + 1.0e-3f &&
         result.estimated_power_w <= problem.available_power_w + 1.0e-3f;
}
}  // namespace Detail

inline Result solve(const Problem& problem, Workspace& work,
                    bool (*abort)(void*) = nullptr, void* context = nullptr)
{
  Result result{};
  ++diag_calls;

  // TEMP DIAG: force AllocateWheelForces() onto AllocateWheelForcesFallback() so the
  // DAQP path (update + solve) is never entered. Set to false to restore.
  constexpr bool FORCE_FALLBACK_DIAG = false;
  if (FORCE_FALLBACK_DIAG)
  {
    result.status = -200;  // Diagnostic bypass.
    return result;
  }

  if (abort != nullptr && abort(context))
  {
    result.time_limited = true;
    return result;
  }
  diag_stage = 1;
  if (!Detail::initialize(problem, work))
  {
    diag_stage = 100;  // Invalid input or matrix update rejected.
    result.status = -100;
    return result;
  }
  diag_stage = 2;
  if (abort != nullptr && abort(context))
  {
    result.time_limited = true;
    return result;
  }
  // Run once: DAQP's local cycle/repair state must survive the whole solve.
  // The iteration cap bounds work; the wall-clock budget is observed before and
  // after this call and is not an interrupt inside an active-set iteration.
  diag_n_prox = work.qp.work.n_prox;
  diag_stage = 3;
  work.qp.Solve();
  diag_stage = 4;
  result.status = work.qp.result.exitflag;
  diag_exitflag = result.status;
  result.iterations = static_cast<unsigned>(
      std::max(0, work.qp.result.iter - (result.status == DAQP_EXIT_ITERLIMIT ? 1 : 0)));
  if (result.status == DAQP_EXIT_OPTIMAL)
  {
    if (Detail::extract(problem, work, result))
    {
      result.valid = result.converged = result.budget_feasible = true;
    }
    else
    {
      result.status = -101;  // Independent physical output verification failed.
    }
  }
  result.time_limited = abort != nullptr && abort(context);
  diag_stage = 5;
  return result;
}
}  // namespace HelmPowerAllocation
