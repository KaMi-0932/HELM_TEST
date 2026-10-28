#pragma once

#ifndef DAQP_SINGLE_PRECISION
#define DAQP_SINGLE_PRECISION
#endif

extern "C"
{
#include "third_party/daqp/include/api.h"
#include "third_party/daqp/include/utils.h"
}

namespace HelmPowerAllocation
{
/* Every pointer used by the ordinary positive-definite QP path is bound to this
 * instance. Setup/free, equality reduction, hierarchy, AVI and BnB are unused. */
struct DaqpStorage
{
  static constexpr unsigned VARIABLES = 8;
  static constexpr unsigned SIMPLE_BOUNDS = 8;
  static constexpr unsigned CONSTRAINTS = 57;
  static constexpr unsigned GENERAL_CONSTRAINTS = CONSTRAINTS - SIMPLE_BOUNDS;
  static constexpr unsigned WORKING_SET_CAPACITY = VARIABLES + 1;

  c_float hessian[VARIABLES * VARIABLES]{};
  c_float linear[VARIABLES]{};
  c_float matrix[GENERAL_CONSTRAINTS * VARIABLES]{};
  c_float lower[CONSTRAINTS]{};
  c_float upper[CONSTRAINTS]{};
  c_float solution[VARIABLES]{};

  DAQPProblem problem{};
  DAQPWorkspace work{};
  DAQPSettings settings{};
  DAQPResult result{};

  DaqpStorage()
  {
    static_assert(sizeof(c_float) == sizeof(float), "DAQP must use float throughout");
    daqp_default_settings(&settings);
    settings.eq_reduction = DAQP_EQ_REDUCTION_OFF;
    problem.n = VARIABLES;
    problem.m = CONSTRAINTS;
    problem.ms = SIMPLE_BOUNDS;
    problem.H = hessian;
    problem.f = linear;
    problem.A = matrix;
    problem.blower = lower;
    problem.bupper = upper;
    /* Null sense means ordinary hard constraints. DAQP detects fixed bounds. */
    problem.sense = nullptr;

    work.qp = &problem;
    work.n = VARIABLES;
    work.m = CONSTRAINTS;
    work.ms = SIMPLE_BOUNDS;
    work.settings = &settings;
    work.M = transformed_matrix_;
    work.dupper = transformed_upper_;
    work.dlower = transformed_lower_;
    work.Rinv = inverse_cholesky_;
    work.v = transformed_linear_;
    work.sense = constraint_sense_;
    work.scaling = scaling_;
    work.x = iterate_;
    work.u = iterate_;
    work.xold = old_solution_;
    work.lam = multipliers_;
    work.lam_star = stationary_multipliers_;
    work.L = factor_l_;
    work.D = factor_d_;
    work.xldl = factor_x_;
    work.zldl = factor_z_;
    work.WS = working_set_;
    work.prox_mask = proximal_mask_;
    work.Mu = matrix_iterate_;
    work.outer_iter = 1;
    for (unsigned i = 0; i < SIMPLE_BOUNDS; ++i)
    {
      scaling_[i] = 1.0f;
    }
    reset_daqp_workspace(&work);
    result.x = solution;
    result.lam = result_multipliers_;
  }

  DaqpStorage(const DaqpStorage&) = delete;
  DaqpStorage& operator=(const DaqpStorage&) = delete;

  /* A full matrix update invalidates the old working set. This method performs
   * no allocation and never requests optional reduced or hierarchical storage. */
  int Update()
  {
    settings.eq_reduction = DAQP_EQ_REDUCTION_OFF;
    reset_daqp_workspace(&work);
    result.exitflag = 0;
    result.iter = 0;
    return daqp_update_ldp(DAQP_UPDATE_Rinv | DAQP_UPDATE_M | DAQP_UPDATE_v |
                               DAQP_UPDATE_d | DAQP_UPDATE_sense,
                           &work, &problem);
  }

  /* Run one bounded active-set solve. Repeated partial solves reset DAQP's
   * cycle-repair counters, so callers use one iteration cap per control cycle. */
  void Solve() { daqp_solve(&result, &work); }

 private:
  c_float transformed_matrix_[GENERAL_CONSTRAINTS * VARIABLES]{};
  c_float transformed_upper_[CONSTRAINTS]{};
  c_float transformed_lower_[CONSTRAINTS]{};
  c_float inverse_cholesky_[VARIABLES * (VARIABLES + 1) / 2]{};
  c_float transformed_linear_[VARIABLES]{};
  c_float scaling_[CONSTRAINTS]{};
  c_float matrix_iterate_[GENERAL_CONSTRAINTS]{};
  int constraint_sense_[CONSTRAINTS]{};
  c_float iterate_[VARIABLES]{};
  c_float old_solution_[VARIABLES]{};
  c_float multipliers_[WORKING_SET_CAPACITY]{};
  c_float stationary_multipliers_[WORKING_SET_CAPACITY]{};
  c_float factor_l_[WORKING_SET_CAPACITY * (WORKING_SET_CAPACITY + 1) / 2]{};
  c_float factor_d_[WORKING_SET_CAPACITY]{};
  c_float factor_x_[WORKING_SET_CAPACITY]{};
  c_float factor_z_[WORKING_SET_CAPACITY]{};
  int working_set_[WORKING_SET_CAPACITY]{};
  int proximal_mask_[VARIABLES]{};
  c_float result_multipliers_[CONSTRAINTS]{};
};
}  // namespace HelmPowerAllocation
