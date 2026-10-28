"""Compare the production float allocator with independent double precision QPs.

Requires NumPy and SciPy. OSQP is optional and is used only if SLSQP reports
uncertainty. C/C++ compilers must be on PATH or supplied with --cc / --cxx.
The script rebuilds vendored C sources, writes all artifacts under --build-dir,
and never modifies firmware sources. Host timing does not establish MCU timing.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import ctypes
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
CHASSIS = HERE.parent
ROOT = CHASSIS.parents[2]
CURRENT_LIMIT = 16384.0
SEGMENTS = 12


def build_wrapper(build_dir, cc, cxx, reused=False):
    source = CHASSIS / "third_party" / "daqp"
    include = source / "include"
    suffix = ".dll" if os.name == "nt" else ".so"
    library_path = build_dir / ("oracle_wrapper" + suffix)
    flags = ["-O2", "-std=c99", "-DDAQP_SINGLE_PRECISION", "-DDAQP_NO_SOFT_WEIGHTS",
             "-I" + str(include)]
    if os.name != "nt":
        flags.append("-fPIC")

    def compile_source(path):
        output = build_dir / (path.stem + "_oracle.o")
        subprocess.run([cc, *flags, "-c", str(path), "-o", str(output)], check=True)
        return str(output)

    with ThreadPoolExecutor(max_workers=4) as pool:
        objects = list(pool.map(compile_source, sorted((source / "src").glob("*.c"))))
    cxx_flags = ["-O2", "-std=c++20", "-shared", "-Wall", "-Wextra", "-Werror",
                 "-DDAQP_SINGLE_PRECISION", "-DDAQP_NO_SOFT_WEIGHTS"]
    if os.name != "nt":
        cxx_flags.append("-fPIC")
    allocator_source = CHASSIS / "HelmPowerAllocation.cpp"
    cpp_sources = [str(HERE / "oracle_wrapper.cpp")]
    if allocator_source.is_file():
        cpp_sources.append(str(allocator_source))
    subprocess.run([cxx, *cxx_flags, *cpp_sources, *objects, "-o", str(library_path)],
                   check=True)
    # Retain the directory handle while loading MinGW's runtime dependencies.
    dll_directory = None
    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        compiler = shutil.which(cxx) or cxx
        dll_directory = os.add_dll_directory(str(Path(compiler).resolve().parent))
    library = ctypes.CDLL(str(library_path))
    function = library.solve_flat_reused if reused else library.solve_flat
    function.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                         ctypes.POINTER(ctypes.c_int)]
    function.restype = None
    return function, library, dll_directory


def call_production(function, problem):
    flat = np.concatenate([
        problem["matrix"].ravel(), problem["requested"], problem["lower"], problem["upper"],
        problem["pid"], problem["previous"], problem["rpm"],
        [problem["conversion"], problem["kt"], problem["k1"], problem["k2"], problem["budget"]],
    ]).astype(np.float32)
    output = np.zeros(16, dtype=np.float32)
    status = np.zeros(6, dtype=np.int32)
    function(flat.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
             output.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
             status.ctypes.data_as(ctypes.POINTER(ctypes.c_int)))
    return output.astype(float), status


def random_problem(rng, index):
    angles = rng.uniform(-np.pi, np.pi, 4)
    length, width = rng.uniform(.2, .6, 2)
    radius = np.hypot(length, width) / 2
    x = np.array([-1, 1, 1, -1]) * width / 2
    y = np.array([-1, -1, 1, 1]) * length / 2
    heading = rng.uniform(-np.pi, np.pi)
    sine, cosine = np.sin(angles), np.cos(angles)
    matrix = np.array([(x * cosine - y * sine) / radius,
                       np.cos(heading) * sine - np.sin(heading) * cosine,
                       np.sin(heading) * sine + np.cos(heading) * cosine])
    if index % 11 == 0:
        matrix[1] = matrix[0]
    if index % 17 == 0:
        matrix[2] = 0
    lower, upper = -rng.uniform(.1, 4.3, 4), rng.uniform(.1, 4.3, 4)
    if index % 7 == 0:
        lower[index % 4] = upper[index % 4] = 0
    if index % 29 == 0:
        upper[:] = lower[:] = 0
    requested = rng.uniform(-12, 12, 3)
    # Exercise requests above the complete wheel authority as well as routine
    # commands. Alternating signs and axes also stress a reused factorization.
    if index % 5 == 1:
        requested = rng.uniform(-80, 80, 3)
    elif index % 5 == 2:
        requested[index % 3] = (-1 if index % 2 else 1) * rng.uniform(80, 250)
    problem = dict(matrix=matrix, requested=requested, lower=lower, upper=upper,
                   pid=rng.uniform(-1, 1, 4), previous=rng.uniform(-1, 1, 4),
                   rpm=rng.uniform(-9500, 9500, 4), conversion=52437.5 / 14,
                   kt=1.99688994e-6, k1=2e-7, k2=3e-7, budget=rng.uniform(2, 140))
    # Compare exactly the values that are passed through the float C interface.
    for key, value in problem.items():
        problem[key] = np.asarray(value, dtype=np.float32).astype(float)
        if problem[key].ndim == 0:
            problem[key] = float(problem[key])
    return problem


def model(problem):
    """Build a separate double QP from the documented cost and physical model."""
    torque_scale = CURRENT_LIMIT / problem["conversion"]
    power_scale = max(1.0, problem["budget"])
    scale = np.maximum(1.0, np.abs(problem["matrix"]).sum(axis=1) * torque_scale)
    matrix = problem["matrix"] * torque_scale / scale[:, None]
    target = problem["requested"] / scale
    weights = np.diag([100.0, 10.0, 1.0])
    hessian = np.diag([.0105] * 4 + [.001] * 4)
    hessian[:4, :4] += matrix.T @ weights @ matrix
    gradient = np.zeros(8)
    gradient[:4] = (-matrix.T @ weights @ target
                    - (.01 * problem["pid"] + .0005 * problem["previous"]) / torque_scale)
    rows, upper = [], []
    for wheel in range(4):
        rpm = problem["rpm"][wheel]
        # Construct chords from endpoint samples in current units; this does
        # not use production's normalized-current slope/intercept expressions.
        current = np.linspace(-CURRENT_LIMIT, CURRENT_LIMIT, SEGMENTS + 1)
        power = (problem["kt"] * current * rpm + problem["k1"] * current**2
                 + problem["k2"] * rpm**2)
        for segment in range(SEGMENTS):
            slope = (power[segment + 1] - power[segment]) / (current[segment + 1] - current[segment])
            intercept = power[segment] - slope * current[segment]
            row = np.zeros(8)
            row[wheel] = slope * CURRENT_LIMIT / power_scale
            row[4 + wheel] = -1
            rows.append(row)
            upper.append(-intercept / power_scale)
    row = np.zeros(8)
    row[4:] = 1
    rows.append(row)
    upper.append((problem["budget"] - min(.05, .001 * problem["budget"])) / power_scale)
    lower_bound = np.r_[problem["lower"] / torque_scale, np.zeros(4)]
    upper_bound = np.r_[problem["upper"] / torque_scale,
                        np.ones(4) * problem["budget"] / power_scale]
    return torque_scale, hessian, gradient, np.array(rows), np.array(upper), lower_bound, upper_bound


def oracle(hessian, gradient, matrix, upper, lower_bound, upper_bound):
    feasibility = linprog(np.zeros(8), A_ub=matrix, b_ub=upper,
                          bounds=list(zip(lower_bound, upper_bound)), method="highs",
                          options={"primal_feasibility_tolerance": 1e-9,
                                   "dual_feasibility_tolerance": 1e-9})
    if feasibility.status == 2:
        return None, "infeasible"
    if not feasibility.success:
        return None, "uncertain LP: " + feasibility.message
    objective = lambda x: .5 * x @ hessian @ x + gradient @ x
    jacobian = lambda x: hessian @ x + gradient
    result = minimize(objective, feasibility.x, jac=jacobian, method="SLSQP",
                      bounds=Bounds(lower_bound, upper_bound),
                      constraints=[LinearConstraint(matrix, -np.inf, upper)],
                      options={"ftol": 1e-11, "maxiter": 3000})
    feasible = (np.max(matrix @ result.x - upper) <= 1e-7
                and np.max(lower_bound - result.x) <= 1e-7
                and np.max(result.x - upper_bound) <= 1e-7)
    if result.success and feasible:
        return (result.x, objective(result.x)), "SLSQP"
    # A separate double solver resolves only uncertain SLSQP exits. The LP
    # feasibility result remains independent of both QP implementations.
    try:
        import osqp
        import scipy.sparse as sparse
    except ImportError:
        return None, "uncertain SLSQP: " + result.message
    combined = sparse.csc_matrix(np.vstack([np.eye(8), matrix]))
    solver = osqp.OSQP()
    solver.setup(P=sparse.csc_matrix(hessian), q=gradient, A=combined,
                 l=np.r_[lower_bound, np.full(len(upper), -np.inf)],
                 u=np.r_[upper_bound, upper], verbose=False, eps_abs=1e-9,
                 eps_rel=1e-9, max_iter=100000, polishing=True)
    secondary = solver.solve()
    if secondary.info.status_val == 1:
        return (secondary.x, objective(secondary.x)), "OSQP after SLSQP uncertainty"
    return None, "uncertain SLSQP/OSQP: " + result.message + "/" + secondary.info.status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20261027)
    parser.add_argument("--build-dir", type=Path,
                        default=ROOT / "build" / "debug" / "helm-priority-power" / "final-oracle")
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--python-tools", type=Path,
                        help="Optional local directory containing NumPy/SciPy/OSQP")
    parser.add_argument("--reuse-workspace", action="store_true",
                        help="Reuse one production workspace across all changing random problems")
    args = parser.parse_args()
    if args.python_tools:
        sys.path.insert(0, str(args.python_tools.resolve()))
    global np, linprog, minimize, Bounds, LinearConstraint
    import numpy as np
    from scipy.optimize import linprog, minimize, Bounds, LinearConstraint
    build_dir = args.build_dir.resolve()
    build_dir.mkdir(parents=True, exist_ok=True)
    function, library, dll_directory = build_wrapper(build_dir, args.cc, args.cxx, args.reuse_workspace)
    rng = np.random.default_rng(args.seed)
    report = dict(cases=args.cases, seed=args.seed,
                  workspace="reused" if args.reuse_workspace else "cold",
                  accepted=0, infeasible=0,
                  infeasible_rejected=0, feasible=0, feasible_fallbacks=0,
                  oracle_uncertain=0, maximum_objective_gap=0.0,
                  minimum_objective_gap=0.0, max_torque_bound_violation_nm=0.0,
                  max_scaled_positive_cost_gap=0.0,
                  repaired_cases=0, max_repair_torque_change_nm=0.0,
                  max_repair_normalized_task_change=0.0,
                  max_physical_power_over_budget_w=0.0,
                  max_upper_power_over_budget_w=0.0,
                  max_reserved_budget_violation_w=0.0, maximum_iterations=0,
                  above_authority_request_cases=0, maximum_absolute_request_nm=0.0,
                  failures=[], feasible_fallback_cases=[], uncertain_cases=[])
    methods, exits, times = Counter(), Counter(), []
    for index in range(args.cases):
        problem = random_problem(rng, index)
        torque_scale, hessian, gradient, matrix, upper, low, high = model(problem)
        authority_scale = np.maximum(1.0, np.abs(problem["matrix"]).sum(axis=1) * torque_scale)
        report["above_authority_request_cases"] += bool(
            np.any(np.abs(problem["requested"]) > authority_scale))
        report["maximum_absolute_request_nm"] = max(
            report["maximum_absolute_request_nm"], float(np.max(np.abs(problem["requested"]))))
        expected, method = oracle(hessian, gradient, matrix, upper, low, high)
        methods[method.split(":")[0]] += 1
        start = time.perf_counter()
        actual, status = call_production(function, problem)
        times.append(time.perf_counter() - start)
        exits[int(status[5])] += 1
        report["maximum_iterations"] = max(report["maximum_iterations"], int(status[0]))
        accepted = bool(status[1] and status[2] and status[3] and not status[4])
        if expected is None:
            if method == "infeasible":
                report["infeasible"] += 1
                if accepted:
                    report["failures"].append([index, "accepted independently infeasible QP", status.tolist()])
                else:
                    report["infeasible_rejected"] += 1
            else:
                report["oracle_uncertain"] += 1
                report["uncertain_cases"].append([index, method, status.tolist()])
        else:
            report["feasible"] += 1
            if not accepted:
                report["feasible_fallbacks"] += 1
                report["feasible_fallback_cases"].append([index, status.tolist()])
        if not accepted:
            continue
        report["accepted"] += 1
        torque = actual[:4]
        raw_torque = actual[12:16]
        repair_delta = float(np.max(np.abs(torque - raw_torque)))
        task_scale = np.maximum(1.0, np.abs(problem["matrix"]).sum(axis=1) * torque_scale)
        repair_task_delta = float(np.max(np.abs(problem["matrix"] @ (torque - raw_torque)) / task_scale))
        report["repaired_cases"] += repair_delta > 1e-5
        report["max_repair_torque_change_nm"] = max(report["max_repair_torque_change_nm"], repair_delta)
        report["max_repair_normalized_task_change"] = max(report["max_repair_normalized_task_change"], repair_task_delta)
        if repair_delta > .003 or repair_task_delta > .0005:
            report["failures"].append([index, "postprocessing change exceeds quality bound", {
                "torque_change_nm": repair_delta, "normalized_task_change": repair_task_delta}])
        current = problem["conversion"] * torque
        physical = np.maximum(0, problem["kt"] * current * problem["rpm"]
                              + problem["k1"] * current**2
                              + problem["k2"] * problem["rpm"]**2).sum()
        x = np.r_[torque / torque_scale, np.zeros(4)]
        for wheel in range(4):
            start_row = wheel * SEGMENTS
            x[4 + wheel] = max(0.0, np.max(matrix[start_row:start_row + SEGMENTS, wheel]
                                             * x[wheel] - upper[start_row:start_row + SEGMENTS]))
        upper_power = x[4:].sum() * max(1, problem["budget"])
        bound_violation = max(0.0, float(np.max(problem["lower"] - torque)),
                              float(np.max(torque - problem["upper"])))
        report["max_torque_bound_violation_nm"] = max(report["max_torque_bound_violation_nm"], bound_violation)
        report["max_physical_power_over_budget_w"] = max(report["max_physical_power_over_budget_w"], physical - problem["budget"])
        report["max_upper_power_over_budget_w"] = max(report["max_upper_power_over_budget_w"], upper_power - problem["budget"])
        reserve = min(.05, .001 * problem["budget"])
        reserve_violation = upper_power - (problem["budget"] - reserve)
        report["max_reserved_budget_violation_w"] = max(report["max_reserved_budget_violation_w"], reserve_violation)
        if (not np.all(np.isfinite(actual)) or bound_violation > 3e-4
                or physical > problem["budget"] + .002
                or upper_power > problem["budget"] + .002
                or abs(physical - actual[10]) > .003
                or abs(upper_power - actual[11]) > .003):
            report["failures"].append([index, "physical output violation", {
                "status": status.tolist(), "bounds_nm": bound_violation,
                "physical_w": float(physical), "upper_w": float(upper_power),
                "budget_w": problem["budget"], "reported_w": actual[10:12].tolist()}])
        if reserve_violation > .002:
            report["failures"].append([index, "reserved QP budget violation", float(reserve_violation), status.tolist()])
        if expected is not None:
            gap = float(.5 * x @ hessian @ x + gradient @ x - expected[1])
            report["maximum_objective_gap"] = max(report["maximum_objective_gap"], gap)
            report["minimum_objective_gap"] = min(report["minimum_objective_gap"], gap)
            # Restore the omitted constant to express the cost as a sum of
            # nonnegative tracking, PID, previous-command and power terms.
            target = problem["requested"] / task_scale
            constant = (.5 * np.sum(np.array([100., 10., 1.]) * target**2)
                        + .005 * np.sum((problem["pid"] / torque_scale)**2)
                        + .00025 * np.sum((problem["previous"] / torque_scale)**2))
            positive_optimal_cost = expected[1] + constant
            scaled_gap = gap / (1.0 + max(0.0, positive_optimal_cost))
            report["max_scaled_positive_cost_gap"] = max(report["max_scaled_positive_cost_gap"], scaled_gap)
            if scaled_gap > .001 or gap < -1e-4:
                report["failures"].append([index, "post-repair objective quality mismatch", {
                    "gap": gap, "scaled_gap": scaled_gap, "oracle": method,
                    "status": status.tolist()}])
    report["oracle_methods"] = dict(methods)
    report["production_exit_counts"] = dict(exits)
    report["host_mean_us"] = float(np.mean(times) * 1e6)
    report["host_max_us"] = max(times) * 1e6
    report["quality_gates"] = dict(max_gap_over_one_plus_positive_optimal_cost=.001,
                                    max_postprocessing_torque_change_nm=.003,
                                    max_postprocessing_normalized_task_change=.0005)
    report["note"] = "Accepted outputs meet an engineering approximation quality bound after bounded reserve repair; they are not claimed to be exact QP optima. Feasible fallbacks are reported separately. Host timing is not MCU timing."
    name = "oracle_report_reused.json" if args.reuse_workspace else "oracle_report.json"
    (build_dir / name).write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    if report["failures"] or report["oracle_uncertain"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
