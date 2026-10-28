# Allocator host checks

`helm_power_allocation_test.cpp` covers deterministic control and failure cases.
`verify_power_allocation.py` separately builds the production single precision
DAQP wrapper and compares it with a double precision reference problem.

The Python check requires NumPy, SciPy, GCC and G++. OSQP is optional and resolves
uncertain SLSQP exits. For example, from the firmware project directory:

```sh
python Modules/QDU-Robomaster/Chassis/tests/verify_power_allocation.py --cases 1000
python Modules/QDU-Robomaster/Chassis/tests/verify_power_allocation.py --cases 1000 --reuse-workspace
```

Use `--cc`, `--cxx`, `--python-tools` and `--build-dir` for locally installed tools.
The script rebuilds every vendored C source with the production precision flags
and compiles the allocator implementation at `-O2`; it links
`HelmPowerAllocation.cpp` when the implementation is in that source file. The
report and compiled test library go into the build directory.

The seeded cases include rectangular wheel positions, changing actual steering
angles, signed speeds, fixed wheel torques, dependent task rows, zero task rows,
requests up to magnitude 250 and power budgets that can be infeasible. The
independent cost uses the wheel-authority scale `max(1, sum(abs(row)) * torque_scale)`;
the magnitude of the requested wrench does not reduce its tracking weight.
HiGHS checks linear constraint
feasibility. SLSQP minimizes the documented weighted quadratic objective using
analytic derivatives and independently constructed power chords. Reports list
feasible cases that the production solver rejects, including status codes;
those cases exercise the firmware fallback rather than prove solver success.

Every accepted output is checked against torque bounds, the positive quadratic
power model, the conservative chord envelope and the numerical reserve. Its
objective is compared with the independent optimum. Both fresh and reused
workspaces should be exercised. Host execution times do not establish Cortex-M4
execution time or replace measurements on the running firmware.

The deterministic C++ regression holds the other tasks fixed and increases the
longitudinal request through `10, 17.5, 25, 40, 80 N m` in the straight-wheel,
6000 rpm, 96 W fixture that exposed request-dependent scaling. It covers both
translation signs, both fixed yaw signs, and fresh versus reused workspaces.
The signed achieved longitudinal output must not decrease beyond a small
numerical tolerance in this fixture. This does not assert monotonicity for
every axis of an arbitrary coupled constrained problem.

The production wrapper can make a small torque correction to restore the
numerical power reserve after single precision solving. Accepted outputs are
checked as approximate solutions, with all these quality gates:

- Cost gap divided by `1 + optimal nonnegative cost` is at most `0.001` (0.1%).
- Postprocessing changes each wheel torque by at most `0.003 N m`.
- Postprocessing changes each normalized motion task by at most `0.0005`.

The nonnegative cost includes tracking error, PID reference, previous command
and power penalties. Reports retain the absolute cost gap and correction sizes.
These gates do not claim exact optimality, and do not relax physical bounds or
power feasibility checks.
