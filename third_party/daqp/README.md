# DAQP for Helm allocation

- Upstream: <https://github.com/darnstrom/daqp>
- Version: 0.10.3, commit `c9dfe3416a29051c9677042c119ef1f4df184d3f`.
- License: MIT; see [LICENSE](LICENSE).
- Files: upstream `include/*.h` and `src/*.c`. The source is pinned to the tested
  commit rather than downloaded during firmware builds.

The firmware compiles this copy with `DAQP_SINGLE_PRECISION` and
`DAQP_NO_SOFT_WEIGHTS`; profiling is disabled. `HelmDaqpStorage.hpp` supplies
per-instance fixed arrays for an 8-variable, 57-constraint ordinary QP. Its
update and solve path does not call the allocating setup/free APIs. Equality
reduction, hierarchical QPs, soft constraints, AVI and branch-and-bound are not
enabled by this adapter.

## Local modification

Each C file that includes `<math.h>` also includes `HelmDaqpMath.h` immediately
afterwards. Under `DAQP_SINGLE_PRECISION`, this shim selects `sqrtf` and `fabsf`
for the upstream `sqrt` and `fabs` calls. It does not change standard library
declarations or the solver algorithm. This avoids software double square roots
on the STM32F407's single-precision FPU.

Floating constants in the C sources and headers also have an `f` suffix. This
mechanical change skips strings, comments and constants that already have a
suffix. Upstream uses float `c_float` storage but unsuffixed expressions can
otherwise promote intermediate calculations to software double arithmetic on
Cortex-M4F. The values and solver structure are unchanged.

To reproduce the vendor copy, copy the upstream headers and C sources from the
pinned commit, preserve LICENSE, and apply the include insertion and float
literal conversion above. The accompanying `adapt_source.py` performs these
local source adjustments on a clean copy. No
upstream allocating setup routine is needed by the fixed-array adapter.
