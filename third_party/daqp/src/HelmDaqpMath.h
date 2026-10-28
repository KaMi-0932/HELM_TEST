#ifndef HELM_DAQP_MATH_H
#define HELM_DAQP_MATH_H

/* Included after math.h in the vendored C files, so these macros cannot alter
 * the standard library's declarations. Cortex-M4F implements float arithmetic. */
#include <math.h>
#ifdef DAQP_SINGLE_PRECISION
#define sqrt(value) sqrtf(value)
#define fabs(value) fabsf(value)
#endif

#endif
