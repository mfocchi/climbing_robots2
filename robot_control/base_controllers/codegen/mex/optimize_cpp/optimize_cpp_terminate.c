/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_terminate.c
 *
 * Code generation for function 'optimize_cpp_terminate'
 *
 */

/* Include files */
#include "optimize_cpp_terminate.h"
#include "_coder_optimize_cpp_mex.h"
#include "optimize_cpp_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void optimize_cpp_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void optimize_cpp_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp_terminate.c) */
