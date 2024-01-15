/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeConstraints_.h
 *
 * Code generation for function 'computeConstraints_'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
int32_T computeConstraints_(const real_T c_obj_nonlcon_tunableEnvironmen[3],
  const real_T d_obj_nonlcon_tunableEnvironmen[3], real_T
  e_obj_nonlcon_tunableEnvironmen, real_T f_obj_nonlcon_tunableEnvironmen, const
  param *g_obj_nonlcon_tunableEnvironmen, int32_T obj_mCineq, const
  emxArray_real_T *x, emxArray_real_T *Cineq_workspace, int32_T ineq0);

/* End of code generation (computeConstraints_.h) */
