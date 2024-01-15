/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * integrate_dynamics.c
 *
 * Code generation for function 'integrate_dynamics'
 *
 */

/* Include files */
#include "integrate_dynamics.h"
#include "dynamics.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_mexutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void integrate_dynamics(real_T x0[6], real_T dt, real_T n_steps, const
  emxArray_real_T *Fr_l, const emxArray_real_T *Fr_r, const real_T Fleg[3],
  const char_T method[3], real_T params_m, real_T params_b, const real_T
  params_p_a1[3], const real_T params_p_a2[3], real_T params_g, real_T
  params_T_th, real_T *t_, emxArray_real_T *x_vec, emxArray_real_T *t_vec)
{
  static const int32_T iv[2] = { 1, 15 };

  static const char_T b[3] = { 'e', 'u', 'l' };

  static const char_T b_b[3] = { 'r', 'k', '4' };

  emxArray_real_T *b_x_vec;
  const mxArray *m;
  const mxArray *y;
  real_T b_x0[6];
  real_T dv[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  real_T a_tmp;
  real_T d;
  real_T d1;
  real_T k_2_tmp;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);

  /* verify is a column vector */
  *t_ = 0.0;
  i = x_vec->size[0] * x_vec->size[1];
  x_vec->size[0] = 6;
  x_vec->size[1] = 1;
  emxEnsureCapacity_real_T(x_vec, i);
  for (i = 0; i < 6; i++) {
    x_vec->data[i] = x0[i];
  }

  i = t_vec->size[0] * t_vec->size[1];
  t_vec->size[0] = 1;
  t_vec->size[1] = 1;
  emxEnsureCapacity_real_T(t_vec, i);
  t_vec->data[0] = 0.0;
  emxInit_real_T(&b_x_vec, 2, true);
  if (memcmp(&method[0], &b[0], 3) == 0) {
    /*  forwatd euler */
    i = (int32_T)(n_steps - 1.0);
    for (b_i = 0; b_i < i; b_i++) {
      dynamics(*t_, x0, Fr_l->data[b_i], Fr_r->data[b_i], Fleg, params_m,
               params_b, params_p_a1, params_p_a2, params_g, params_T_th, dv);
      for (i1 = 0; i1 < 6; i1++) {
        x0[i1] += dt * dv[i1];
      }

      /*  we have time invariant dynamics so t wont count */
      *t_ += dt;
      i1 = b_x_vec->size[0] * b_x_vec->size[1];
      b_x_vec->size[0] = 6;
      loop_ub = x_vec->size[1];
      b_x_vec->size[1] = x_vec->size[1] + 1;
      emxEnsureCapacity_real_T(b_x_vec, i1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (i2 = 0; i2 < 6; i2++) {
          i3 = i2 + 6 * i1;
          b_x_vec->data[i3] = x_vec->data[i3];
        }
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_x_vec->data[i1 + 6 * x_vec->size[1]] = x0[i1];
      }

      i1 = x_vec->size[0] * x_vec->size[1];
      x_vec->size[0] = 6;
      x_vec->size[1] = b_x_vec->size[1];
      emxEnsureCapacity_real_T(x_vec, i1);
      loop_ub = b_x_vec->size[0] * b_x_vec->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        x_vec->data[i1] = b_x_vec->data[i1];
      }

      i1 = t_vec->size[1];
      i2 = t_vec->size[0] * t_vec->size[1];
      t_vec->size[1]++;
      emxEnsureCapacity_real_T(t_vec, i2);
      t_vec->data[i1] = *t_;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else if (memcmp(&method[0], &b_b[0], 3) == 0) {
    /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/ */
    /*  we have  time invariant dynamics so t wont count */
    i = (int32_T)(n_steps - 1.0);
    for (b_i = 0; b_i < i; b_i++) {
      d = Fr_l->data[b_i];
      d1 = Fr_r->data[b_i];
      dynamics(*t_, x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, k_1);
      a_tmp = 0.5 * dt;
      for (i1 = 0; i1 < 6; i1++) {
        b_x0[i1] = x0[i1] + a_tmp * k_1[i1];
      }

      k_2_tmp = *t_ + 0.5 * dt;
      dynamics(k_2_tmp, b_x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, k_2);
      for (i1 = 0; i1 < 6; i1++) {
        b_x0[i1] = x0[i1] + a_tmp * k_2[i1];
      }

      dynamics(k_2_tmp, b_x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, k_3);
      for (i1 = 0; i1 < 6; i1++) {
        b_x0[i1] = x0[i1] + k_3[i1] * dt;
      }

      *t_ += dt;
      dynamics(*t_, b_x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, dv);
      for (i1 = 0; i1 < 6; i1++) {
        x0[i1] += 0.16666666666666666 * (((k_1[i1] + 2.0 * k_2[i1]) + 2.0 *
          k_3[i1]) + dv[i1]) * dt;
      }

      i1 = b_x_vec->size[0] * b_x_vec->size[1];
      b_x_vec->size[0] = 6;
      loop_ub = x_vec->size[1];
      b_x_vec->size[1] = x_vec->size[1] + 1;
      emxEnsureCapacity_real_T(b_x_vec, i1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (i2 = 0; i2 < 6; i2++) {
          i3 = i2 + 6 * i1;
          b_x_vec->data[i3] = x_vec->data[i3];
        }
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_x_vec->data[i1 + 6 * x_vec->size[1]] = x0[i1];
      }

      i1 = x_vec->size[0] * x_vec->size[1];
      x_vec->size[0] = 6;
      x_vec->size[1] = b_x_vec->size[1];
      emxEnsureCapacity_real_T(x_vec, i1);
      loop_ub = b_x_vec->size[0] * b_x_vec->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        x_vec->data[i1] = b_x_vec->data[i1];
      }

      i1 = t_vec->size[1];
      i2 = t_vec->size[0] * t_vec->size[1];
      t_vec->size[1]++;
      emxEnsureCapacity_real_T(t_vec, i2);
      t_vec->data[i1] = *t_;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &cv[0]);
    emlrtAssign(&y, m);
    disp(y, &emlrtMCI);
  }

  emxFree_real_T(&b_x_vec);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (integrate_dynamics.c) */
