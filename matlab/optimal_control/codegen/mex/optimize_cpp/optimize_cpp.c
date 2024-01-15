/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp.c
 *
 * Code generation for function 'optimize_cpp'
 *
 */

/* Include files */
#include "optimize_cpp.h"
#include "computePositionVelocity.h"
#include "computeRollout.h"
#include "diff.h"
#include "fmincon.h"
#include "integrate_dynamics.h"
#include "norm.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void anon(const real_T p0[3], const real_T pf[3], real_T Fleg_max, real_T mu,
          const param *params, const emxArray_real_T *x, emxArray_real_T
          *varargout_1)
{
  emxArray_real_T *Fr_l;
  emxArray_real_T *b_t;
  emxArray_real_T *b_x;
  emxArray_real_T *p;
  emxArray_real_T *states;
  real_T dv[6];
  real_T Fleg[3];
  real_T absxk;
  real_T b_params;
  real_T b_scale;
  real_T b_y;
  real_T c_idx_0;
  real_T c_idx_1;
  real_T c_idx_2;
  real_T scale;
  real_T t;
  real_T y;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&Fr_l, 2, true);

  /*  ineq are <= 0 */
  Fleg[0] = x->data[0];
  Fleg[1] = x->data[1];
  Fleg[2] = x->data[2];
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  loop_ub = (int32_T)muDoubleScalarFloor(params->N_dyn - 1.0);
  Fr_l->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(Fr_l, i);
  for (i = 0; i <= loop_ub; i++) {
    Fr_l->data[i] = x->data[(int32_T)(params->num_params + (real_T)(i + 1)) - 1];
  }

  scale = (params->num_params + params->N_dyn) + 1.0;
  y = params->num_params + 2.0 * params->N_dyn;
  if (scale > y) {
    i = 0;
    i1 = 0;
  } else {
    i = (int32_T)scale - 1;
    i1 = (int32_T)y;
  }

  /*  check they are column vectors */
  /*  size not known */
  varargout_1->size[0] = 1;
  varargout_1->size[1] = 0;

  /*  number of constraints */
  /*  already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes */
  /*  variable intergration step */
  /*  single shooting */
  b_scale = 3.3121686421112381E-170;
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p0[0] - params->p_a1[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  absxk = muDoubleScalarAbs(p0[0] - params->p_a2[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = muDoubleScalarAbs(p0[1] - params->p_a1[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  absxk = muDoubleScalarAbs(p0[1] - params->p_a2[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = muDoubleScalarAbs(p0[2] - params->p_a1[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  absxk = muDoubleScalarAbs(p0[2] - params->p_a2[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  emxInit_real_T(&b_x, 2, true);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  y = scale * muDoubleScalarSqrt(y);
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = b_y;
  dv[2] = y;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  i2 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  loop_ub = i1 - i;
  b_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_x, i2);
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_x->data[i1] = x->data[i + i1];
  }

  emxInit_real_T(&states, 2, true);
  emxInit_real_T(&b_t, 2, true);
  computeRollout(dv, x->data[3] / (params->N_dyn - 1.0), params->N_dyn, Fr_l,
                 b_x, Fleg, params->int_method, params->int_steps, params->m,
                 params->b, params->p_a1, params->p_a2, params->g, params->T_th,
                 states, b_t);
  loop_ub = states->size[1];
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  Fr_l->size[1] = states->size[1];
  emxEnsureCapacity_real_T(Fr_l, i);
  for (i = 0; i < loop_ub; i++) {
    Fr_l->data[i] = states->data[6 * i];
  }

  loop_ub = states->size[1];
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_x, i);
  for (i = 0; i < loop_ub; i++) {
    b_x->data[i] = states->data[6 * i + 1];
  }

  loop_ub = states->size[1];
  i = b_t->size[0] * b_t->size[1];
  b_t->size[0] = 1;
  b_t->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_t, i);
  for (i = 0; i < loop_ub; i++) {
    b_t->data[i] = states->data[6 * i + 2];
  }

  emxFree_real_T(&states);
  emxInit_real_T(&p, 2, true);
  computePositionVelocity(params->b, Fr_l, b_x, b_t, p);

  /* only position */
  /*  I assume px py pz  are row vectors */
  /*  init struct foc C++ code generation */
  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0  */
  emxFree_real_T(&b_x);
  emxFree_real_T(&b_t);
  if (params->obstacle_avoidance) {
    /* [0; 3;-7.5]; */
    /* px > sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2); */
    /* -px + sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2)<0 */
    /*  better implementaiton with complex numbers for code generation */
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      scale = p->data[3 * b_i + 2] - params->obstacle_location[2];
      y = p->data[3 * b_i + 1] - params->obstacle_location[1];
      scale = (2.25 - 3.0001760103259394 * (scale * scale)) - y * y;

      /* %%add ineq only if inside sphere */
      if (scale > 0.0) {
        i1 = Fr_l->size[0] * Fr_l->size[1];
        Fr_l->size[0] = 1;
        Fr_l->size[1] = varargout_1->size[1] + 1;
        emxEnsureCapacity_real_T(Fr_l, i1);
        loop_ub = varargout_1->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          Fr_l->data[i1] = varargout_1->data[i1];
        }

        Fr_l->data[varargout_1->size[1]] = ((-p->data[3 * b_i] +
          params->obstacle_location[0]) + muDoubleScalarSqrt(scale)) +
          params->jump_clearance;
        i1 = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[0] = 1;
        varargout_1->size[1] = Fr_l->size[1];
        emxEnsureCapacity_real_T(varargout_1, i1);
        loop_ub = Fr_l->size[0] * Fr_l->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          varargout_1->data[i1] = Fr_l->data[i1];
        }
      } else {
        i1 = varargout_1->size[1];
        i2 = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[1]++;
        emxEnsureCapacity_real_T(varargout_1, i2);
        varargout_1->data[i1] = -p->data[3 * b_i];
      }

      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      i1 = varargout_1->size[1];
      i2 = varargout_1->size[0] * varargout_1->size[1];
      varargout_1->size[1]++;
      emxEnsureCapacity_real_T(varargout_1, i2);
      varargout_1->data[i1] = -p->data[3 * b_i];

      /* ineq = [ineq -psi(i) ];  */
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  }

  /*  % % debug */
  /*  disp('after wall') */
  /*  length(ineq) */
  /*  2- N_dyn constraints on retraction force   -Fr_max < Fr < 0  */
  /*  unilaterality */
  /*  debug */
  /*  disp('after Fr') */
  /*  length(ineq) */
  /*  constraints on impulse force */
  scale = params->contact_normal[1] * 0.0 - params->contact_normal[2];
  y = params->contact_normal[2] * 0.0 - params->contact_normal[0] * 0.0;
  b_scale = params->contact_normal[0] - params->contact_normal[1] * 0.0;
  c_idx_0 = params->contact_normal[1] - params->contact_normal[2] * 0.0;
  c_idx_1 = params->contact_normal[2] * 0.0 - params->contact_normal[0];
  c_idx_2 = params->contact_normal[0] * 0.0 - params->contact_normal[1] * 0.0;

  /*  compute components */
  b_params = (params->contact_normal[0] * Fleg[0] + params->contact_normal[1] *
              Fleg[1]) + params->contact_normal[2] * Fleg[2];
  y = ((y * params->contact_normal[2] - b_scale * params->contact_normal[1]) *
       Fleg[0] + (b_scale * params->contact_normal[0] - scale *
                  params->contact_normal[2]) * Fleg[1]) + (scale *
    params->contact_normal[1] - y * params->contact_normal[0]) * Fleg[2];

  /* 3 ------------------------------ Fleg constraints */
  /*  unilateral */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  varargout_1->data[i] = -b_params;

  /* (Fun >fmin )  */
  /* max force  */
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(Fleg[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  absxk = muDoubleScalarAbs(Fleg[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  scale = ((c_idx_1 * params->contact_normal[2] - c_idx_2 *
            params->contact_normal[1]) * Fleg[0] + (c_idx_2 *
            params->contact_normal[0] - c_idx_0 * params->contact_normal[2]) *
           Fleg[1]) + (c_idx_0 * params->contact_normal[1] - c_idx_1 *
                       params->contact_normal[0]) * Fleg[2];
  absxk = muDoubleScalarAbs(Fleg[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  b_y = b_scale * muDoubleScalarSqrt(b_y);
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  varargout_1->data[i] = b_y - Fleg_max;

  /* (Fun < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    i = Fr_l->size[0] * Fr_l->size[1];
    Fr_l->size[0] = 1;
    Fr_l->size[1] = varargout_1->size[1] + 1;
    emxEnsureCapacity_real_T(Fr_l, i);
    loop_ub = varargout_1->size[1];
    for (i = 0; i < loop_ub; i++) {
      Fr_l->data[i] = varargout_1->data[i];
    }

    Fr_l->data[varargout_1->size[1]] = muDoubleScalarSqrt(y * y + scale * scale)
      - mu * b_params;
    i = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = 1;
    varargout_1->size[1] = Fr_l->size[1];
    emxEnsureCapacity_real_T(varargout_1, i);
    loop_ub = Fr_l->size[0] * Fr_l->size[1];
    for (i = 0; i < loop_ub; i++) {
      varargout_1->data[i] = Fr_l->data[i];
    }

    /* friction constraints */
  }

  emxFree_real_T(&Fr_l);

  /*   */
  /*  % debug */
  /*  disp('after Fu') */
  /*  length(ineq) */
  /*  final point  variable slack   */
  /* ineq= [ineq norm(p_f - pf) - x(num_params+N+N_dyn+1)]; */
  /*  4- initial final point  fixed slack  */
  /* *norm(p0 - pf);  */
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1)] - pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1) + 1] - pf[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1) + 2] - pf[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  b_y = b_scale * muDoubleScalarSqrt(b_y);
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  varargout_1->data[i] = b_y - 0.02;

  /* 5 - jump clearance */
  if (!params->obstacle_avoidance) {
    i = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    varargout_1->data[i] = -p->data[3 * ((int32_T)(params->N_dyn / 2.0) - 1)] +
      params->jump_clearance;
  }

  emxFree_real_T(&p);

  /*  if any(isinf(ineq)) */
  /*      disp('Infn in constraint') */
  /*      find(isinf(ineq))  */
  /*      isinf(ineq) */
  /*  end */
  /*  if any(isnan(ineq)) */
  /*      disp('Nan in constraint') */
  /*      find(isnan(ineq)) */
  /*      isnan(ineq) */
  /*  end */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

real_T b_anon(const real_T p0[3], real_T params_m, real_T params_num_params,
              const char_T params_int_method[3], real_T params_N_dyn, real_T
              params_int_steps, real_T params_b, const real_T params_p_a1[3],
              const real_T params_p_a2[3], real_T params_g, real_T params_w1,
              real_T params_w2, real_T params_T_th, const emxArray_real_T *x)
{
  emxArray_real_T *Fr_l;
  emxArray_real_T *b_states;
  emxArray_real_T *b_t;
  emxArray_real_T *b_x;
  emxArray_real_T *c_states;
  emxArray_real_T *d_states;
  emxArray_real_T *e_states;
  emxArray_real_T *p;
  emxArray_real_T *pd;
  emxArray_real_T *states;
  real_T dv[6];
  real_T c_x[3];
  real_T absxk;
  real_T b_scale;
  real_T b_y;
  real_T dt_dyn;
  real_T scale;
  real_T t;
  real_T varargout_1;
  real_T y;
  int32_T i;
  int32_T k;
  int32_T loop_ub;
  int32_T nx;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&Fr_l, 2, true);
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  loop_ub = (int32_T)muDoubleScalarFloor(params_N_dyn - 1.0);
  Fr_l->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(Fr_l, i);
  for (i = 0; i <= loop_ub; i++) {
    Fr_l->data[i] = x->data[(int32_T)(params_num_params + (real_T)(i + 1)) - 1];
  }

  dt_dyn = (params_num_params + params_N_dyn) + 1.0;
  scale = params_num_params + 2.0 * params_N_dyn;
  if (dt_dyn > scale) {
    i = 0;
    k = 0;
  } else {
    i = (int32_T)dt_dyn - 1;
    k = (int32_T)scale;
  }

  /*  check they are column vectors */
  /*  variable intergration step */
  dt_dyn = x->data[3] / (params_N_dyn - 1.0);

  /*  single shooting */
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p0[0] - params_p_a1[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = muDoubleScalarAbs(p0[0] - params_p_a2[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  absxk = muDoubleScalarAbs(p0[1] - params_p_a1[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = muDoubleScalarAbs(p0[1] - params_p_a2[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  absxk = muDoubleScalarAbs(p0[2] - params_p_a1[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = muDoubleScalarAbs(p0[2] - params_p_a2[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  emxInit_real_T(&b_x, 2, true);
  y = scale * muDoubleScalarSqrt(y);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = y;
  dv[2] = b_y;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  nx = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  loop_ub = k - i;
  b_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_x, nx);
  for (k = 0; k < loop_ub; k++) {
    b_x->data[k] = x->data[i + k];
  }

  emxInit_real_T(&states, 2, true);
  emxInit_real_T(&b_t, 2, true);
  c_x[0] = x->data[0];
  c_x[1] = x->data[1];
  c_x[2] = x->data[2];
  computeRollout(dv, dt_dyn, params_N_dyn, Fr_l, b_x, c_x, params_int_method,
                 params_int_steps, params_m, params_b, params_p_a1, params_p_a2,
                 params_g, params_T_th, states, b_t);
  nx = states->size[1];
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_x, k);
  for (k = 0; k < nx; k++) {
    b_x->data[k] = states->data[6 * k];
  }

  emxInit_real_T(&b_states, 2, true);
  nx = states->size[1];
  k = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, k);
  for (k = 0; k < nx; k++) {
    b_states->data[k] = states->data[6 * k + 1];
  }

  nx = states->size[1];
  k = b_t->size[0] * b_t->size[1];
  b_t->size[0] = 1;
  b_t->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_t, k);
  for (k = 0; k < nx; k++) {
    b_t->data[k] = states->data[6 * k + 2];
  }

  emxInit_real_T(&c_states, 2, true);
  nx = states->size[1];
  k = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, k);
  for (k = 0; k < nx; k++) {
    c_states->data[k] = states->data[6 * k + 3];
  }

  emxInit_real_T(&d_states, 2, true);
  nx = states->size[1];
  k = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, k);
  for (k = 0; k < nx; k++) {
    d_states->data[k] = states->data[6 * k + 4];
  }

  emxInit_real_T(&e_states, 2, true);
  nx = states->size[1];
  k = e_states->size[0] * e_states->size[1];
  e_states->size[0] = 1;
  e_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(e_states, k);
  for (k = 0; k < nx; k++) {
    e_states->data[k] = states->data[6 * k + 5];
  }

  emxInit_real_T(&p, 2, true);
  emxInit_real_T(&pd, 2, true);
  b_computePositionVelocity(params_b, b_x, b_states, b_t, c_states, d_states,
    e_states, p, pd);

  /*  be careful there are only N values in this vector the path migh be */
  /*  underestimated! */
  /*      deltax = diff(p(1,:));  % diff(X); */
  /*      deltay = diff(p(2,:));   % diff(Y); */
  /*      deltaz = diff(p(3,:));    % diff(Z); */
  /*      path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2)); */
  /* minimize the final kin energy at contact */
  /*  minimize hoist work / energy consumption for the hoist work we integrathe the power on a rough grid */
  k = b_t->size[0] * b_t->size[1];
  b_t->size[0] = 1;
  b_t->size[1] = Fr_l->size[1];
  emxEnsureCapacity_real_T(b_t, k);
  nx = Fr_l->size[1];
  emxFree_real_T(&e_states);
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  emxFree_real_T(&pd);
  emxFree_real_T(&p);
  for (k = 0; k < nx; k++) {
    b_t->data[k] = Fr_l->data[k] * states->data[6 * k + 4];
  }

  nx = b_t->size[1];
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = b_t->size[1];
  emxEnsureCapacity_real_T(b_x, k);
  for (k = 0; k < nx; k++) {
    b_x->data[k] = muDoubleScalarAbs(b_t->data[k]);
  }

  k = b_t->size[0] * b_t->size[1];
  b_t->size[0] = 1;
  b_t->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_t, k);
  for (k = 0; k < loop_ub; k++) {
    b_t->data[k] = x->data[i + k] * states->data[6 * k + 5];
  }

  emxFree_real_T(&states);
  nx = b_t->size[1];
  k = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = b_t->size[1];
  emxEnsureCapacity_real_T(b_states, k);
  for (k = 0; k < nx; k++) {
    b_states->data[k] = muDoubleScalarAbs(b_t->data[k]);
  }

  /* assume the motor is not regenreating */
  /*  smoothnes: minimize jerky control action TODO this is wrong! it goes */
  /*  to -180 and stays there! with sum(abs(diff(Fr_r))) + */
  /*  sum(abs(diff(Fr_l))) but does not converge at all  */
  /*  this is nice but slower */
  /* fprintf("hoist_work %f\n ",hoist_work)     */
  /* fprintf("smooth %f\n ", smooth) */
  /* fprintf("tempo %f\n ", w6*Tf) */
  /* cost =  0.001 * params.w1 *Ekinfcost +   params.w4 *smooth ;% converge */
  /* super slowly */
  k = b_x->size[0] * b_x->size[1];
  nx = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  emxEnsureCapacity_real_T(b_x, nx);
  nx = k - 1;
  for (k = 0; k <= nx; k++) {
    b_x->data[k] *= dt_dyn;
  }

  nx = b_x->size[1];
  if (b_x->size[1] == 0) {
    y = 0.0;
  } else {
    y = b_x->data[0];
    for (k = 2; k <= nx; k++) {
      y += b_x->data[k - 1];
    }
  }

  k = b_states->size[0] * b_states->size[1];
  nx = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  emxEnsureCapacity_real_T(b_states, nx);
  nx = k - 1;
  for (k = 0; k <= nx; k++) {
    b_states->data[k] *= dt_dyn;
  }

  nx = b_states->size[1];
  if (b_states->size[1] == 0) {
    b_y = 0.0;
  } else {
    b_y = b_states->data[0];
    for (k = 2; k <= nx; k++) {
      b_y += b_states->data[k - 1];
    }
  }

  emxFree_real_T(&b_states);
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_x, k);
  for (k = 0; k < loop_ub; k++) {
    b_x->data[k] = x->data[i + k];
  }

  diff(b_x, b_t);
  nx = b_t->size[1];
  emxFree_real_T(&b_x);
  if (b_t->size[1] == 0) {
    dt_dyn = 0.0;
  } else {
    dt_dyn = b_t->data[0];
    for (k = 2; k <= nx; k++) {
      dt_dyn += b_t->data[k - 1];
    }
  }

  diff(Fr_l, b_t);
  nx = b_t->size[1];
  emxFree_real_T(&Fr_l);
  if (b_t->size[1] == 0) {
    scale = 0.0;
  } else {
    scale = b_t->data[0];
    for (k = 2; k <= nx; k++) {
      scale += b_t->data[k - 1];
    }
  }

  emxFree_real_T(&b_t);
  varargout_1 = params_w2 * (y + b_y) + params_w1 * (dt_dyn + scale);

  /*  72 iter */
  /*  cost =    params.w4 *smooth ;% 27 iter */
  /*  cost =    params.w4 *smooth_correct ;% 96 iter */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

void optimize_cpp(const real_T p0[3], const real_T pf[3], real_T Fleg_max,
                  real_T Fr_max, real_T mu, const param *params, struct0_T
                  *solution)
{
  emxArray_real_T *Fr_l;
  emxArray_real_T *b_Fleg_max;
  emxArray_real_T *b_states;
  emxArray_real_T *c_Fleg_max;
  emxArray_real_T *c_states;
  emxArray_real_T *d_Fleg_max;
  emxArray_real_T *d_states;
  emxArray_real_T *pd_fine;
  emxArray_real_T *states;
  emxArray_real_T *states_rough;
  emxArray_real_T *x;
  real_T state0[6];
  real_T Fleg[3];
  real_T b_this_tunableEnvironment_f2[3];
  real_T this_tunableEnvironment_f1[3];
  real_T this_tunableEnvironment_f2[3];
  real_T c;
  real_T d;
  real_T d1;
  real_T l1_tmp;
  real_T l2_tmp;
  real_T n_samples;
  real_T state0_tmp;
  real_T t_;
  real_T unusedU1;
  real_T y_tmp;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T nx;
  uint32_T rough_count;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&b_Fleg_max, 2, true);

  /* make it column vector */
  /* for eval solution */
  /*  needs to be fixed for code generation */
  /* compute initial state from jump param */
  Fleg[0] = p0[0] - params->p_a1[0];
  Fleg[1] = p0[1] - params->p_a1[1];
  Fleg[2] = p0[2] - params->p_a1[2];
  l1_tmp = b_norm(Fleg);
  Fleg[0] = p0[0] - params->p_a2[0];
  Fleg[1] = p0[1] - params->p_a2[1];
  Fleg[2] = p0[2] - params->p_a2[2];
  l2_tmp = b_norm(Fleg);

  /* pendulum period */
  /*  half period TODO replace with linearized x0(2) = l10 */
  /* opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r */
  /*  % does not always satisfy bounds */
  tic();
  Fleg[0] = p0[0];
  this_tunableEnvironment_f2[0] = pf[0];
  this_tunableEnvironment_f1[0] = p0[0];
  b_this_tunableEnvironment_f2[0] = pf[0];
  Fleg[1] = p0[1];
  this_tunableEnvironment_f2[1] = pf[1];
  this_tunableEnvironment_f1[1] = p0[1];
  b_this_tunableEnvironment_f2[1] = pf[1];
  Fleg[2] = p0[2];
  this_tunableEnvironment_f2[2] = pf[2];
  this_tunableEnvironment_f1[2] = p0[2];
  b_this_tunableEnvironment_f2[2] = pf[2];
  i = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  i1 = ((int32_T)params->N_dyn + (int32_T)params->N_dyn) + 4;
  b_Fleg_max->size[1] = i1;
  emxEnsureCapacity_real_T(b_Fleg_max, i);
  b_Fleg_max->data[0] = Fleg_max;
  b_Fleg_max->data[1] = Fleg_max;
  b_Fleg_max->data[2] = Fleg_max;
  b_Fleg_max->data[3] = 6.2831853071795862 * muDoubleScalarSqrt(l1_tmp /
    params->g) / 4.0;
  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    b_Fleg_max->data[i + 4] = 0.0;
  }

  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    b_Fleg_max->data[(i + (int32_T)params->N_dyn) + 4] = 0.0;
  }

  emxInit_real_T(&c_Fleg_max, 2, true);
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = i1;
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  c_Fleg_max->data[0] = -Fleg_max;
  c_Fleg_max->data[1] = -Fleg_max;
  c_Fleg_max->data[2] = -Fleg_max;
  c_Fleg_max->data[3] = 0.01;
  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    c_Fleg_max->data[i + 4] = -Fr_max;
  }

  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    c_Fleg_max->data[(i + (int32_T)params->N_dyn) + 4] = -Fr_max;
  }

  emxInit_real_T(&d_Fleg_max, 2, true);
  i = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = i1;
  emxEnsureCapacity_real_T(d_Fleg_max, i);
  d_Fleg_max->data[0] = Fleg_max;
  d_Fleg_max->data[1] = Fleg_max;
  d_Fleg_max->data[2] = Fleg_max;
  d_Fleg_max->data[3] = rtInf;
  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    d_Fleg_max->data[i + 4] = 0.0;
  }

  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    d_Fleg_max->data[(i + (int32_T)params->N_dyn) + 4] = 0.0;
  }

  emxInit_real_T(&x, 2, true);
  emxInit_real_T(&Fr_l, 2, true);
  fmincon(Fleg, this_tunableEnvironment_f2, params, b_Fleg_max, c_Fleg_max,
          d_Fleg_max, this_tunableEnvironment_f1, b_this_tunableEnvironment_f2,
          Fleg_max, Fr_max, mu, params, x, &solution->cost,
          &solution->problem_solved, &solution->optim_output.iterations,
          &solution->optim_output.funcCount, solution->optim_output.algorithm,
          &solution->optim_output.constrviolation,
          &solution->optim_output.stepsize, &solution->optim_output.lssteplength,
          &solution->optim_output.firstorderopt);
  toc();

  /* eval trajectory */
  Fleg[0] = x->data[0];
  Fleg[1] = x->data[1];
  Fleg[2] = x->data[2];
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  loop_ub = (int32_T)muDoubleScalarFloor(params->N_dyn - 1.0);
  Fr_l->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(Fr_l, i);
  for (i = 0; i <= loop_ub; i++) {
    Fr_l->data[i] = x->data[(int32_T)(params->num_params + (real_T)(i + 1)) - 1];
  }

  d = (params->num_params + params->N_dyn) + 1.0;
  d1 = params->num_params + 2.0 * params->N_dyn;
  if (d > d1) {
    i = 0;
    i1 = 0;
  } else {
    i = (int32_T)d - 1;
    i1 = (int32_T)d1;
  }

  /*  resample inputs  */
  n_samples = muDoubleScalarFloor(x->data[3] / 0.001);
  nx = solution->c->size[0] * solution->c->size[1];
  solution->c->size[0] = 1;
  solution->c->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(solution->c, nx);
  nx = solution->Fr_r_fine->size[0] * solution->Fr_r_fine->size[1];
  solution->Fr_r_fine->size[0] = 1;
  solution->Fr_r_fine->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(solution->Fr_r_fine, nx);
  rough_count = 1U;
  t_ = 0.0;
  nx = (int32_T)n_samples;
  for (b_i = 0; b_i < nx; b_i++) {
    t_ += 0.001;
    if (t_ >= n_samples * 0.001 / (params->N_dyn - 1.0)) {
      rough_count++;
      t_ = 0.0;
    }

    solution->c->data[b_i] = Fr_l->data[(int32_T)rough_count - 1];
    solution->Fr_r_fine->data[b_i] = x->data[(i + (int32_T)rough_count) - 1];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  /*  single shooting */
  state0_tmp = muDoubleScalarAtan2(p0[0], -p0[2]);
  state0[0] = state0_tmp;
  state0[1] = l1_tmp;
  state0[2] = l2_tmp;
  state0[3] = 0.0;
  state0[4] = 0.0;
  state0[5] = 0.0;

  /*  course integration */
  nx = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_loop_ub = i1 - i;
  b_Fleg_max->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(b_Fleg_max, nx);
  for (i1 = 0; i1 < b_loop_ub; i1++) {
    b_Fleg_max->data[i1] = x->data[i + i1];
  }

  emxInit_real_T(&states, 2, true);
  computeRollout(state0, x->data[3] / (params->N_dyn - 1.0), params->N_dyn, Fr_l,
                 b_Fleg_max, Fleg, params->int_method, params->int_steps,
                 params->m, params->b, params->p_a1, params->p_a2, params->g,
                 params->T_th, states, solution->time);
  nx = states->size[1];
  i1 = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    b_Fleg_max->data[i1] = states->data[6 * i1];
  }

  nx = states->size[1];
  i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    c_Fleg_max->data[i1] = states->data[6 * i1 + 1];
  }

  nx = states->size[1];
  i1 = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    d_Fleg_max->data[i1] = states->data[6 * i1 + 2];
  }

  emxInit_real_T(&b_states, 2, true);
  nx = states->size[1];
  i1 = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, i1);
  for (i1 = 0; i1 < nx; i1++) {
    b_states->data[i1] = states->data[6 * i1 + 3];
  }

  emxInit_real_T(&c_states, 2, true);
  nx = states->size[1];
  i1 = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, i1);
  for (i1 = 0; i1 < nx; i1++) {
    c_states->data[i1] = states->data[6 * i1 + 4];
  }

  emxInit_real_T(&d_states, 2, true);
  nx = states->size[1];
  i1 = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, i1);
  for (i1 = 0; i1 < nx; i1++) {
    d_states->data[i1] = states->data[6 * i1 + 5];
  }

  b_computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
    b_states, c_states, d_states, solution->solution_constr.p, solution->p_fine);

  /*  fine integration  */
  /* init */
  i1 = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = solution->c->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, i1);
  nx = solution->c->size[0] * solution->c->size[1] - 1;
  for (i1 = 0; i1 <= nx; i1++) {
    b_Fleg_max->data[i1] = solution->c->data[i1];
  }

  i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->Fr_r_fine->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i1);
  nx = solution->Fr_r_fine->size[0] * solution->Fr_r_fine->size[1] - 1;
  for (i1 = 0; i1 <= nx; i1++) {
    c_Fleg_max->data[i1] = solution->Fr_r_fine->data[i1];
  }

  emxInit_real_T(&states_rough, 2, true);
  integrate_dynamics(state0, 0.001, n_samples, b_Fleg_max, c_Fleg_max, Fleg,
                     params->int_method, params->m, params->b, params->p_a1,
                     params->p_a2, params->g, params->T_th, &unusedU1,
                     states_rough, solution->time_fine);
  nx = states_rough->size[1];
  i1 = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    b_Fleg_max->data[i1] = states_rough->data[6 * i1];
  }

  nx = states_rough->size[1];
  i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    c_Fleg_max->data[i1] = states_rough->data[6 * i1 + 1];
  }

  nx = states_rough->size[1];
  i1 = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(d_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    d_Fleg_max->data[i1] = states_rough->data[6 * i1 + 2];
  }

  nx = states_rough->size[1];
  i1 = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(b_states, i1);
  for (i1 = 0; i1 < nx; i1++) {
    b_states->data[i1] = states_rough->data[6 * i1 + 3];
  }

  nx = states_rough->size[1];
  i1 = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(c_states, i1);
  for (i1 = 0; i1 < nx; i1++) {
    c_states->data[i1] = states_rough->data[6 * i1 + 4];
  }

  nx = states_rough->size[1];
  i1 = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(d_states, i1);
  for (i1 = 0; i1 < nx; i1++) {
    d_states->data[i1] = states_rough->data[6 * i1 + 5];
  }

  emxInit_real_T(&pd_fine, 2, true);
  b_computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
    b_states, c_states, d_states, solution->p_fine, pd_fine);

  /*  init struct foc C++ code generation */
  /* compute path */
  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  /*  check length is always l */
  /*      a = vecnorm(p) */
  /*      a -  ones(1,length(a))*l */
  i1 = solution->Ekin->size[0] * solution->Ekin->size[1];
  solution->Ekin->size[0] = 1;
  solution->Ekin->size[1] = solution->time_fine->size[1];
  emxEnsureCapacity_real_T(solution->Ekin, i1);
  nx = solution->time_fine->size[1];
  emxFree_real_T(&d_states);
  for (i1 = 0; i1 < nx; i1++) {
    solution->Ekin->data[i1] = 0.0;
  }

  t_ = 0.0;

  /*  kinetic energy at the beginning */
  y_tmp = params->m / 2.0;
  i1 = solution->time_fine->size[1];
  for (b_i = 0; b_i < i1; b_i++) {
    c = pd_fine->data[3 * b_i];
    unusedU1 = y_tmp * c * c;
    c = pd_fine->data[3 * b_i + 1];
    unusedU1 += y_tmp * c * c;
    c = pd_fine->data[3 * b_i + 2];
    unusedU1 += y_tmp * c * c;
    solution->Ekin->data[b_i] = unusedU1;
    t_ += unusedU1 * 0.001;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  nx = solution->solution_constr.p->size[1];
  i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    c_Fleg_max->data[i1] = solution->solution_constr.p->data[3 * i1];
  }

  diff(c_Fleg_max, d_Fleg_max);
  i1 = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = d_Fleg_max->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, i1);
  nx = d_Fleg_max->size[1];
  for (b_i = 0; b_i < nx; b_i++) {
    c = d_Fleg_max->data[b_i];
    b_Fleg_max->data[b_i] = c * c;
  }

  nx = solution->solution_constr.p->size[1];
  i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    c_Fleg_max->data[i1] = solution->solution_constr.p->data[3 * i1 + 1];
  }

  diff(c_Fleg_max, d_Fleg_max);
  i1 = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = d_Fleg_max->size[1];
  emxEnsureCapacity_real_T(b_states, i1);
  nx = d_Fleg_max->size[1];
  for (b_i = 0; b_i < nx; b_i++) {
    c = d_Fleg_max->data[b_i];
    b_states->data[b_i] = c * c;
  }

  nx = solution->solution_constr.p->size[1];
  i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i1);
  for (i1 = 0; i1 < nx; i1++) {
    c_Fleg_max->data[i1] = solution->solution_constr.p->data[3 * i1 + 2];
  }

  diff(c_Fleg_max, d_Fleg_max);
  i1 = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = d_Fleg_max->size[1];
  emxEnsureCapacity_real_T(c_states, i1);
  nx = d_Fleg_max->size[1];
  for (b_i = 0; b_i < nx; b_i++) {
    c = d_Fleg_max->data[b_i];
    c_states->data[b_i] = c * c;
  }

  i1 = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  nx = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  emxEnsureCapacity_real_T(b_Fleg_max, nx);
  nx = i1 - 1;
  for (i1 = 0; i1 <= nx; i1++) {
    b_Fleg_max->data[i1] = (b_Fleg_max->data[i1] + b_states->data[i1]) +
      c_states->data[i1];
  }

  emxFree_real_T(&c_states);
  emxFree_real_T(&b_states);
  nx = b_Fleg_max->size[1];
  for (b_i = 0; b_i < nx; b_i++) {
    b_Fleg_max->data[b_i] = muDoubleScalarSqrt(b_Fleg_max->data[b_i]);
  }

  nx = b_Fleg_max->size[1];
  if (b_Fleg_max->size[1] == 0) {
    unusedU1 = 0.0;
  } else {
    unusedU1 = b_Fleg_max->data[0];
    for (b_i = 2; b_i <= nx; b_i++) {
      unusedU1 += b_Fleg_max->data[b_i - 1];
    }
  }

  solution->path_length = unusedU1;
  this_tunableEnvironment_f2[0] = solution->solution_constr.p->data[0] - p0[0];
  this_tunableEnvironment_f2[1] = solution->solution_constr.p->data[1] - p0[1];
  this_tunableEnvironment_f2[2] = solution->solution_constr.p->data[2] - p0[2];
  solution->initial_error = b_norm(this_tunableEnvironment_f2);
  this_tunableEnvironment_f2[0] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1)] - pf[0];
  solution->Fleg[0] = Fleg[0];
  this_tunableEnvironment_f2[1] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1) + 1] - pf[1];
  solution->Fleg[1] = Fleg[1];
  this_tunableEnvironment_f2[2] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1) + 2] - pf[2];
  solution->Fleg[2] = Fleg[2];
  solution->final_error_real = b_norm(this_tunableEnvironment_f2);
  i1 = solution->Fr_l->size[0] * solution->Fr_l->size[1];
  solution->Fr_l->size[0] = 1;
  solution->Fr_l->size[1] = Fr_l->size[1];
  emxEnsureCapacity_real_T(solution->Fr_l, i1);
  nx = Fr_l->size[0] * Fr_l->size[1];
  for (i1 = 0; i1 < nx; i1++) {
    solution->Fr_l->data[i1] = Fr_l->data[i1];
  }

  i1 = solution->Fr_r->size[0] * solution->Fr_r->size[1];
  solution->Fr_r->size[0] = 1;
  solution->Fr_r->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_r, i1);
  for (i1 = 0; i1 < b_loop_ub; i1++) {
    solution->Fr_r->data[i1] = x->data[i + i1];
  }

  i = solution->p->size[0] * solution->p->size[1];
  solution->p->size[0] = 3;
  solution->p->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(solution->p, i);
  b_loop_ub = solution->solution_constr.p->size[0] * solution->
    solution_constr.p->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    solution->p->data[i] = solution->solution_constr.p->data[i];
  }

  b_loop_ub = states->size[1];
  i = solution->psi->size[0] * solution->psi->size[1];
  solution->psi->size[0] = 1;
  solution->psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psi, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->psi->data[i] = states->data[6 * i];
  }

  b_loop_ub = states->size[1];
  i = solution->l1->size[0] * solution->l1->size[1];
  solution->l1->size[0] = 1;
  solution->l1->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l1->data[i] = states->data[6 * i + 1];
  }

  b_loop_ub = states->size[1];
  i = solution->l2->size[0] * solution->l2->size[1];
  solution->l2->size[0] = 1;
  solution->l2->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l2->data[i] = states->data[6 * i + 2];
  }

  b_loop_ub = states->size[1];
  i = solution->psid->size[0] * solution->psid->size[1];
  solution->psid->size[0] = 1;
  solution->psid->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psid, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->psid->data[i] = states->data[6 * i + 3];
  }

  b_loop_ub = states->size[1];
  i = solution->l1d->size[0] * solution->l1d->size[1];
  solution->l1d->size[0] = 1;
  solution->l1d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1d, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l1d->data[i] = states->data[6 * i + 4];
  }

  b_loop_ub = states->size[1];
  i = solution->l2d->size[0] * solution->l2d->size[1];
  solution->l2d->size[0] = 1;
  solution->l2d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2d, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l2d->data[i] = states->data[6 * i + 5];
  }

  i = solution->Fr_l_fine->size[0] * solution->Fr_l_fine->size[1];
  solution->Fr_l_fine->size[0] = 1;
  solution->Fr_l_fine->size[1] = solution->c->size[1];
  emxEnsureCapacity_real_T(solution->Fr_l_fine, i);
  b_loop_ub = solution->c->size[0] * solution->c->size[1] - 1;
  for (i = 0; i <= b_loop_ub; i++) {
    solution->Fr_l_fine->data[i] = solution->c->data[i];
  }

  b_loop_ub = states_rough->size[1];
  i = solution->psi_fine->size[0] * solution->psi_fine->size[1];
  solution->psi_fine->size[0] = 1;
  solution->psi_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->psi_fine, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->psi_fine->data[i] = states_rough->data[6 * i];
  }

  b_loop_ub = states_rough->size[1];
  i = solution->l1_fine->size[0] * solution->l1_fine->size[1];
  solution->l1_fine->size[0] = 1;
  solution->l1_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l1_fine, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l1_fine->data[i] = states_rough->data[6 * i + 1];
  }

  b_loop_ub = states_rough->size[1];
  i = solution->l2_fine->size[0] * solution->l2_fine->size[1];
  solution->l2_fine->size[0] = 1;
  solution->l2_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l2_fine, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l2_fine->data[i] = states_rough->data[6 * i + 2];
  }

  b_loop_ub = states_rough->size[1];
  i = solution->psid_fine->size[0] * solution->psid_fine->size[1];
  solution->psid_fine->size[0] = 1;
  solution->psid_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->psid_fine, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->psid_fine->data[i] = states_rough->data[6 * i + 3];
  }

  b_loop_ub = states_rough->size[1];
  i = solution->l1d_fine->size[0] * solution->l1d_fine->size[1];
  solution->l1d_fine->size[0] = 1;
  solution->l1d_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l1d_fine, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l1d_fine->data[i] = states_rough->data[6 * i + 4];
  }

  b_loop_ub = states_rough->size[1];
  i = solution->l2d_fine->size[0] * solution->l2d_fine->size[1];
  solution->l2d_fine->size[0] = 1;
  solution->l2d_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l2d_fine, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->l2d_fine->data[i] = states_rough->data[6 * i + 5];
  }

  emxFree_real_T(&states_rough);
  solution->Tf = x->data[3];
  solution->Etot = 0.0;
  solution->Ekin0x = y_tmp * pd_fine->data[0] * pd_fine->data[0];
  solution->Ekin0y = y_tmp * pd_fine->data[1] * pd_fine->data[1];
  solution->Ekin0z = y_tmp * pd_fine->data[2] * pd_fine->data[2];
  solution->intEkin = t_;
  solution->U0 = 0.0;
  solution->Uf = 0.0;
  solution->Ekinfx = y_tmp * pd_fine->data[3 * (pd_fine->size[1] - 1)] *
    pd_fine->data[3 * (pd_fine->size[1] - 1)];
  solution->Ekinfy = y_tmp * pd_fine->data[3 * (pd_fine->size[1] - 1) + 1] *
    pd_fine->data[3 * (pd_fine->size[1] - 1) + 1];
  solution->Ekinfz = y_tmp * pd_fine->data[3 * (pd_fine->size[1] - 1) + 2] *
    pd_fine->data[3 * (pd_fine->size[1] - 1) + 2];
  solution->achieved_target[0] = solution->p_fine->data[3 * (solution->
    p_fine->size[1] - 1)];
  c = pd_fine->data[0];
  unusedU1 = y_tmp * c * c;
  solution->achieved_target[1] = solution->p_fine->data[3 * (solution->
    p_fine->size[1] - 1) + 1];
  c = pd_fine->data[1];
  unusedU1 += y_tmp * c * c;
  solution->achieved_target[2] = solution->p_fine->data[3 * (solution->
    p_fine->size[1] - 1) + 2];
  c = pd_fine->data[2];
  unusedU1 += y_tmp * c * c;
  solution->Ekin0 = unusedU1;
  solution->Ekinf = (y_tmp * pd_fine->data[3 * (pd_fine->size[1] - 1)] *
                     pd_fine->data[3 * (pd_fine->size[1] - 1)] + y_tmp *
                     pd_fine->data[3 * (pd_fine->size[1] - 1) + 1] *
                     pd_fine->data[3 * (pd_fine->size[1] - 1) + 1]) + y_tmp *
    pd_fine->data[3 * (pd_fine->size[1] - 1) + 2] * pd_fine->data[3 *
    (pd_fine->size[1] - 1) + 2];
  solution->T_th = params->T_th;

  /* (EXITFLAG == 1) || (EXITFLAG == 2); */
  /*  1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance. */
  /*  0 Number of iterations exceeded options.MaxIterations or number of function evaluations exceeded options.MaxFunctionEvaluations. */
  /*  -1 Stopped by an output function or plot function. */
  /*  -2 No feasible point was found. */
  /*  2 Change in x was less than options.StepTolerance (Termination tolerance on x, a scalar, the default is 1e-10) and maximum constraint violation was less than options.ConstraintTolerance. */
  /*  evaluate constraint violation  */
  /*  ineq are <= 0 */
  Fleg[0] = x->data[0];
  Fleg[1] = x->data[1];
  Fleg[2] = x->data[2];
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  Fr_l->size[1] = (int32_T)muDoubleScalarFloor(params->N_dyn - 1.0) + 1;
  emxEnsureCapacity_real_T(Fr_l, i);
  emxFree_real_T(&pd_fine);
  for (i = 0; i <= loop_ub; i++) {
    Fr_l->data[i] = x->data[(int32_T)(params->num_params + (real_T)(i + 1)) - 1];
  }

  if (d > d1) {
    i = 0;
    i1 = 0;
  } else {
    i = (int32_T)d - 1;
    i1 = (int32_T)d1;
  }

  /*  check they are column vectors */
  /*  size not known */
  solution->c->size[0] = 1;
  solution->c->size[1] = 0;

  /*  number of constraints */
  /*  already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes */
  /*  variable intergration step */
  /*  single shooting */
  state0[0] = state0_tmp;
  state0[1] = l1_tmp;
  state0[2] = l2_tmp;
  state0[3] = 0.0;
  state0[4] = 0.0;
  state0[5] = 0.0;
  nx = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  loop_ub = i1 - i;
  b_Fleg_max->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_Fleg_max, nx);
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_Fleg_max->data[i1] = x->data[i + i1];
  }

  computeRollout(state0, x->data[3] / (params->N_dyn - 1.0), params->N_dyn, Fr_l,
                 b_Fleg_max, Fleg, params->int_method, params->int_steps,
                 params->m, params->b, params->p_a1, params->p_a2, params->g,
                 params->T_th, states, solution->solution_constr.time);
  loop_ub = states->size[1];
  i = solution->solution_constr.psid->size[0] * solution->
    solution_constr.psid->size[1];
  solution->solution_constr.psid->size[0] = 1;
  solution->solution_constr.psid->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.psid, i);
  emxFree_real_T(&Fr_l);
  emxFree_real_T(&x);
  for (i = 0; i < loop_ub; i++) {
    solution->solution_constr.psid->data[i] = states->data[6 * i + 3];
  }

  loop_ub = states->size[1];
  i = solution->solution_constr.l1d->size[0] * solution->
    solution_constr.l1d->size[1];
  solution->solution_constr.l1d->size[0] = 1;
  solution->solution_constr.l1d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l1d, i);
  for (i = 0; i < loop_ub; i++) {
    solution->solution_constr.l1d->data[i] = states->data[6 * i + 4];
  }

  loop_ub = states->size[1];
  i = solution->solution_constr.l2d->size[0] * solution->
    solution_constr.l2d->size[1];
  solution->solution_constr.l2d->size[0] = 1;
  solution->solution_constr.l2d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l2d, i);
  for (i = 0; i < loop_ub; i++) {
    solution->solution_constr.l2d->data[i] = states->data[6 * i + 5];
  }

  loop_ub = states->size[1];
  i = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, i);
  for (i = 0; i < loop_ub; i++) {
    b_Fleg_max->data[i] = states->data[6 * i];
  }

  loop_ub = states->size[1];
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  for (i = 0; i < loop_ub; i++) {
    c_Fleg_max->data[i] = states->data[6 * i + 1];
  }

  loop_ub = states->size[1];
  i = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_Fleg_max, i);
  for (i = 0; i < loop_ub; i++) {
    d_Fleg_max->data[i] = states->data[6 * i + 2];
  }

  computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
    solution->solution_constr.p);

  /* only position */
  /*  I assume px py pz  are row vectors */
  /*  init struct foc C++ code generation */
  loop_ub = states->size[1];
  i = solution->solution_constr.psi->size[0] * solution->
    solution_constr.psi->size[1];
  solution->solution_constr.psi->size[0] = 1;
  solution->solution_constr.psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.psi, i);
  emxFree_real_T(&d_Fleg_max);
  emxFree_real_T(&b_Fleg_max);
  for (i = 0; i < loop_ub; i++) {
    solution->solution_constr.psi->data[i] = states->data[6 * i];
  }

  loop_ub = states->size[1];
  i = solution->solution_constr.l1->size[0] * solution->solution_constr.l1->
    size[1];
  solution->solution_constr.l1->size[0] = 1;
  solution->solution_constr.l1->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l1, i);
  for (i = 0; i < loop_ub; i++) {
    solution->solution_constr.l1->data[i] = states->data[6 * i + 1];
  }

  loop_ub = states->size[1];
  i = solution->solution_constr.l2->size[0] * solution->solution_constr.l2->
    size[1];
  solution->solution_constr.l2->size[0] = 1;
  solution->solution_constr.l2->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l2, i);
  for (i = 0; i < loop_ub; i++) {
    solution->solution_constr.l2->data[i] = states->data[6 * i + 2];
  }

  emxFree_real_T(&states);
  this_tunableEnvironment_f2[0] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1)] - pf[0];
  this_tunableEnvironment_f2[1] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1) + 1] - pf[1];
  this_tunableEnvironment_f2[2] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1) + 2] - pf[2];
  solution->solution_constr.final_error_discrete = b_norm
    (this_tunableEnvironment_f2);

  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0  */
  if (params->obstacle_avoidance) {
    /* [0; 3;-7.5]; */
    /* px > sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2); */
    /* -px + sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2)<0 */
    /*  better implementaiton with complex numbers for code generation */
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      t_ = solution->solution_constr.p->data[3 * b_i + 2] -
        params->obstacle_location[2];
      unusedU1 = solution->solution_constr.p->data[3 * b_i + 1] -
        params->obstacle_location[1];
      t_ = (2.25 - 3.0001760103259394 * (t_ * t_)) - unusedU1 * unusedU1;

      /* %%add ineq only if inside sphere */
      if (t_ > 0.0) {
        i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
        c_Fleg_max->size[0] = 1;
        loop_ub = solution->c->size[1];
        c_Fleg_max->size[1] = solution->c->size[1] + 1;
        emxEnsureCapacity_real_T(c_Fleg_max, i1);
        for (i1 = 0; i1 < loop_ub; i1++) {
          c_Fleg_max->data[i1] = solution->c->data[i1];
        }

        c_Fleg_max->data[solution->c->size[1]] = ((-solution->
          solution_constr.p->data[3 * b_i] + params->obstacle_location[0]) +
          muDoubleScalarSqrt(t_)) + params->jump_clearance;
        i1 = solution->c->size[0] * solution->c->size[1];
        solution->c->size[0] = 1;
        solution->c->size[1] = c_Fleg_max->size[1];
        emxEnsureCapacity_real_T(solution->c, i1);
        loop_ub = c_Fleg_max->size[0] * c_Fleg_max->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          solution->c->data[i1] = c_Fleg_max->data[i1];
        }
      } else {
        i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
        c_Fleg_max->size[0] = 1;
        loop_ub = solution->c->size[1];
        c_Fleg_max->size[1] = solution->c->size[1] + 1;
        emxEnsureCapacity_real_T(c_Fleg_max, i1);
        for (i1 = 0; i1 < loop_ub; i1++) {
          c_Fleg_max->data[i1] = solution->c->data[i1];
        }

        c_Fleg_max->data[solution->c->size[1]] = -solution->
          solution_constr.p->data[3 * b_i];
        i1 = solution->c->size[0] * solution->c->size[1];
        solution->c->size[0] = 1;
        solution->c->size[1] = c_Fleg_max->size[1];
        emxEnsureCapacity_real_T(solution->c, i1);
        loop_ub = c_Fleg_max->size[0] * c_Fleg_max->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          solution->c->data[i1] = c_Fleg_max->data[i1];
        }
      }

      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      i1 = c_Fleg_max->size[0] * c_Fleg_max->size[1];
      c_Fleg_max->size[0] = 1;
      loop_ub = solution->c->size[1];
      c_Fleg_max->size[1] = solution->c->size[1] + 1;
      emxEnsureCapacity_real_T(c_Fleg_max, i1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        c_Fleg_max->data[i1] = solution->c->data[i1];
      }

      c_Fleg_max->data[solution->c->size[1]] = -solution->
        solution_constr.p->data[3 * b_i];
      i1 = solution->c->size[0] * solution->c->size[1];
      solution->c->size[0] = 1;
      solution->c->size[1] = c_Fleg_max->size[1];
      emxEnsureCapacity_real_T(solution->c, i1);
      loop_ub = c_Fleg_max->size[0] * c_Fleg_max->size[1];
      for (i1 = 0; i1 < loop_ub; i1++) {
        solution->c->data[i1] = c_Fleg_max->data[i1];
      }

      /* ineq = [ineq -psi(i) ];  */
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  }

  /*  % % debug */
  /*  disp('after wall') */
  /*  length(ineq) */
  /*  2- N_dyn constraints on retraction force   -Fr_max < Fr < 0  */
  /*  unilaterality */
  /*  debug */
  /*  disp('after Fr') */
  /*  length(ineq) */
  /*  constraints on impulse force */
  /*  compute components */
  t_ = params->contact_normal[1] * 0.0 - params->contact_normal[2];
  unusedU1 = params->contact_normal[2] * 0.0 - params->contact_normal[0] * 0.0;
  n_samples = params->contact_normal[0] - params->contact_normal[1] * 0.0;
  y_tmp = (params->contact_normal[0] * Fleg[0] + params->contact_normal[1] *
           Fleg[1]) + params->contact_normal[2] * Fleg[2];
  c = ((unusedU1 * params->contact_normal[2] - n_samples *
        params->contact_normal[1]) * Fleg[0] + (n_samples *
        params->contact_normal[0] - t_ * params->contact_normal[2]) * Fleg[1]) +
    (t_ * params->contact_normal[1] - unusedU1 * params->contact_normal[0]) *
    Fleg[2];
  t_ = params->contact_normal[1] - params->contact_normal[2] * 0.0;
  unusedU1 = params->contact_normal[2] * 0.0 - params->contact_normal[0];
  n_samples = params->contact_normal[0] * 0.0 - params->contact_normal[1] * 0.0;
  t_ = ((unusedU1 * params->contact_normal[2] - n_samples *
         params->contact_normal[1]) * Fleg[0] + (n_samples *
         params->contact_normal[0] - t_ * params->contact_normal[2]) * Fleg[1])
    + (t_ * params->contact_normal[1] - unusedU1 * params->contact_normal[0]) *
    Fleg[2];

  /* 3 ------------------------------ Fleg constraints */
  /*  unilateral */
  i = solution->c->size[1];
  i1 = solution->c->size[0] * solution->c->size[1];
  solution->c->size[1]++;
  emxEnsureCapacity_real_T(solution->c, i1);
  solution->c->data[i] = -y_tmp;

  /* (Fun >fmin )  */
  /* max force  */
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->c->size[1] + 1;
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  loop_ub = solution->c->size[1];
  for (i = 0; i < loop_ub; i++) {
    c_Fleg_max->data[i] = solution->c->data[i];
  }

  c_Fleg_max->data[solution->c->size[1]] = b_norm(Fleg) - Fleg_max;
  i = solution->c->size[0] * solution->c->size[1];
  solution->c->size[0] = 1;
  solution->c->size[1] = c_Fleg_max->size[1];
  emxEnsureCapacity_real_T(solution->c, i);
  loop_ub = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  for (i = 0; i < loop_ub; i++) {
    solution->c->data[i] = c_Fleg_max->data[i];
  }

  /* (Fun < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
    c_Fleg_max->size[0] = 1;
    c_Fleg_max->size[1] = solution->c->size[1] + 1;
    emxEnsureCapacity_real_T(c_Fleg_max, i);
    loop_ub = solution->c->size[1];
    for (i = 0; i < loop_ub; i++) {
      c_Fleg_max->data[i] = solution->c->data[i];
    }

    c_Fleg_max->data[solution->c->size[1]] = muDoubleScalarSqrt(c * c + t_ * t_)
      - mu * y_tmp;
    i = solution->c->size[0] * solution->c->size[1];
    solution->c->size[0] = 1;
    solution->c->size[1] = c_Fleg_max->size[1];
    emxEnsureCapacity_real_T(solution->c, i);
    loop_ub = c_Fleg_max->size[0] * c_Fleg_max->size[1];
    for (i = 0; i < loop_ub; i++) {
      solution->c->data[i] = c_Fleg_max->data[i];
    }

    /* friction constraints */
  }

  /*   */
  /*  % debug */
  /*  disp('after Fu') */
  /*  length(ineq) */
  /*  final point  variable slack   */
  /* ineq= [ineq norm(p_f - pf) - x(num_params+N+N_dyn+1)]; */
  /*  4- initial final point  fixed slack  */
  /* *norm(p0 - pf);  */
  this_tunableEnvironment_f2[0] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1)] - pf[0];
  this_tunableEnvironment_f2[1] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1) + 1] - pf[1];
  this_tunableEnvironment_f2[2] = solution->solution_constr.p->data[3 *
    (solution->solution_constr.p->size[1] - 1) + 2] - pf[2];
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->c->size[1] + 1;
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  loop_ub = solution->c->size[1];
  for (i = 0; i < loop_ub; i++) {
    c_Fleg_max->data[i] = solution->c->data[i];
  }

  c_Fleg_max->data[solution->c->size[1]] = b_norm(this_tunableEnvironment_f2) -
    0.02;
  i = solution->c->size[0] * solution->c->size[1];
  solution->c->size[0] = 1;
  solution->c->size[1] = c_Fleg_max->size[1];
  emxEnsureCapacity_real_T(solution->c, i);
  loop_ub = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  for (i = 0; i < loop_ub; i++) {
    solution->c->data[i] = c_Fleg_max->data[i];
  }

  /* 5 - jump clearance */
  if (!params->obstacle_avoidance) {
    i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
    c_Fleg_max->size[0] = 1;
    c_Fleg_max->size[1] = solution->c->size[1] + 1;
    emxEnsureCapacity_real_T(c_Fleg_max, i);
    loop_ub = solution->c->size[1];
    for (i = 0; i < loop_ub; i++) {
      c_Fleg_max->data[i] = solution->c->data[i];
    }

    c_Fleg_max->data[solution->c->size[1]] = -solution->solution_constr.p->data
      [3 * ((int32_T)(params->N_dyn / 2.0) - 1)] + params->jump_clearance;
    i = solution->c->size[0] * solution->c->size[1];
    solution->c->size[0] = 1;
    solution->c->size[1] = c_Fleg_max->size[1];
    emxEnsureCapacity_real_T(solution->c, i);
    loop_ub = c_Fleg_max->size[0] * c_Fleg_max->size[1];
    for (i = 0; i < loop_ub; i++) {
      solution->c->data[i] = c_Fleg_max->data[i];
    }
  }

  emxFree_real_T(&c_Fleg_max);

  /*  if any(isinf(ineq)) */
  /*      disp('Infn in constraint') */
  /*      find(isinf(ineq))  */
  /*      isinf(ineq) */
  /*  end */
  /*  if any(isnan(ineq)) */
  /*      disp('Nan in constraint') */
  /*      find(isnan(ineq)) */
  /*      isnan(ineq) */
  /*  end */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp.c) */
