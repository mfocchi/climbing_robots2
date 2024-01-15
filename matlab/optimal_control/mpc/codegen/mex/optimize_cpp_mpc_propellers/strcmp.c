/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * strcmp.c
 *
 * Code generation for function 'strcmp'
 *
 */

/* Include files */
#include "strcmp.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
boolean_T b_strcmp(const char_T a[3])
{
  static const char_T b[3] = { 'r', 'k', '4' };

  return memcmp(&a[0], &b[0], 3) == 0;
}

/* End of code generation (strcmp.c) */
