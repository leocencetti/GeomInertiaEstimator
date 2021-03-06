//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sign.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 26-Feb-2019 22:17:41
//

// Include Files
#include "rt_nonfinite.h"
#include "calc_EKF_F_optimized.h"
#include "sign.h"

// Function Definitions

//
// Arguments    : double *x
// Return Type  : void
//
void b_sign(double *x)
{
  if (*x < 0.0) {
    *x = -1.0;
  } else if (*x > 0.0) {
    *x = 1.0;
  } else {
    if (*x == 0.0) {
      *x = 0.0;
    }
  }
}

//
// File trailer for sign.cpp
//
// [EOF]
//
