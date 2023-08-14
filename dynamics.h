//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: dynamics.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 06-Jun-2023 14:46:19
//
#ifndef DYNAMICS_H
#define DYNAMICS_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "dynamics_types.h"

// Function Declarations
extern void dynamics(double state_pos[3],
                     double M[9], double C[9], double B[9], double G[3]);
/*
extern void dynamics(double state_pos[3], double fin_length[3],
                     double M[9], double C[9], double B[9], double G[3]); */

#endif

//
// File trailer for dynamics.h
//
// [EOF]
//
