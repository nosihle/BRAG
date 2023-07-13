//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: finger_pos.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 06-Jun-2023 14:35:01
//

// Include Files
#include "finger_pos.h"
#include <cmath>

// Function Definitions

//
// Finger_pos coordinates of finger's position in world frame
//    state_pos: [q1; q2; q3]; this is in radians
//    l1: length of link 1;
//    l2: length of link 2;
//    l3: length of link 3;
//    finger: [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4];
// Arguments    : const double state_pos[3]
//                const double fin_length[3]
//                double finger[9]
// Return Type  : void
//
void finger_pos(double state_pos[3], double fin_length[3], double
                finger[9])
{
  double R_01_tmp;
  double b_R_01_tmp;
  double H_01[16];
  double R_12_tmp[16];
  double b_fin_length[4];
  int i;
  double d;
  int i1;
  double b_H_01[4];
  int i2;
  double H_02[16];
  double b_H_02[4];
  double c_H_02[4];
  double d1;
  R_01_tmp = std::sin(state_pos[0]);
  b_R_01_tmp = std::cos(state_pos[0]);
  H_01[0] = b_R_01_tmp;
  H_01[4] = -R_01_tmp;
  H_01[8] = 0.0;
  H_01[1] = R_01_tmp;
  H_01[5] = b_R_01_tmp;
  H_01[9] = 0.0;
  H_01[2] = 0.0;
  H_01[12] = 0.0;
  H_01[6] = 0.0;
  H_01[13] = 0.0;
  H_01[10] = 1.0;
  H_01[14] = 0.0;
  H_01[3] = 0.0;
  H_01[7] = 0.0;
  H_01[11] = 0.0;
  H_01[15] = 1.0;
  R_01_tmp = std::sin(state_pos[1]);
  b_R_01_tmp = std::cos(state_pos[1]);
  R_12_tmp[0] = b_R_01_tmp;
  R_12_tmp[4] = -R_01_tmp;
  R_12_tmp[8] = 0.0;
  R_12_tmp[1] = R_01_tmp;
  R_12_tmp[5] = b_R_01_tmp;
  R_12_tmp[9] = 0.0;
  R_12_tmp[2] = 0.0;
  R_12_tmp[6] = 0.0;
  R_12_tmp[10] = 1.0;
  R_12_tmp[12] = fin_length[0];
  R_12_tmp[13] = 0.0;
  R_12_tmp[14] = 0.0;
  R_12_tmp[3] = 0.0;
  R_12_tmp[7] = 0.0;
  R_12_tmp[11] = 0.0;
  R_12_tmp[15] = 1.0;
  R_01_tmp = std::sin(state_pos[2]);
  b_R_01_tmp = std::cos(state_pos[2]);
  b_fin_length[0] = fin_length[0];
  b_fin_length[1] = 0.0;
  b_fin_length[2] = 0.0;
  b_fin_length[3] = 1.0;
  for (i = 0; i < 4; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      int H_02_tmp;
      i2 = i1 << 2;
      H_02_tmp = i + i2;
      H_02[H_02_tmp] = ((H_01[i] * R_12_tmp[i2] + H_01[i + 4] * R_12_tmp[i2 + 1])
                        + H_01[i + 8] * R_12_tmp[i2 + 2]) + H_01[i + 12] *
        R_12_tmp[i2 + 3];
      d += H_01[H_02_tmp] * b_fin_length[i1];
    }

    b_H_01[i] = d;
  }

  H_01[0] = b_R_01_tmp;
  H_01[4] = -R_01_tmp;
  H_01[8] = 0.0;
  H_01[1] = R_01_tmp;
  H_01[5] = b_R_01_tmp;
  H_01[9] = 0.0;
  H_01[2] = 0.0;
  H_01[6] = 0.0;
  H_01[10] = 1.0;
  H_01[12] = fin_length[1];
  H_01[13] = 0.0;
  H_01[14] = 0.0;
  H_01[3] = 0.0;
  H_01[7] = 0.0;
  H_01[11] = 0.0;
  H_01[15] = 1.0;
  b_fin_length[0] = fin_length[2];
  b_fin_length[1] = 0.0;
  b_fin_length[2] = 0.0;
  b_fin_length[3] = 1.0;
  for (i = 0; i < 4; i++) {
    d = H_02[i + 4];
    R_01_tmp = H_02[i + 8];
    b_R_01_tmp = H_02[i + 12];
    b_H_02[i] = ((H_02[i] * fin_length[1] + d * 0.0) + R_01_tmp * 0.0) +
      b_R_01_tmp;
    d1 = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      d1 += (((H_02[i] * H_01[i2] + d * H_01[i2 + 1]) + R_01_tmp * H_01[i2 + 2])
             + b_R_01_tmp * H_01[i2 + 3]) * b_fin_length[i1];
    }

    c_H_02[i] = d1;
  }

  finger[0] = b_H_01[0];
  finger[3] = b_H_02[0];
  finger[6] = c_H_02[0];
  finger[1] = b_H_01[1];
  finger[4] = b_H_02[1];
  finger[7] = c_H_02[1];
  finger[2] = b_H_01[2];
  finger[5] = b_H_02[2];
  finger[8] = c_H_02[2];
}

//
// File trailer for finger_pos.cpp
//
// [EOF]
//
