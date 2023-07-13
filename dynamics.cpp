//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: dynamics.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 06-Jun-2023 14:46:19
//

// Include Files
#include "dynamics.h"
#include <cmath>

// Function Definitions

//
// DYNAMICS returns the dynamic parameters of each finger. One finger is modelled
//  as a plannar 3 link RRR manipulator
//  the current states: state_pos
//  the finger parameters: fing_length. For a start, m_i and I_i are
//  specified in this function.
// Arguments    : const double state_pos[3]
//                const double fin_length[3]
//                double M[9]
//                double C[9]
//                double B[9]
//                double G[3]
// Return Type  : void
//
void dynamics(double state_pos[3], double fin_length[3], double M[9],
              double C[9], double B[9], double G[3])
{
  double l_1;
  double l_2;
  double l_3;
  double m11_tmp;
  double m11_tmp_tmp;
  double c_332;
  double m13;
  double c_223;
  double m12_tmp;
  double b_m12_tmp;
  double c_m12_tmp;
  double d_m12_tmp;
  double m12;
  double m13_tmp;
  double m23;
  double M_tmp;
  l_1 = 0.5 * fin_length[0];
  l_2 = 0.5 * fin_length[1];
  l_3 = 0.5 * fin_length[2];
  m11_tmp = std::cos(state_pos[1]);
  m11_tmp_tmp = state_pos[1] + state_pos[2];
  c_332 = std::cos(m11_tmp_tmp);
  m13 = std::cos(state_pos[2]);
  c_223 = l_2 * l_2;
  m12_tmp = l_3 * l_3;
  b_m12_tmp = fin_length[1] * fin_length[1];
  c_m12_tmp = 2.0 * fin_length[1] * l_3 * m13;
  d_m12_tmp = fin_length[0] * l_3;
  m12 = ((0.5 * (c_223 + fin_length[0] * c_223 * m11_tmp) + 0.4 * ((((b_m12_tmp
              + m12_tmp) + c_m12_tmp) + fin_length[0] * fin_length[1] * m11_tmp)
           + d_m12_tmp * c_332)) + 0.5) + 0.5;
  m13_tmp = fin_length[1] * l_3;
  m13 = (0.4 * (m12_tmp + fin_length[0] * l_3 * std::cos(state_pos[1] +
           state_pos[2])) + m13_tmp * m13) + 0.5;
  m23 = 0.4 * (m12_tmp + fin_length[1] * l_3 * std::cos(state_pos[2])) + 0.5;
  M_tmp = fin_length[0] * fin_length[0];
  M[0] = ((((0.7 * (l_1 * l_1) + 0.5 * ((M_tmp + c_223) + 2.0 * fin_length[0] *
              l_1 * m11_tmp)) + 0.4 * (((((M_tmp + b_m12_tmp) + m12_tmp) + 2.0 *
    fin_length[0] * fin_length[1] * m11_tmp) + c_m12_tmp) + l_3 * c_332)) + 0.5)
          + 0.5) + 0.5;
  M[3] = m12;
  M[6] = m13;
  M[1] = m12;
  M[4] = ((0.4 * (b_m12_tmp * m12_tmp + c_m12_tmp) + 0.5 * c_223) + 0.5) + 0.5;
  M[7] = m23;
  M[2] = m13;
  M[5] = m23;
  M[8] = 0.4 * m12_tmp + 0.5;
  m12_tmp = fin_length[0] * l_2 * std::sin(state_pos[1]);
  m23 = d_m12_tmp * std::sin(m11_tmp_tmp);
  m13 = m13_tmp * std::sin(state_pos[2]);
  c_332 = -0.4 * m13;
  c_223 = 0.4 * m13;
  C[0] = 0.0;
  M_tmp = 0.4 * (m23 + m12_tmp);
  m11_tmp = -0.5 * m12_tmp - M_tmp;
  C[3] = m11_tmp;
  m23 += m13;
  m13 = -0.4 * m23;
  C[6] = m13;
  C[1] = 0.5 * m12_tmp + M_tmp;
  C[4] = 0.0;
  C[7] = c_332;
  C[2] = 0.4 * m23;
  C[5] = c_223;
  C[8] = 0.0;
  B[0] = m11_tmp + m11_tmp;
  m23 = m13 + m13;
  B[3] = m23;
  B[6] = m23;
  B[1] = 0.0;
  m23 = c_332 + c_332;
  B[4] = m23;
  B[7] = m23;
  B[2] = c_223 + c_223;
  B[5] = 0.0;
  B[8] = 0.0;

  //  B_1 = 2*[c_211 c_311 c_321; c_212 c_312 c_322; c_213 c_313 c_323];
  m13 = state_pos[0] + state_pos[1];
  m23 = (0.5 * l_2 + 0.4 * fin_length[2]) * 9.81 * std::cos(m13);
  m13 = 3.9240000000000004 * l_3 * std::cos(m13 + state_pos[2]);
  G[0] = ((0.7 * l_1 + fin_length[0] * 0.9) * 9.81 * std::cos(state_pos[0]) +
          m23) + m13;
  G[1] = m23 + m13;
  G[2] = m13;
}

//
// File trailer for dynamics.cpp
//
// [EOF]
//
