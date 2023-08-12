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
  /*
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
  */

  double theta_1, theta_2, theta_3, L_1, L_2, L_3;
  // For now, use hardcoded lengths. Only one finger will be tested
  /*
    L_1 = fin_length[0];
    L_2 = fin_length[1];
    L_3 = fin_length[2];
  */

  L_1 = 38.88 / 1000; //m
  L_2 = 25.85 / 1000;
  L_3 = 31.84 / 1000;

  double l_1 = 0.5 * L_1;
  double l_2 = 0.5 * L_2;
  double l_3 = 0.5 * L_3;

  theta_1 = state_pos[0];
  theta_2 = state_pos[1];
  theta_3 = state_pos[2];

  double m_1 = 7.5 / 1000; //kg
  double m_2 = 4.6 / 1000;
  double m_3 = 4.4 / 1000;

  double d_1 = 18.94 / 1000; //m
  double d_2 = 16.55 / 1000;
  double d_3 = 15.80 / 1000;

  double g = 9.81;//m/s^2
  double I_1 = (1 / 4) * (pow(m_1 * (d_1 / 2), 2)) + (m_1 * (pow(L_1, 2))) / 3;
  double I_2 = (1 / 4) * (pow(m_2 * (d_2 / 2), 2)) + (m_2 * (pow(L_2, 2))) / 3;
  double I_3 = (1 / 4) * (pow(m_3 * (d_3 / 2), 2)) + (m_3 * (pow(L_3, 2))) / 3;

  /*
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
  */

  double m11 = m_1 * pow(l_1, 2) + m_2 * (pow(L_1, 2) + pow(l_2, 2) + 2 * L_1 * l_1 * std::cos(theta_2)) + m_3 *
               (pow(L_1, 2) + pow(L_2, 2) + pow(l_3, 2) + 2 * L_1 * L_2 * std::cos(theta_2) + 2 * L_2 * l_3 * std::cos(theta_3)
                + l_3 * std::cos(theta_2 + theta_3)) + I_1 + I_2 + I_3;
  double m12 = m_2 * (pow(l_2, 2) + L_1 * pow(l_2, 2) * std::cos(theta_2)) + m_3 * (pow(L_2, 2) + pow(l_3, 2) +
               2 * L_2 * l_3 * std::cos(theta_3) + L_1 * L_2 * std::cos(theta_2) + L_1 * l_3 * std::cos(theta_2 + theta_3))
               + I_2 + I_3;
  double m13 = m_3 * (pow(l_3, 2) + L_1 * l_3 * std::cos(theta_2 + theta_3)) + L_2 * l_3 * std::cos(theta_3) + I_3;
  double m21 = m12;
  double m22 = m_3 * (pow(L_2, 2) * pow(l_3, 2) + 2 * L_2 * l_3 * std::cos(theta_3)) + m_2 * pow(l_2, 2) + I_2 + I_3;
  double m23 = m_3 * (pow(l_3, 2) + L_2 * l_3 * std::cos(theta_3)) + I_3;
  double m31 = m13;
  double m32 = m23;
  double m33 = m_3 * pow(l_3, 2) + I_3;

  M[0] = m11;
  M[1] = m12;
  M[2] = m13;
  M[3] = m21;
  M[4] = m22;
  M[5] = m23;
  M[6] = m31;
  M[7] = m32;
  M[8] = m33;

  double h1 = L_1 * l_2 * std::sin(theta_2);
  double h2 = L_1 * l_3 * std::sin(theta_2 + theta_3);
  double h3 = L_2 * l_3 * std::sin(theta_3);
  double h4 = L_1 * l_2 * std::sin(theta_2);

  double c_111 = 0;
  double c_221 = -m_2 * h1 - m_3 * (h2 + h4);
  double c_331 = -m_3 * (h2 + h3);
  double c_112 = m_2 * h1 + m_3 * (h2 + h4);
  double c_222 = 0;
  double c_332 = -m_3 * h3;
  double c_113 = m_3 * (h2 + h3);
  double c_223 = m_3 * h3;
  double c_333 = 0;

  C[0] = c_111;
  C[1] = c_221;
  C[2] = c_331;
  C[3] = c_112;
  C[4] = c_222;
  C[5] = c_332;
  C[6] = c_113;
  C[7] = c_223;
  C[8] = c_333;

  /*
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
  */

  double c_211 = -m_2 * h1 - m_3 * (h2 + h4);
  double c_121 = c_211;
  double c_311 = -m_3 * (h2 + h3);
  double c_131 = c_311;
  double c_321 = c_311;
  double c_231 = c_131;
  double c_212 = 0;
  double c_122 = c_212;
  double c_312 = c_332;
  double c_132 = c_332;
  double c_322 = c_332;
  double c_232 = c_332;
  double c_213 = c_223;
  double c_123 = c_223;
  double c_313 = 0;
  double c_133 = 0;
  double c_323 = 0;
  double c_233 = 0;


  B[0] = (c_211 + c_121);
  B[1] = (c_311 + c_131);
  B[2] = (c_321 + c_231);
  B[3] = (c_212 + c_122);
  B[4] = (c_312 + c_132);
  B[5] = (c_322 + c_232);
  B[6] = (c_213 + c_123);
  B[7] = (c_313 + c_133);
  B[8] = (c_323 + c_233);

  /*

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
  */

  G[0] = (m_1 * l_1 + L_1 * (m_2 + m_3)) * g * std::cos(theta_1) + (m_2 * l_2 + m_3 * L_3) * g * std::cos(theta_1 + theta_2) +
         m_3 * g * l_3 * std::cos(theta_1 + theta_2 + theta_3);

  G[1] = (m_2 * l_2 + m_3 * L_3) * g * std::cos(theta_1 + theta_2) + m_3 * g * l_3 * std::cos(theta_1 + theta_2 + theta_3);
  G[2] = m_3 * g * l_3 * std::cos(theta_1 + theta_2 + theta_3);

  /*
        //  B_1 = 2*[c_211 c_311 c_321; c_212 c_312 c_322; c_213 c_313 c_323];
    m13 = state_pos[0] + state_pos[1];
    m23 = (0.5 * l_2 + 0.4 * fin_length[2]) * 9.81 * std::cos(m13);
    m13 = 3.9240000000000004 * l_3 * std::cos(m13 + state_pos[2]);
    G[0] = ((0.7 * l_1 + fin_length[0] * 0.9) * 9.81 * std::cos(state_pos[0]) +
            m23) + m13;
    G[1] = m23 + m13;
    G[2] = m13;
  */


}

//
// File trailer for dynamics.cpp
//
// [EOF]
//
