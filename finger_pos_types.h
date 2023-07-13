//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: finger_pos_types.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 06-Jun-2023 14:35:01
//
#ifndef FINGER_POS_TYPES_H
#define FINGER_POS_TYPES_H

// Include Files
#include "rtwtypes.h"
#ifdef FINGER_POS_XIL_BUILD
#if defined(_MSC_VER) || defined(__LCC__)
#define FINGER_POS_DLL_EXPORT          __declspec(dllimport)
#else
#define FINGER_POS_DLL_EXPORT
#endif

#elif defined(BUILDING_FINGER_POS)
#if defined(_MSC_VER) || defined(__LCC__)
#define FINGER_POS_DLL_EXPORT          __declspec(dllexport)
#else
#define FINGER_POS_DLL_EXPORT          __attribute__ ((visibility("default")))
#endif

#else
#define FINGER_POS_DLL_EXPORT
#endif
#endif

//
// File trailer for finger_pos_types.h
//
// [EOF]
//
