#ifndef AWARD_FUNCTIONS_H
#define AWARD_FUNCTIONS_H

#include <Encoder.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_LSM6DS33.h>
#include <arduino.h>

#include <TMM_vector.h>
#include <TMM_enable_if.h>
#include <TMM_matrix.h>
#include <TMM_squarematrix.h>

typedef struct IMUData IMUData_t;
typedef struct EncderCounts EncderCounts_t;
typedef struct encRevs encRevs_t;
typedef struct FsrScpData FsrScpData_t;
typedef struct AnglesComps AnglesComps_t;
typedef struct AnglesDerivatives AnglesDerivatives_t;
typedef struct jntAngl jntAngl_t;
typedef struct fingAngles fingAngles_t;
typedef struct EulerAngIMU EulerAngIMU_t;
typedef struct coords coords_t;
typedef struct trq trq_t;
typedef struct state state_t;
typedef struct polyCoef polyCoef_t;

struct IMUData {
  double aX0, aY0, aZ0, gX0, gY0, gZ0;

  // Constructor to initialize all members to 0.0f
  IMUData() : aX0(0.0), aY0(0.0), aZ0(0.0), gX0(0.0), gY0(0.0), gZ0(0.0) {}

};

struct EncderCounts {
  volatile long m1, m2, m3, m4, m5, m6;

  // Constructor to initialize all members to 0.0f
  EncderCounts() : m1(0.0f), m2(0.0f), m3(0.0f), m4(0.0f), m5(0.0f), m6(0.0f) {}

};

struct encRevs {
  double m1, m2, m3, m4, m5, m6;

  encRevs() : m1(0.0), m2(0.0), m3(0.0), m4(0.0), m5(0.0), m6(0.0) {}

};

struct FsrScpData {
  double amps1, amps2, amps3, amps4, amps5, amps6,
         scp1, scp2, scp3, scp4, fsr1, fsr2, fsr3, fsr4;

  FsrScpData() : amps1(0.0), amps2(0.0), amps3(0.0), amps4(0.0), amps5(0.0),
    amps6(0.0), scp1(0.0), scp2(0.0), scp3(0.0), scp4(0.0), fsr1(0.0), fsr2(0.0), fsr3(0.0), fsr4(0.0) {}
};

struct AnglesComps {
  double aX, aY, aZ, gX, gY, gZ, d_time;

  AnglesComps() : aX(0.0), aY(0.0), aZ(0.0), gX(0.0), gY(0.0), gZ(0.0) {}

};

struct AnglesDerivatives {
  double rVel, pVel, yVel, rAcc, pAcc, yAcc;

  AnglesDerivatives() : rVel(0.0), pVel(0.0), yVel(0.0), rAcc(0.0), pAcc(0.0), yAcc(0.0) {}

};

struct jntAngl {
  double theta_1, theta_2, theta_3;

  jntAngl() : theta_1(0.0), theta_2(0.0), theta_3(0.0) {}

};

struct fingAngles {
  double a1, a2, a3;
  fingAngles() : a1(0.0), a2(0.0), a3(0.0) {}

};

struct EulerAngIMU {
  double roll, pitch;
  EulerAngIMU() : roll(0.0), pitch(0.0) {}

};

struct coords {
  double x, y, z;
  coords() : x(0.0), y(0.0), z(0.0) {}

};

struct trq {
  double tau_1, tau_2, tau_3;
  trq() : tau_1(0.0), tau_2(0.0), tau_3(0.0) {}

};

struct state {
  coords pos, vel, acc;
};

struct polyCoef {
  double a0, a1, a2, a3, a4, a5;

  polyCoef() : a0(0.0), a1(0.0), a2(0.0), a3(0.0), a4(0.0), a5(0.0) {}

};

void getSensorData(FsrScpData_t& );
void calibrateIMUs(Adafruit_LSM6DS33 lsm6ds33, int , sensors_event_t& , sensors_event_t& , sensors_event_t& , AnglesComps_t& );
void readIMUData(Adafruit_LSM6DS33 lsm6ds33, int , sensors_event_t& , sensors_event_t& , sensors_event_t& , AnglesComps_t& );
void switchMux(int );
void FWD_Motors(int MJ_BIN[], double);
void RVS_Motors(int MJ_BIN[], double);
void FWD_MotorsV2(int MJ_BIN[], double);
void RVS_MotorsV2(int MJ_BIN[], double);
void STOP_Motors(int MJ_BIN[]);
void BRK_Motors(int MJ_BIN[]);
float TsaDisp(float, float, float);
int sgn(float );
float dispToRots(float, float, float);
float RotsToDisp(float, float, float);
float Voltage2Force(int );
void SineTraj(state_t&, unsigned );
void CubicTrajCoeff (polyCoef_t& , unsigned , unsigned , float , float , float , float );
void QuinticTrajCoeff (polyCoef_t& , unsigned , unsigned , float , float , float , float , float , float );
void CubicTraj (state_t& , polyCoef_t& , double );
void QuinticTraj (state_t& , polyCoef_t& , unsigned );
float rotSpeed (float, float , float, float );
void computeTendonLengths (state_t& tendon_len, state_t& desired_theta);
void computeTendonVelocity (state_t& tendon_vel, state_t& desired_theta);
#endif
