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
typedef struct jntAngl jntAngl_t;
typedef struct fingAngles fingAngles_t;
typedef struct EulerAngIMU EulerAngIMU_t;
typedef struct coords coords_t;
typedef struct trq trq_t;
typedef struct state state_t;
typedef struct polyCoef polyCoef_t;

struct IMUData {
  float aX0, aY0, aZ0, gX0, gY0, gZ0;
  initialize() {
    aX0 = aY0 = aZ0 = gX0 = gY0 = gZ0 = 0.0f;
  }
};

struct EncderCounts {
  volatile long m1, m2, m3, m4, m5, m6;
  initialize() {
    m1 = m2 = m3 = m4 = m5 = m6 = 0;
  }
};

struct encRevs {
  float m1, m2, m3, m4, m5, m6;
  initialize() {
    m1 = m2 = m3 = m4 = m5 = m6 = 0.0f;
  }
};

struct FsrScpData {
  float amps1, amps2, amps3, amps4, amps5, amps6,
        scp1, scp2, scp3, scp4, fsr1, fsr2, fsr3, fsr4;
  initialize() {
    amps1 = amps2 = amps3
                    = amps4 = amps5 = amps6
                                      = scp1 = scp2 = scp3 =
                                            scp4 = fsr1 = fsr2 = fsr3 = fsr4 = 0.0f;
  }
};

struct AnglesComps {
  float aX, aY, aZ, gX, gY, gZ, d_time;
  initialize() {
    aX = aY = aZ = gX = gY = gZ = d_time = 0.0f;
  }
};

struct jntAngl {
  float theta_1, theta_2, theta_3;
  initialize() {
    theta_1 = theta_2 = theta_3 = 0.0f;
  }
};

struct fingAngles {
  float a1, a2, a3;
  initialize() {
    a1 = a2 = a3 = 0.0f;
  }
};

struct EulerAngIMU {
  float roll, pitch;
  initialize() {
    roll = pitch = 0.0f;
  }
};

struct coords {
  float x, y, z;
  initialize() {
    x = y = z = 0.0f;
  }
};

struct trq {
  float tau_1, tau_2, tau_3;
  initialize() {
    tau_1 = tau_2 = tau_3 = 0.0f;
  }
};

struct state {
  coords pos, vel, acc;
};

struct polyCoef {
  float a0, a1, a2, a3, a4, a5;
  initialize() {
    a1 = a2 = a3 = a4 = a5 = 0.0f;
  }
};

void getSensorData(FsrScpData_t& );
void calibrateIMUs(Adafruit_LSM6DS33 lsm6ds33, int , sensors_event_t& , sensors_event_t& , sensors_event_t& , AnglesComps_t& );
void readIMUData(Adafruit_LSM6DS33 lsm6ds33, int , sensors_event_t& , sensors_event_t& , sensors_event_t& , AnglesComps_t& );
void switchMux(int );
void FWD_Motors(int MJ_BIN[], double);
void RVS_Motors(int MJ_BIN[], double);
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
void CubicTraj (state_t& , polyCoef_t& , unsigned );
void QuinticTraj (state_t& , polyCoef_t& , unsigned );
float rotSpeed (float, float , float, float );
#endif
