#ifndef AWARD_FUNCTIONS_H
#define AWARD_FUNCTIONS_H

#include <Encoder.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_LSM6DS33.h>
#include <MadgwickAHRS.h>
#include <arduino.h>

struct IMUData {
  float aX0, aY0, aZ0, gX0, gY0, gZ0;
};

struct FsrScpData {
  long amps1, amps2, amps3, amps4, amps5, amps6, scp1, scp2, scp3, scp4, fsr1, fsr2, fsr3, fsr4;
};

struct AnglesComps {
  float aX, aY, aZ, gX, gY, gZ, d_time;
};

struct EulerAngIMU {
  float roll, pitch;
};

void getSensorData(FsrScpData& data);
void calibrateIMUs(Adafruit_LSM6DS33 lsm6ds33, int channel, sensors_event_t& acc, sensors_event_t& gyr, sensors_event_t& tmp, AnglesComps& OutBias);
//void calibrateIMUs(IMUData& data, Adafruit_LSM6DS33 lsm6ds33, sensors_event_t acc, sensors_event_t gyro, sensors_event_t temp);
//void readIMUData(int channel, sensors_event_t acc, sensors_event_t gyro, sensors_event_t temp);
void readIMUData(Adafruit_LSM6DS33 lsm6ds33, int channel, sensors_event_t& acc, sensors_event_t& gyro, sensors_event_t& temp);
void switchMux(int channel);
void FWD_Motors(int MJ_BIN[], double vel);
void RVS_Motors(int MJ_BIN[], double vel);
void STOP_Motors(int MJ_BIN[]);
void BRK_Motors(int MJ_BIN[]);
float TsaDisp(float Lt, float n, float r);
int sgn(float num);
float dispToRots(float displ, float radius, float Lt);
float RotsToDisp(float n, float radius, float Lt);
float Voltage2Force(int forceBits);
#endif
