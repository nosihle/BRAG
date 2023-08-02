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



template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order + 1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order + 1]; // Raw values
    float y[order + 1]; // Filtered values

  public:
    LowPass(float f0, float fs, bool adaptive) {
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.

      omega0 = 6.28318530718 * f0;
      dt = 1.0 / fs;
      adapt = adaptive;
      tn1 = -dt;
      for (int k = 0; k < order + 1; k++) {
        x[k] = 0;
        y[k] = 0;
      }
      setCoef();
    }

    void setCoef() {
      if (adapt) {
        float t = micros() / 1.0e6;
        dt = t - tn1;
        tn1 = t;
      }

      float alpha = omega0 * dt;
      if (order == 1) {
        a[0] = -(alpha - 2.0) / (alpha + 2.0);
        b[0] = alpha / (alpha + 2.0);
        b[1] = alpha / (alpha + 2.0);
      }
      if (order == 2) {
        float alphaSq = alpha * alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
        b[0] = alphaSq / D;
        b[1] = 2 * b[0];
        b[2] = b[0];
        a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
        a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
      }
    }

    float filt(float xn) {
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if (adapt) {
        setCoef(); // Update coefficients if necessary
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for (int k = 0; k < order; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
      }
      y[0] += b[order] * x[order];

      // Save the historical values
      for (int k = order; k > 0; k--) {
        y[k] = y[k - 1];
        x[k] = x[k - 1];
      }

      // Return the filtered value
      return y[0];
    }
};
#endif
