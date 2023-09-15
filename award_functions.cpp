#include "award_functions.h"
#include "dynamics.h"
#include "finger_pos.h"

//Define MUX control pins and SIG_pin
const int s0 = 30, s1 = 31, s2 = 32, s3 = 34;
const int ctrl_pins[] = {s0, s1, s2, s3};
const int com_pin = 38;

// For SPI mode, we need a CS pin
// For software-SPI mode we need SCK/MOSI/MISO pins
const int LSM_SCK = 27;
const int LSM_MISO = 39;
const int LSM_MOSI = 26;

void calibrateIMUs(Adafruit_LSM6DS33 lsm6ds33, int channel, sensors_event_t& acc,
                   sensors_event_t& gyr, sensors_event_t& tmp, AnglesComps_t& OutBias) {
  int num_readings = 500;
  float aX, aY, aZ, gX, gY, gZ;

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    readIMUData(lsm6ds33, channel, acc, gyr, tmp, OutBias);

    aX += acc.acceleration.x;
    aY += acc.acceleration.y;
    aZ += acc.acceleration.z;
    gX += gyr.gyro.x;
    gY += gyr.gyro.y;
    gZ += gyr.gyro.z;

  }

  OutBias.aX = aX / num_readings;
  OutBias.aY = aY / num_readings;
  OutBias.aZ = aZ / num_readings;
  OutBias.gX = gX / num_readings;
  OutBias.gY = gY / num_readings;
  OutBias.gZ = gZ / num_readings;
  OutBias.d_time = 0;
}

void readIMUData(Adafruit_LSM6DS33 lsm6ds33, int channel,
                 sensors_event_t& acc, sensors_event_t& gyro, sensors_event_t& temp,
                 AnglesComps_t& InBias) {
  /*
     Function allows for the reading of the data (acc, gyro, temp) from an IMU object
  */

  // Select chip. This function uses the multiplexer to select a channel to ground/LOW
  switchMux(channel);
  delay(10); // perhaps this can improve not reading channels. Needs to be tested.

  if (!lsm6ds33.begin_SPI(com_pin, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    /*
       Needs to be completed. What happens if you cant read a sensor during operation?
       At the moment, we just print that unaccessible channel. When implementing this in real hardware,
       a stratefy is needed.

       Should try again? If yes, for how long? and what do you do if you cant read after that time interval?
    */
    //could not begin, so try again
    //Serial.println(channel);
    float start_timer = millis();
    float curr_time = millis();
    while ((!lsm6ds33.begin_SPI(com_pin, LSM_SCK, LSM_MISO, LSM_MOSI)) && (curr_time - start_timer < 10)) {
      lsm6ds33.begin_SPI(com_pin, LSM_SCK, LSM_MISO, LSM_MOSI);
      curr_time = millis();
    }
  }

  lsm6ds33.getEvent(&acc, &gyro, &temp); // Get data

  //include the bias values to the readings

  acc.acceleration.x = acc.acceleration.x + InBias.aX;
  acc.acceleration.y = acc.acceleration.y + InBias.aY;
  acc.acceleration.z = acc.acceleration.z + InBias.aZ;

  gyro.gyro.x = gyro.gyro.x + InBias.gX;
  gyro.gyro.y = gyro.gyro.y + InBias.gY;
  gyro.gyro.z = gyro.gyro.z + InBias.gZ;
}

void switchMux(int channel) {
  /*
     Given 4 control Pins, 16 channels can be implemented.
     This function implements the switching of the channels for MUX/Multiplexer
  */
  int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[][4] = {
    {0, 0, 0, 0}, //channel 0
    {1, 0, 0, 0}, //channel 1
    {0, 1, 0, 0}, //channel 2
    {1, 1, 0, 0}, //channel 3
    {0, 0, 1, 0}, //channel 4
    {1, 0, 1, 0}, //channel 5
    {0, 1, 1, 0}, //channel 6
    {1, 1, 1, 0}, //channel 7
    {0, 0, 0, 1}, //channel 8
    {1, 0, 0, 1} //channel 9
  };

  //loop through the 4 sig
  for (int i = 0; i < 4; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
}

void FWD_Motors(int MJ_BIN[], double vel) {
  /*
     Functions drives motors forward
     MJ_BIN is an array of the Motor Direction pins
     vel is the desired PWM of the motors
  */
  for (int i = 0; i < sizeof(MJ_BIN) / 2; i++) {
    analogWrite(MJ_BIN[2 * i], vel);
    analogWrite(MJ_BIN[2 * i + 1], 0);
  }
}

void RVS_Motors(int MJ_BIN[], double vel) {
  /*
     Drives motors in reverse.
     MJ_BIN is an array of the Motor Direction pins
     vel is the desired PWM for the motors
  */
  for (int i = 0; i < sizeof(MJ_BIN) / 2; i++) {
    analogWrite(MJ_BIN[2 * i], 0);
    analogWrite(MJ_BIN[2 * i + 1], vel);
  }
}

void STOP_Motors(int MJ_BIN[]) {
  /*
     Stops motors by removing power, leaving motors coasting
     MJ_BIN is an array of the Motor Direction pins
  */
  for (int i = 0; i < sizeof(MJ_BIN); i++) {
    analogWrite(MJ_BIN[i], 0);
  }
}

void BRK_Motors(int MJ_BIN[]) {
  /*
     Stops motors with Power, allowing for braking
     MJ_BIN is an array of the Motor Direction pins
  */
  for (int i = 0; i < sizeof(MJ_BIN); i++) {
    analogWrite(MJ_BIN[i], 256);
  }
}

float RotsToDisp(float n, float radius, float Lt) {
  /*
     Function computes the displacement of TSA given the number of rotations
     n =  number of rotations
     radius = radius of the strings used
     Lt = is the length of the fixed twisting zone (Fixed twisting zone TSA is assumed)
     public:
    float radius;
    float Lt;
    TODO: improve function so that radius and Lt are not inputs
  */
  float temp = 2 * PI * n;
  float d = sqrt(Lt * Lt + temp * radius * temp * radius) - Lt;
  return d;
}

float dispToRots(float displ, float radius, float Lt) {
  /*
     Function computes the number of rotations needed for a given distance of TSA

     public: float radius; float Lt;
    TODO: improve function such that radius and Lt are not inputs
  */
  float numRots = sqrt(displ * displ + 2 * displ * Lt) / (2 * PI * radius);
  return numRots; 
}

float rotSpeed (float linSpeed, float Lt, float n, float radius) {
  /*
     This function computes the rotational speed in rad/s, given that Lt,
     radius are in the same units (cm / m) and linSpeed is also in corresponding units
     cm/s or m/s
  */
  float theta = 2 * PI * n; // radians
  return linSpeed * sqrt(pow(Lt, 2) + pow(theta * radius, 2)) / (theta * pow(radius, 2)); //radians/second
}

int sgn(float num) {
  /*
     Returns the sign of a number
  */
  return (num > 0.0f) - (num < 0.0f);
}

float Voltage2Force(int forceBits) {
  /*
     function converts the bits from ADS1115 to force in Newtons.
     calibration was performed with coefficients identified using fmincon
  */

  if (forceBits < 10) { //bits should never be below zero. negative means no reading
    float out = 0;
    return out;
  } else {
    float p1, p2, p3, p4, p5, p6, p7, p8, p9;
    p1 = 0.1395; p2 = -1.878; p3 = 10.52; p4 = -31.58;
    p5 = 54.86; p6 = -55.14; p7 = 29.8; p8 = -5.521;
    p9 = 1.216;

    //Convert bits to voltage, V
    float SCALE = 0.18725 / 1000; // V per bit.
    float forceVolts = forceBits * SCALE;

    float out = p1 * pow(forceVolts, 8) + p2 * pow(forceVolts, 7) + p3 * pow(forceVolts, 6) +
                p4 * pow(forceVolts, 5) + p5 * pow(forceVolts, 4) + p6 * pow(forceVolts, 3) +
                p7 * pow(forceVolts, 2) + p8 * forceVolts + p9;

    return out; //force in Newtons
  }
}

void SineTraj(state_t& des_state, unsigned currTime) {
  /*
     generates signals for sinusiadal position, velocity and acceleration of the
     tendon lengths, hence only 1D
  */

  des_state.pos.x = sin(currTime);
  des_state.pos.y = 0;
  des_state.pos.z = 0;

  des_state.vel.x = cos(currTime);
  des_state.vel.y = 0;
  des_state.vel.z = 0;

  des_state.acc.x = -1 * sin(currTime);
  des_state.acc.y = 0;
  des_state.acc.z = 0;

  /*
    des_state.pos.x = sin(currTime);
    des_state.pos.y = -1 * cos(currTime);
    des_state.pos.z = sin(currTime);

    des_state.vel.x = cos(currTime);
    des_state.vel.y = sin(currTime);
    des_state.vel.z = cos(currTime);

    des_state.acc.x = -1 * sin(currTime);
    des_state.acc.y = cos(currTime);
    des_state.acc.z = -1 * sin(currTime);
  */
}

void CubicTrajCoeff (polyCoef_t& Coeff, unsigned t_init, unsigned t_final, float PosInit,
                     float PosFinal, float VelInit, float VelFinal) {
  /*
     computes the coefficients to fit a cubic function for a given
     set of initial and final conditions for velocity and acceleration
  */
  //unsigned T = (t_final - t_init) * 1e-3;
  unsigned T = t_final * 1e-3;
  Coeff.a0 = PosInit;
  Coeff.a1 = VelInit;
  //Coeff.a2 = -1 * (3 * PosInit - 3 * PosFinal - VelInit + VelFinal + 3 * T * VelInit) / (T * (3 * T - 2));
  //Coeff.a3 = (2 * PosInit - 2 * PosFinal + T * VelInit + T * VelFinal) / (pow(T, 3) * (3 * T - 2));
  Coeff.a2 = 3 * (PosFinal - PosInit) / (pow(T, 2));
  Coeff.a3 = -2 * (PosFinal - PosInit) / (pow(T, 3));
  Coeff.a4 = 0;
  Coeff.a5 = 0;

  /*
    Serial.print(Coeff.a0); Serial.print(",");
    Serial.print(Coeff.a1); Serial.print(",");
    Serial.print(Coeff.a2, 5); Serial.print(",");
    Serial.print(Coeff.a3,8); Serial.print(",");
    Serial.print(Coeff.a4); Serial.print(",");
    Serial.print(Coeff.a5); Serial.print(",");
    Serial.print(PosFinal); Serial.print(",");
    Serial.print(T); Serial.println();
  */
}

void QuinticTrajCoeff (polyCoef_t& Coeff, unsigned t_init, unsigned t_final, float PosInit,
                       float PosFinal, float VelInit, float VelFinal, float AccInit, float AccFinal) {
  /*
     Generates the constants to fit in a quintic polynomial when the initial
     position, velocity and acceleration together with final position, velocity and accelerations
     are specified.
  */
  unsigned T = t_final - t_init;
  Coeff.a0 = PosInit;
  Coeff.a1 = VelInit;
  Coeff.a2 = AccInit / 2;
  Coeff.a3 = -1 * (20 * PosInit - 20 * PosFinal + 12 * T * VelInit + 8 * T * VelFinal + 3 * AccInit *
                   pow(T, 2) - AccFinal * pow(T, 2)) / (2 * pow(T, 3));
  Coeff.a4 = (30 * PosInit - 30 * PosFinal + 16 * T * VelInit + 14 * T * VelFinal + 3 * AccInit *
              pow(T, 2) - 2 * AccFinal * pow(T, 2)) / (2 * pow(T, 4));
  Coeff.a5 = -1 * (12 * PosInit - 12 * PosFinal + 6 * T * VelInit + 6 * T * VelFinal + AccInit *
                   pow(T, 2) - AccFinal * pow(T, 2)) / (2 * pow(T, 5));
}

void CubicTraj (state_t& des_state, polyCoef_t& Coeff, double t) {
  /*
     given the constants for a cubic function and time, compute the target position (tendon length),
     and tendon velocity. Time is in seconds.
  */
  des_state.pos.x = Coeff.a0 + Coeff.a1 * t + Coeff.a2 * pow(t, 2) + Coeff.a3 * pow(t, 3);
  des_state.pos.y = 0.0;
  des_state.pos.z = 0.0;

  des_state.vel.x = Coeff.a1 + 2 * Coeff.a2 * t + 3 * Coeff.a3 * pow(t, 2);
  des_state.vel.y = 0.0;
  des_state.vel.z = 0.0;

  des_state.acc.x = 2 * Coeff.a2 + 6 * Coeff.a3 * t;
  des_state.acc.y = 0.0;
  des_state.acc.z = 0.0;

  /*
    Serial.println();
    Serial.print(des_state.pos.x, 8); Serial.print(",");
    Serial.print(des_state.vel.x, 8); Serial.print(",");
    Serial.print(t, 4); Serial.println();
  */
}

void QuinticTraj (state_t& des_state, polyCoef_t& Coeff, unsigned t) {
  /*
     compute the tendon position/length, tendon velocity, and tendon acceleration given the constants
     for quintic polynomial and the current time
  */
  des_state.pos.x = Coeff.a0 + Coeff.a1 * t + Coeff.a2 * pow(t, 2) + Coeff.a3 * pow(t, 3) + Coeff.a4 * pow(t, 4) + Coeff.a5 * pow(t, 5);
  des_state.pos.y = 0.0;
  des_state.pos.z = 0.0;

  des_state.vel.x = Coeff.a1 + 2 * Coeff.a2 * t + 3 * Coeff.a3 * pow(t, 2) + 4 * Coeff.a4 * pow(t, 3) + 5 * Coeff.a5 * pow(t, 5);
  des_state.vel.y = 0.0;
  des_state.vel.z = 0.0;

  des_state.acc.x = 2 * Coeff.a2 + 6 * Coeff.a3 * t + 12 * Coeff.a4 * pow(t, 2) + 20 * Coeff.a5 * pow(t, 3);
  des_state.acc.y = 0.0;
  des_state.acc.z = 0.0;

}

void computeTendonLengths (state_t& tendon_len, state_t& desired_theta) {
  /*
     Note that theta_d is in degrees. Need to be converted to radians for these computations
  */

  //cm
  double a1 = 5.076; double b1 = 5.076;
  double a2 = 5.216; double b2 = 5.216;
  double a3 = 4.252; double b3 = 4.252;

  //mm
  float d_1 = 18.94; float d_2 = 16.55; float d_3 = 15.80;

  //cm
  float r1 = d_1 / (2 * 10); float r2 = d_2 / (2 * 10); float r3 = d_3 / (2 * 10);

  float RADIANS_TO_DEGREES = 180 / 3.14159;

  double theta1 = desired_theta.pos.x / RADIANS_TO_DEGREES;
  double theta2 = desired_theta.pos.y / RADIANS_TO_DEGREES;
  double theta3 = desired_theta.pos.z / RADIANS_TO_DEGREES;

  double q1 = sqrt(pow(a1, 2) + pow(b1, 2) - 2 * a1 * b1 * cos(PI - theta1)) +
              sqrt(pow(a2, 2) + pow(b2, 2) - 2 * a2 * b2 * cos(PI - theta2)) +
              sqrt(pow(a3, 2) + pow(b3, 2) - 2 * a3 * b3 * cos(PI - theta3));
  double q2 = r1 * theta1 + r2 * theta2 + r3 * theta3;

  tendon_len.pos.x = q1; //cm
  tendon_len.pos.y = q2; //cm
  tendon_len.pos.z = 0.0;
}

void computeTendonVelocity (state_t& tendon_vel, state_t& desired_theta) {

  //cm
  double a1 = 5.076; double b1 = 5.076;
  double a2 = 5.216; double b2 = 5.216;
  double a3 = 4.252; double b3 = 4.252;

  //mm
  float d_1 = 18.94; float d_2 = 16.55; float d_3 = 15.80;

  //cm
  float r1 = d_1 / (2 * 10); float r2 = d_2 / (2 * 10); float r3 = d_3 / (2 * 10);

  float RADIANS_TO_DEGREES = 180 / 3.14159;

  double theta1 = desired_theta.pos.x / RADIANS_TO_DEGREES;
  double theta2 = desired_theta.pos.y / RADIANS_TO_DEGREES;
  double theta3 = desired_theta.pos.z / RADIANS_TO_DEGREES;

  double dtheta1 = desired_theta.vel.x / RADIANS_TO_DEGREES;
  double dtheta2 = desired_theta.vel.y / RADIANS_TO_DEGREES;
  double dtheta3 = desired_theta.vel.z / RADIANS_TO_DEGREES;

  double h11 = -a1 * b1 * sin(theta1) / sqrt(pow(a1, 2) + pow(b1, 2) + 2 * a1 * b1 * cos(theta1));
  double h12 = -a2 * b2 * sin(theta2) / sqrt(pow(a2, 2) + pow(b2, 2) + 2 * a2 * b2 * cos(theta2));
  double h13 = -a3 * b3 * sin(theta3) / sqrt(pow(a3, 2) + pow(b3, 2) + 2 * a3 * b3 * cos(theta3));

  const tmm::Scalar H_theta[2][3] = {
    {h11, h12, h13},
    {r1, r2, r3},
  };
  
  tmm::Matrix<2, 3> H(H_theta);

  tmm::Scalar dtheta_v[3][1] = {
    {dtheta1},
    {dtheta2},
    {dtheta3}
  };
  tmm::Matrix<3, 1> dottheta(dtheta_v);

  tmm::Matrix<2, 1> comp_tendon_vel = H * dottheta;

  tendon_vel.pos.x = comp_tendon_vel[0][0]; //cm/s
  tendon_vel.pos.y = comp_tendon_vel[1][0]; //cm/s
  tendon_vel.pos.z = 0.0;
}
