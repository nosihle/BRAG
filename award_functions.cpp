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

/*
  void calibrateIMUs(IMUData& data, Adafruit_LSM6DS33 lsm6ds33, sensors_event_t& acc, sensors_event_t& gyro, sensors_event_t& temp) { // Needs that ai0,gi0 be already defined
  //TODO - change so that it outputs ai0,gi0
  int num_readings = 1000;

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    lsm6ds33.getEvent(&acc, &gyro, &temp);
    data.aX0 += acc.acceleration.x;
    data.aY0 += acc.acceleration.y;
    data.aZ0 += acc.acceleration.z;
    data.gX0 += gyro.gyro.x;
    data.gY0 += gyro.gyro.y;
    data.gZ0 += gyro.gyro.z;
    //delay(10);
  }
  data.aX0 /= num_readings;
  data.aY0 /= num_readings;
  data.aZ0 /= num_readings;
  data.gX0 /= num_readings;
  data.gY0 /= num_readings;
  data.gZ0 /= num_readings;
  }
*/

void calibrateIMUs(Adafruit_LSM6DS33 lsm6ds33, int channel, sensors_event_t& acc, sensors_event_t& gyr, sensors_event_t& tmp, AnglesComps& OutBias) {
  int num_readings = 100;
  float aX, aY, aZ, gX, gY, gZ;

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    readIMUData(lsm6ds33, channel, acc, gyr, tmp);

    aX += acc.acceleration.x;
    aY += acc.acceleration.y;
    aZ += acc.acceleration.z;
    gX += gyr.gyro.x;
    gY += gyr.gyro.y;
    gZ += gyr.gyro.z;

    //delay(10);
  }

  OutBias.aX = aX / num_readings;
  OutBias.aY = aY / num_readings;
  OutBias.aZ = aZ / num_readings;
  OutBias.gX = gX / num_readings;
  OutBias.gY = gY / num_readings;
  OutBias.gZ = gZ / num_readings;
  OutBias.d_time = 0;

}

void readIMUData(Adafruit_LSM6DS33 lsm6ds33, int channel, sensors_event_t& acc, sensors_event_t& gyro, sensors_event_t& temp) {
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


int sgn(float num) {
  /*
     Returns the sign of a number
  */
  return (num > 0.0f) - (num < 0.0f);
}

float Voltage2Force(int forceBits) {
  float p1, p2, p3, p4, p5, p6, p7, p8, p9;
  p1 = 0.1395; p2 = -1.878; p3 = 10.52; p4 = -31.58;
  p5 = 54.86; p6 = -55.14; p7 = 29.8; p8 = -5.521;
  p9 = 1.216;

  float out = p1 * pow(forceBits, 8) + p2 * pow(forceBits, 7) + p3 * pow(forceBits, 6) +
              p4 * pow(forceBits, 5) + p5 * pow(forceBits, 4) + p6 * pow(forceBits, 3) +
              p7 * pow(forceBits, 2) + p8 * forceBits + p9;

  return out;
}
