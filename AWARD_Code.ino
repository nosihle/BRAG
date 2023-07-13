/*  By: Thulani Tsabedze
    Created: 01/10/2023
    Modified: 06/06/2023

    AWARD Code
*/

#include "award_functions.h"
#include "dynamics.h"
#include "finger_pos.h"
#include <SD.h>

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;
Adafruit_ADS1115 ads4;

//=====================================================================
float GR[] = {30.0, 30.0, 50.0, 30.0, 30.0, 50.0}; //Gear ratios of motors used
const int CPR = 12, ER_MARGIN = 20;
float ROT_D[] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0}; //target
const int nM = 6; //number of motors in the device
double vel[nM] = {}; //velocity for motors
double velPD[nM] = {}; //compensated command velocity
const int motor[2] = {};
int Revs_Count[] = {0, 0, 0, 0, 0, 0}; double Revs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

struct EncderCounts {
  volatile long m1 = 0;
  volatile long m2 = 0;
  volatile long m3 = 0;
  volatile long m4 = 0;
  volatile long m5 = 0;
  volatile long m6 = 0;
};

struct encRevs {
  float m1 = 0;
  float m2 = 0;
  float m3 = 0;
  float m4 = 0;
  float m5 = 0;
  float m6 = 0;
};

//need to double check these for servocity motors, units in kg.mm/A
float Kt_10 = 1/0.82;
float Kt_15 = 1/0.56;
float Kt_30 = 1/0.31;
float Kt_50 = 1/0.19;

EncderCounts ticks;
encRevs rots;
float goalRots = 100;
long int goalTicks = goalRots * 10; // GR = 15;

//---------------------------------------------------------------------
//Define motor direction control pins
const int MJ1_BIN1 = 0, MJ1_BIN2 = 1, MJ2_BIN1 = 22, MJ2_BIN2 = 23;
const int MJ3_BIN1 = 33, MJ3_BIN2 = 12, MJ4_BIN1 = 36, MJ4_BIN2 = 37;
const int MJ5_BIN1 = 24, MJ5_BIN2 = 25, MJ6_BIN1 = 28, MJ6_BIN2 = 29;
const int MJ_BIN[] = {MJ1_BIN1, MJ1_BIN2, MJ2_BIN1, MJ2_BIN2, MJ3_BIN1, MJ3_BIN2,
                      MJ4_BIN1, MJ4_BIN2, MJ5_BIN1, MJ5_BIN2, MJ6_BIN1, MJ6_BIN2
                     }; //All motor pins
//define motors individually, easier for control implementation
const int m_FNT1[] = {MJ_BIN[0], MJ_BIN[1]}, m_FNT2[] = {MJ_BIN[2], MJ_BIN[3]};
const int m_FNT3[] = {MJ_BIN[4], MJ_BIN[5]}, m_FNT4[] = {MJ_BIN[6], MJ_BIN[7]};
const int m_BCK1[] = {MJ_BIN[8], MJ_BIN[9]}, m_BCK2[] = {MJ_BIN[10], MJ_BIN[11]};

//---------------------------------------------------------------------
//Define encoder a and b state pins
const int MJ1_EN_A = 2, MJ1_EN_B = 3, MJ2_EN_A = 5, MJ2_EN_B = 4;
const int MJ3_EN_A = 6, MJ3_EN_B = 7, MJ4_EN_A = 8, MJ4_EN_B = 9;
const int MJ5_EN_A = 10, MJ5_EN_B = 11, MJ6_EN_A = 16, MJ6_EN_B = 17;

//---------------------------------------------------------------------
//Create 6 encoder objects for the six motors
Encoder FNT1(MJ1_EN_A, MJ1_EN_B); Encoder FNT2(MJ2_EN_A, MJ2_EN_B);
Encoder FNT3(MJ3_EN_A, MJ3_EN_B); Encoder FNT4(MJ4_EN_A, MJ4_EN_B);
Encoder BCK1(MJ5_EN_A, MJ5_EN_B); Encoder BCK2(MJ6_EN_A, MJ6_EN_B);

//create an array of encoder objects
//Encoder encoders[6] = {AD, AB, FL, EX, PR, SU};
int COUNTER = 0;

//Initialize sensor reading, Current, SCPs, FSRs
FsrScpData sensorData;


//initialize for kinematics and dynamics
double M[9], C[9], B[9], G[3], finger[9];
double state_pos[3] = {.1, 1, 2};
double fin_length[3] = {3, 4, 5};

struct fingAngles {
  float a1, a2, a3;
};

//IMU needs
Adafruit_LSM6DS33 lsm6ds33;
#define SAMPLE_RATE 100.0 // in Hz
unsigned long microsPerReading, microsPrevious;

//Define MUX control pins and SIG_pin
const int s0 = 30, s1 = 31, s2 = 32, s3 = 34;
const int ctrl_pins[] = {s0, s1, s2, s3};
const int com_pin = 38;
const int numIMUs = 9;

const int chipSelect = BUILTIN_SDCARD;

const int pinNum = 40; //32

// For software-SPI mode we need SCK/MOSI/MISO pins
const int LSM_SCK = 27;
const int LSM_MISO = 39;
const int LSM_MOSI = 26;

Madgwick filter0, filter1, filter2, filter3, filter4, filter5, filter6, filter7, filter8;

float roll, roll1, roll_last, pitch, pitch_last, yaw, yaw_last, yaw_diff;
sensors_event_t accel1, accel2, accel3, accel4, accel5, accel6, accel7, accel8, accel1_last;
sensors_event_t gyro1, gyro2, gyro3, gyro4, gyro5, gyro6, gyro7, gyro8, gyro1_last;
sensors_event_t temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp1_last;

fingAngles fin_angles;
AnglesComps anglesOld1, anglesOld2, anglesOld3, anglesOld6;
AnglesComps BiasIMU1, BiasIMU2, BiasIMU3, BiasIMU6;
EulerAngIMU IMUAngs1, IMUAngs2, IMUAngs3, IMUAngs6;


float start_time_2; float start_time;

// initialize variables to pace updates to correct rate
float dt = (1 / SAMPLE_RATE);

/*
  struct IMUData {
  float aX0, aY0, aZ0, gX0, gY0, gZ0;
  }*/

void setup(void) {
  pinMode(pinNum, INPUT_PULLUP); //push button

  //Ensure control pins are outputs and they are LOW
  for (int pin = 0; pin < 4; pin++) {
    pinMode(ctrl_pins[pin], OUTPUT);
    digitalWrite(ctrl_pins[pin], HIGH); // Connected with channel 15
  }
  pinMode(com_pin, OUTPUT);
  digitalWrite(com_pin, LOW);

  //Define motor direction control pins
  for (int iter = 0; iter < 12; iter++) { //
    pinMode(MJ_BIN[iter], OUTPUT);
  }
  //Setup ADS for current readings
  // I2C addresses are assigned based on the connection of the ADDR pin
  ads1.begin(0x48);
  ads2.begin(0x49);
  ads3.begin(0x4A);
  ads4.begin(0x4B);

  //get the reference encoder ticks initially.
  //also get initial sensor readings
  getEncCounts(ticks, rots);
  getSensorData(sensorData);

  Serial.begin(115200);

  /*
    filter0.begin(5);
    filter1.begin(5);
    filter2.begin(5);
    filter3.begin(5);
    filter4.begin(5);
    filter5.begin(5);
    filter6.begin(5);
    filter7.begin(5);
    filter8.begin(5);
  */

  microsPerReading = 1000000 * dt; //delta_t in microseconds.
  microsPrevious = micros();

  //calibrate IMUs
  Serial.println("Calibrating IMUs...");

  calibrateIMUs(lsm6ds33, 1, accel1, gyro1, temp1, BiasIMU1);
  calibrateIMUs(lsm6ds33, 2, accel2, gyro2, temp2, BiasIMU2);
  calibrateIMUs(lsm6ds33, 3, accel3, gyro3, temp3, BiasIMU3);
  calibrateIMUs(lsm6ds33, 6, accel6, gyro6, temp6, BiasIMU6);

  /*
    calibrateIMUs(lsm6ds33, 5, accel1, gyro1, temp1, BiasIMU1);
    calibrateIMUs(lsm6ds33, 8, accel2, gyro2, temp2, BiasIMU2);
  */

  //initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
  } else {
    Serial.println("card initialized.");
  }

  Serial.println("Setup done...");
  delayMicroseconds(25);
  Serial.println("Starting to run main code...");
}

void loop(void) {
  butControl(pinNum);
  //butControlRVS(pinNum);
  unsigned long t_now = micros();
  if (t_now - microsPrevious >= microsPerReading) {

    /* calibration debugging code

        readIMUData(lsm6ds33, 5, accel1, gyro1, temp1);
        readIMUData(lsm6ds33, 8, accel2, gyro2, temp2);

        computeFingerAngleCF(accel1, gyro1, anglesOld1, BiasIMU1);
        computeFingerAngleCF(accel2, gyro2, anglesOld2, BiasIMU2);

        // open the file.
        //File dataFile = SD.open("datalog.txt", FILE_WRITE);

        // if the file is available, write to it:
        //if (dataFile) {

        Serial.print(accel1.acceleration.x); Serial.print(",");
        Serial.print(accel1.acceleration.y); Serial.print(",");
        Serial.print(accel1.acceleration.z); Serial.print(",");

        Serial.print(accel2.acceleration.x); Serial.print(",");
        Serial.print(accel2.acceleration.y); Serial.print(",");
        Serial.print(accel2.acceleration.z); Serial.print(",");

        Serial.print(gyro1.gyro.x); Serial.print(",");
        Serial.print(gyro1.gyro.y); Serial.print(",");
        Serial.print(gyro1.gyro.z); Serial.print(",");

        Serial.print(gyro2.gyro.x); Serial.print(",");
        Serial.print(gyro2.gyro.y); Serial.print(",");
        Serial.print(gyro2.gyro.z); Serial.print(",");

        Serial.print(anglesOld1.aX, 3); Serial.print(",");
        Serial.print(anglesOld2.aX, 3); Serial.print(",");
        Serial.print(t_now * 1e-6, 3); //48
        Serial.println();

        //dataFile.close();

        //}
    */
    //Serial.println("Reading IMUs...");
    //Continously read data
    readIMUData(lsm6ds33, 1, accel1, gyro1, temp1);
    readIMUData(lsm6ds33, 2, accel2, gyro2, temp2);
    readIMUData(lsm6ds33, 3, accel3, gyro3, temp3);
    readIMUData(lsm6ds33, 6, accel6, gyro6, temp6);

    //Serial.println("Updating ticks and SCP,Current data...");
    getEncCounts(ticks, rots);
    getSensorData(sensorData);

    /*
      computeFingerAngle(accel1, gyro1, accel2, gyro2, accel3,
                         gyro3, accel6, gyro6, fin_angles);
      state_pos[0] = fin_angles.a1;
      state_pos[1] = fin_angles.a2;
      state_pos[2] = fin_angles.a3;
    */

    computeFingerAngleCF(accel1, gyro1, anglesOld1, BiasIMU1);
    computeFingerAngleCF(accel2, gyro2, anglesOld2, BiasIMU2);
    computeFingerAngleCF(accel3, gyro3, anglesOld3, BiasIMU3);
    computeFingerAngleCF(accel6, gyro6, anglesOld6, BiasIMU6);

    // control
    //trackingControlMM(ticks.m3, ticks.m4, m_FNT3, m_FNT4, goalTicks, t_now);

    //tsaPosControl(4, 0.015, 18, m_FNT3, m_FNT4, t_now); //units have to match

    /*
        //cast data to the serial monitors or record to SD card
        File dataFile = SD.open("datalog8.txt", FILE_WRITE);  // open the file.

        // if the file is available, write to it:

        if (dataFile) {
          //Serial.println("Printing to SC card");
          dataFile.print(accel1.acceleration.x); dataFile.print(",");
          dataFile.print(accel1.acceleration.y); dataFile.print(",");
          dataFile.print(accel1.acceleration.z); dataFile.print(",");

          dataFile.print(accel2.acceleration.x); dataFile.print(",");
          dataFile.print(accel2.acceleration.y); dataFile.print(",");
          dataFile.print(accel2.acceleration.z); dataFile.print(",");

          dataFile.print(accel3.acceleration.x); dataFile.print(",");
          dataFile.print(accel3.acceleration.y); dataFile.print(",");
          dataFile.print(accel3.acceleration.z); dataFile.print(",");

          dataFile.print(accel6.acceleration.x); dataFile.print(",");
          dataFile.print(accel6.acceleration.y); dataFile.print(",");
          dataFile.print(accel6.acceleration.z); dataFile.print(",");

          dataFile.print(gyro1.gyro.x); dataFile.print(",");
          dataFile.print(gyro1.gyro.y); dataFile.print(",");
          dataFile.print(gyro1.gyro.z); dataFile.print(",");

          dataFile.print(gyro2.gyro.x); dataFile.print(",");
          dataFile.print(gyro2.gyro.y); dataFile.print(",");
          dataFile.print(gyro2.gyro.z); dataFile.print(",");

          dataFile.print(gyro3.gyro.x); dataFile.print(",");
          dataFile.print(gyro3.gyro.y); dataFile.print(",");
          dataFile.print(gyro3.gyro.z); dataFile.print(",");

          dataFile.print(gyro6.gyro.x); dataFile.print(",");
          dataFile.print(gyro6.gyro.y); dataFile.print(",");
          dataFile.print(gyro6.gyro.z); dataFile.print(",");

          dataFile.print(anglesOld1.aX, 3); dataFile.print(",");
          dataFile.print(anglesOld2.aX, 3); dataFile.print(",");
          dataFile.print(anglesOld3.aX, 3); dataFile.print(",");
          dataFile.print(anglesOld6.aX, 3); dataFile.print(",");

          dataFile.print(ticks.m3); dataFile.print(",");
          dataFile.print(rots.m3); dataFile.print(",");
          dataFile.print(ticks.m4); dataFile.print(",");
          dataFile.print(rots.m4); dataFile.print(",");

          dataFile.print(sensorData.amps3); dataFile.print(",");
          dataFile.print(sensorData.amps4); dataFile.print(",");
          dataFile.print(sensorData.scp1); dataFile.print(",");

          dataFile.print(t_now * 1e-6, 3);
          dataFile.println();

          dataFile.close();
    */

    /*
          //Aslo print to serial monitor for immediate checking

          Serial.print(accel1.acceleration.x); Serial.print(",");
          Serial.print(accel1.acceleration.y); Serial.print(",");
          Serial.print(accel1.acceleration.z); Serial.print(",");

          Serial.print(accel2.acceleration.x); Serial.print(",");
          Serial.print(accel2.acceleration.y); Serial.print(",");
          Serial.print(accel2.acceleration.z); Serial.print(",");

          Serial.print(accel3.acceleration.x); Serial.print(",");
          Serial.print(accel3.acceleration.y); Serial.print(",");
          Serial.print(accel3.acceleration.z); Serial.print(",");

          Serial.print(accel6.acceleration.x); Serial.print(",");
          Serial.print(accel6.acceleration.y); Serial.print(",");
          Serial.print(accel6.acceleration.z); Serial.print(",");

          Serial.print(gyro1.gyro.x); Serial.print(",");
          Serial.print(gyro1.gyro.y); Serial.print(",");
          Serial.print(gyro1.gyro.z); Serial.print(",");

          Serial.print(gyro2.gyro.x); Serial.print(",");
          Serial.print(gyro2.gyro.y); Serial.print(",");
          Serial.print(gyro2.gyro.z); Serial.print(",");

          Serial.print(gyro3.gyro.x); Serial.print(",");
          Serial.print(gyro3.gyro.y); Serial.print(",");
          Serial.print(gyro3.gyro.z); Serial.print(",");

          Serial.print(gyro6.gyro.x); Serial.print(",");
          Serial.print(gyro6.gyro.y); Serial.print(",");
          Serial.print(gyro6.gyro.z); Serial.print(",");

          Serial.print(anglesOld1.aX, 3); Serial.print(",");
          Serial.print(anglesOld2.aX, 3); Serial.print(",");
          Serial.print(anglesOld3.aX, 3); Serial.print(",");
          Serial.print(anglesOld6.aX, 3); Serial.print(","); */

    Serial.print(ticks.m3); Serial.print(",");
    Serial.print(rots.m3); Serial.print(",");
    Serial.print(ticks.m4); Serial.print(",");
    Serial.print(rots.m4); Serial.print(",");

    
    Serial.print(sensorData.amps3); Serial.print(",");
    Serial.print(sensorData.amps4); Serial.print(",");
    
    Serial.println();
    /*

      Serial.print(sensorData.amps3); Serial.print(",");
      Serial.print(sensorData.amps4); Serial.print(",");
      Serial.print(sensorData.scp1); Serial.print(",");

      Serial.print(t_now * 1e-6, 3);
      Serial.println();
      } */
  }

  // increment previous time, so we keep proper pace
  microsPrevious = microsPrevious + microsPerReading;

  /*
    dynamics(state_pos, fin_length, M, C, B, G);
    finger_pos(state_pos, fin_length, finger);
  */

}

//Supporting functions
//-----------------------------------------------------

void dataPrint(int Transient_Seconds, unsigned long currTime) {
  unsigned long Time_Started = micros() + Transient_Seconds;
  while (micros() <= Time_Started) {
    Serial.println( (String)
                    + accel1.acceleration.x + "," + accel1.acceleration.y + "," + accel1.acceleration.z +
                    "," + accel2.acceleration.x + "," + accel2.acceleration.y + "," + accel2.acceleration.z +
                    "," + accel3.acceleration.x + "," + accel3.acceleration.y + "," + accel3.acceleration.z +
                    "," + accel6.acceleration.x + "," + accel6.acceleration.y + "," + accel6.acceleration.z +
                    "," + gyro1.gyro.x + "," + gyro1.gyro.y + "," + gyro1.gyro.z +
                    "," + gyro2.gyro.x + "," + gyro2.gyro.y + "," + gyro2.gyro.z +
                    "," + gyro3.gyro.x + "," + gyro3.gyro.y + "," + gyro3.gyro.z +
                    "," + gyro6.gyro.x + "," + gyro6.gyro.y + "," + gyro6.gyro.z +
                    "," + IMUAngs1.roll + "," + IMUAngs1.pitch + "," + IMUAngs2.roll +
                    "," + IMUAngs2.pitch + "," + IMUAngs3.roll + "," + IMUAngs3.pitch +
                    "," + IMUAngs6.roll + "," + IMUAngs6.pitch + "," + ticks.m3 +
                    "," + rots.m3 + "," + ticks.m4 + "," + rots.m4 +
                    "," + sensorData.amps3 + "," + sensorData.amps4 + "," + sensorData.scp1 +
                    "," + currTime * 1e-6);

    //delay(DATA_DELAY);
    delayMicroseconds(1);

    /*
        Serial.print(accel1.acceleration.x); Serial.print(",");
        Serial.print(accel1.acceleration.y); Serial.print(",");
        Serial.print(accel1.acceleration.z); Serial.print(",");

        Serial.print(accel2.acceleration.x); Serial.print(",");
        Serial.print(accel2.acceleration.y); Serial.print(",");
        Serial.print(accel2.acceleration.z); Serial.print(",");

        Serial.print(accel3.acceleration.x); Serial.print(",");
        Serial.print(accel3.acceleration.y); Serial.print(",");
        Serial.print(accel3.acceleration.z); Serial.print(",");

        Serial.print(accel6.acceleration.x); Serial.print(",");
        Serial.print(accel6.acceleration.y); Serial.print(",");
        Serial.print(accel6.acceleration.z); Serial.print(",");

        Serial.print(gyro1.gyro.x); Serial.print(",");
        Serial.print(gyro1.gyro.y); Serial.print(",");
        Serial.print(gyro1.gyro.z); Serial.print(",");

        Serial.print(gyro2.gyro.x); Serial.print(",");
        Serial.print(gyro2.gyro.y); Serial.print(",");
        Serial.print(gyro2.gyro.z); Serial.print(",");

        Serial.print(gyro3.gyro.x); Serial.print(",");
        Serial.print(gyro3.gyro.y); Serial.print(",");
        Serial.print(gyro3.gyro.z); Serial.print(",");

        Serial.print(gyro6.gyro.x); Serial.print(",");
        Serial.print(gyro6.gyro.y); Serial.print(",");
        Serial.print(gyro6.gyro.z); Serial.print(",");

        Serial.print(IMUAngs1.roll, 3); Serial.print(",");
        Serial.print(IMUAngs1.pitch, 3); Serial.print(",");

        Serial.print(IMUAngs2.roll, 3); Serial.print(",");
        Serial.print(IMUAngs2.pitch, 3); Serial.print(",");

        Serial.print(IMUAngs3.roll, 3); Serial.print(",");
        Serial.print(IMUAngs3.pitch, 3); Serial.print(",");

        Serial.print(IMUAngs6.roll, 3); Serial.print(",");
        Serial.print(IMUAngs6.pitch, 3); Serial.print(",");

        Serial.print(ticks.m3); Serial.print(",");
        Serial.print(rots.m3); Serial.print(",");
        Serial.print(ticks.m4); Serial.print(",");
        Serial.print(rots.m4); Serial.print(",");

        Serial.print(sensorData.amps3); Serial.print(",");
        Serial.print(sensorData.amps4); Serial.print(",");
        Serial.print(sensorData.scp1); Serial.print(",");

        Serial.print(t_now * 1e-6, 3);
        Serial.println();
    */
  }
}

void dataPrintOld(int Transient_Seconds) {
  unsigned long Time_Started = micros() + Transient_Seconds * 1000000;
  while (micros() <= Time_Started) {
    //getEncCounts();
    getSensorData(sensorData);

    Serial.println( (String) + sensorData.fsr1 + "," + sensorData.fsr2 + "," + sensorData.fsr3 +
                    "," + sensorData.fsr4 + "," + sensorData.scp1 + "," + sensorData.scp2 +
                    "," + sensorData.scp3 + "," + sensorData.scp4 + "," + Revs_Count[2] +
                    "," + Revs_Count[3] + "," + Revs_Count[4] + "," + Revs_Count[5] );

    //                        Serial.println( (String) + Revs_Count[0] + "," + Revs[0] + "," + Revs_Count[1] +
    //                    "," + Revs[1] + "," + Revs_Count[2] + "," + Revs[2] +
    //                    "," + Revs_Count[3] + "," + Revs[3] + "," + Revs_Count[4] +
    //                    "," + Revs[4] + "," + Revs_Count[5] + "," + Revs[5] );
    //delay(DATA_DELAY);
    delayMicroseconds(1);
  }
}


void getEncCounts(EncderCounts& mTicks, encRevs& mRots) {
  /*
     Assumes that motors are of the same gear ratio. If different,
     this needs to be updated.

  */

  const float gr = 10.0; // confirm with motors being used
  const int CPR = 12;  //counts per revolution

  mTicks.m1 = FNT1.read();
  mTicks.m2 = FNT2.read();
  mTicks.m3 = FNT3.read();
  mTicks.m4 = FNT4.read();
  mTicks.m5 = BCK1.read();
  mTicks.m6 = BCK2.read();

  mRots.m1 = mTicks.m1 / (gr * CPR);
  mRots.m2 = mTicks.m2 / (gr * CPR);
  mRots.m3 = mTicks.m3 / (gr * CPR);
  mRots.m4 = mTicks.m4 / (gr * CPR);
  mRots.m5 = mTicks.m5 / (gr * CPR);

}

void getSensorData(FsrScpData& data) { // Read the data from the ADS 1115.

  float Viout = 0.245 * 1000; // mV Zero current Output Voltage
  float SCALE = 0.18725; // mV per bit. depends on gain of ADS
  float SENSITIVITY = 400; // mV/A

  ads1.setGain(GAIN_TWOTHIRDS);
  long amps1 = ads1.readADC_SingleEnded(0);
  long amps2 = ads1.readADC_SingleEnded(1);
  long amps3 = ads1.readADC_SingleEnded(2);
  long amps4 = ads1.readADC_SingleEnded(3);

  ads2.setGain(GAIN_TWOTHIRDS);
  long amps5 = ads2.readADC_SingleEnded(0);
  long amps6 = ads2.readADC_SingleEnded(1);
  data.scp1 = ads2.readADC_SingleEnded(2);
  data.scp2 = ads2.readADC_SingleEnded(3);

  ads3.setGain(GAIN_TWOTHIRDS);
  data.scp3 = ads3.readADC_SingleEnded(0);
  data.scp4 = ads3.readADC_SingleEnded(1);
  data.fsr1 = ads3.readADC_SingleEnded(2);
  data.fsr2 = ads3.readADC_SingleEnded(3);

  ads4.setGain(GAIN_TWOTHIRDS); //Two channels are open and not currently used
  data.fsr3 = ads4.readADC_SingleEnded(0);
  data.fsr4 = ads4.readADC_SingleEnded(1);

  //convert to Amps using sensor datasheet
  data.amps1 = (amps1 * SCALE - Viout) / SENSITIVITY;
  data.amps2 = (amps2 * SCALE - Viout) / SENSITIVITY;
  data.amps3 = (amps3 * SCALE - Viout) / SENSITIVITY;
  data.amps4 = (amps4 * SCALE - Viout) / SENSITIVITY;
  data.amps5 = (amps5 * SCALE - Viout) / SENSITIVITY;
  data.amps6 = (amps6 * SCALE - Viout) / SENSITIVITY;

}

void tsaPosControl(float travelGoal, float radius, float L0, int MJ_BIN_m1[], int MJ_BIN_m2[], unsigned long duration) {

  /*
     The function controls the rotations of the motors by controlling the contraction
     of the TSA using the TSA governing equations
     currPos = the current position of the TSA
     travelGoal = the desired contraction/travel from current position
     radius = radius of the strings of the TSAs
     L0 = the initial length of the TSA
     MJ_BIN_m  = motor control pins
  */

  //compute the rotations needed to meet desired travel
  float needRots = dispToRots(travelGoal, radius, L0);  //RotsToDisp(n, radius, L0)
  int GR = 10;
  long int targetTicks = needRots * GR;
  //position control using the computed rotations
  trackingControlMM(ticks.m3, ticks.m4, MJ_BIN_m1, MJ_BIN_m2, targetTicks, duration);

}



void trackingControlMM(long m1_ticks, long m2_ticks, int MJ_BIN_m1[], int MJ_BIN_m2[], long int targetTicks, unsigned long duration) {
  /*
     This function controls two antagonistic motors to attain a desired position of a finger
     In this case, it is assumed that the motors are of the same gear ratio, hence speed
     If the motors are of the different gear ratios, the faster motor needs to be slowed down
     by scaling the computed PWM. The scaling factor will be determined experimentally
  */

  int ER_MARGIN = 50;
  long int err_m1 = m1_ticks - targetTicks;
  long int err_m2 = m2_ticks - targetTicks;
  long int Err_m = 0;

  if (abs(err_m1 > err_m2)) {
    Err_m = err_m1;
  } else {
    Err_m = err_m2;
  }
  // define PID control gains
  float KP = 0.0005;
  float KD = 0;
  float loopTime = 0.02; //


  while (abs(Err_m) > ER_MARGIN) {

    double temp_command1 = KP * Err_m + KD * (Err_m / loopTime); //looptime is 1/frequency

    // normalize to PWM using tanh
    double vel_PD = tanh(temp_command1) * 255;

    if (abs(vel_PD) < 80) { //minimum PWM to overcome friction
      vel_PD  = 80;
    }

    if (Err_m > ER_MARGIN) { //overshot target, reverse
      RVS_Motors(MJ_BIN_m1, abs(vel_PD)); //motors move opposite each other
      FWD_Motors(MJ_BIN_m2, abs(vel_PD));
    }

    if (Err_m < ER_MARGIN) { //havent reached target, continue
      FWD_Motors(MJ_BIN_m1, abs(vel_PD));
      RVS_Motors(MJ_BIN_m2, abs(vel_PD));
    }

    // read sensor data
    readIMUData(lsm6ds33, 1, accel1, gyro1, temp1);
    readIMUData(lsm6ds33, 2, accel2, gyro2, temp2);
    readIMUData(lsm6ds33, 3, accel3, gyro3, temp3);
    readIMUData(lsm6ds33, 6, accel6, gyro6, temp6);

    getEncCounts(ticks, rots);
    getSensorData(sensorData);
    /*
        //update error and perform other computations
        computeFingerAngle(filter0, accel1, gyro1, BiasIMU1, IMUAngs1);
        computeFingerAngle(filter1, accel2, gyro2, BiasIMU2, IMUAngs2);
        computeFingerAngle(filter2, accel3, gyro3, BiasIMU3, IMUAngs3);
        computeFingerAngle(filter3, accel6, gyro6, BiasIMU6, IMUAngs6);
    */
    computeFingerAngleCF(accel1, gyro1, anglesOld1, BiasIMU1);
    computeFingerAngleCF(accel2, gyro2, anglesOld2, BiasIMU2);
    computeFingerAngleCF(accel3, gyro3, anglesOld3, BiasIMU3);
    computeFingerAngleCF(accel6, gyro6, anglesOld6, BiasIMU6);

    // we know where these motors are connected for their encoders
    err_m1 = ticks.m3 - targetTicks;
    err_m2 = ticks.m4 - targetTicks;

    if (abs(err_m1 > err_m2)) {
      Err_m = err_m1;
    } else {
      Err_m = err_m2;
    }
    /*
      Serial.print(Err_m); Serial.print(","); Serial.print(vel_PD);
      Serial.print(","); Serial.print(temp_command1);
      Serial.println(); */

    //print data/write data to SD card
    /*
        File dataFile = SD.open("datalog1.txt", FILE_WRITE);  // open the file.
        if (dataFile) {

          dataFile.print(accel1.acceleration.x); dataFile.print(",");
          dataFile.print(accel1.acceleration.y); dataFile.print(",");
          dataFile.print(accel1.acceleration.z); dataFile.print(",");

          dataFile.print(accel2.acceleration.x); dataFile.print(",");
          dataFile.print(accel2.acceleration.y); dataFile.print(",");
          dataFile.print(accel2.acceleration.z); dataFile.print(",");

          dataFile.print(accel3.acceleration.x); dataFile.print(",");
          dataFile.print(accel3.acceleration.y); dataFile.print(",");
          dataFile.print(accel3.acceleration.z); dataFile.print(",");

          dataFile.print(accel6.acceleration.x); dataFile.print(",");
          dataFile.print(accel6.acceleration.y); dataFile.print(",");
          dataFile.print(accel6.acceleration.z); dataFile.print(",");

          dataFile.print(gyro1.gyro.x); dataFile.print(",");
          dataFile.print(gyro1.gyro.y); dataFile.print(",");
          dataFile.print(gyro1.gyro.z); dataFile.print(",");

          dataFile.print(gyro2.gyro.x); dataFile.print(",");
          dataFile.print(gyro2.gyro.y); dataFile.print(",");
          dataFile.print(gyro2.gyro.z); dataFile.print(",");

          dataFile.print(gyro3.gyro.x); dataFile.print(",");
          dataFile.print(gyro3.gyro.y); dataFile.print(",");
          dataFile.print(gyro3.gyro.z); dataFile.print(",");

          dataFile.print(gyro6.gyro.x); dataFile.print(",");
          dataFile.print(gyro6.gyro.y); dataFile.print(",");
          dataFile.print(gyro6.gyro.z); dataFile.print(",");

          dataFile.print(anglesOld1.aX, 3); dataFile.print(",");
          dataFile.print(anglesOld2.aX, 3); dataFile.print(",");
          dataFile.print(anglesOld3.aX, 3); dataFile.print(",");
          dataFile.print(anglesOld6.aX, 3); dataFile.print(",");

          dataFile.print(ticks.m3); dataFile.print(",");
          dataFile.print(rots.m3); dataFile.print(",");
          dataFile.print(ticks.m4); dataFile.print(",");
          dataFile.print(rots.m4); dataFile.print(",");

          dataFile.print(sensorData.amps3); dataFile.print(",");
          dataFile.print(sensorData.amps4); dataFile.print(",");
          dataFile.print(sensorData.scp1); dataFile.print(",");

          dataFile.print(duration * 1e-6, 3);
          dataFile.println();
          dataFile.close();
        }
    */
    /*
          Serial.print(accel1.acceleration.x); Serial.print(",");
          Serial.print(accel1.acceleration.y); Serial.print(",");
          Serial.print(accel1.acceleration.z); Serial.print(",");

          Serial.print(accel2.acceleration.x); Serial.print(",");
          Serial.print(accel2.acceleration.y); Serial.print(",");
          Serial.print(accel2.acceleration.z); Serial.print(",");

          Serial.print(accel3.acceleration.x); Serial.print(",");
          Serial.print(accel3.acceleration.y); Serial.print(",");
          Serial.print(accel3.acceleration.z); Serial.print(",");

          Serial.print(accel6.acceleration.x); Serial.print(",");
          Serial.print(accel6.acceleration.y); Serial.print(",");
          Serial.print(accel6.acceleration.z); Serial.print(",");

          Serial.print(gyro1.gyro.x); Serial.print(",");
          Serial.print(gyro1.gyro.y); Serial.print(",");
          Serial.print(gyro1.gyro.z); Serial.print(",");

          Serial.print(gyro2.gyro.x); Serial.print(",");
          Serial.print(gyro2.gyro.y); Serial.print(",");
          Serial.print(gyro2.gyro.z); Serial.print(",");

          Serial.print(gyro3.gyro.x); Serial.print(",");
          Serial.print(gyro3.gyro.y); Serial.print(",");
          Serial.print(gyro3.gyro.z); Serial.print(",");

          Serial.print(gyro6.gyro.x); Serial.print(",");
          Serial.print(gyro6.gyro.y); Serial.print(",");
          Serial.print(gyro6.gyro.z); Serial.print(",");

          Serial.print(anglesOld1.aX, 3); Serial.print(",");
          Serial.print(anglesOld2.aX, 3); Serial.print(",");
          Serial.print(anglesOld3.aX, 3); Serial.print(",");
          Serial.print(anglesOld6.aX, 3); Serial.print(",");

          Serial.print(ticks.m3); Serial.print(",");
          Serial.print(rots.m3); Serial.print(",");
          Serial.print(ticks.m4); Serial.print(",");
          Serial.print(rots.m4); Serial.print(",");

          Serial.print(sensorData.amps3); Serial.print(",");
          Serial.print(sensorData.amps4); Serial.print(",");
          Serial.print(sensorData.scp1); Serial.print(",");

          Serial.print(duration * 1e-6, 3);
          Serial.println();
    */

    Serial.print(ticks.m3); Serial.print(",");
    Serial.print(rots.m3); Serial.print(",");
    Serial.print(ticks.m4); Serial.print(",");
    Serial.print(rots.m4); Serial.print(",");
    Serial.print(duration * 1e-6, 3);
    Serial.println();

  }

  //reached desired position, so stop
  STOP_Motors(MJ_BIN_m1);
  STOP_Motors(MJ_BIN_m2);

}

void trackingControl1(Encoder motor, int MJ_BIN[], float targetTicks) { //looking at an encoder object
  /*
     This function controls one motor to attain a desired rotations
     Question: move N number of ticks from where you are.
  */
  int ER_MARGIN = 20;
  float Err_m = motor.read() - targetTicks;

  while (abs(Err_m) > ER_MARGIN) {
    // define PID control gains
    float KP = 1;
    float KD = 1;
    float loopTime = 0.02;

    float temp_command1 = KP * Err_m + KD * (Err_m / loopTime); //looptime is 1/frequency

    // normalize to PWM using tanh
    float vel_PD = tanh(temp_command1) * 255;

    if (Err_m > ER_MARGIN) { //overshot target, reverse
      RVS_Motors(MJ_BIN, abs(vel_PD)); //200
    }

    if (Err_m < ER_MARGIN) { //havent reached target, continue
      FWD_Motors(MJ_BIN, abs(vel_PD)); //200
    }
    Err_m = motor.read() - targetTicks; //update error
  }

  //reached desired position, so stop
  STOP_Motors(MJ_BIN);
}

void forceControlPD(FsrScpData& sensData, float target_force, int MJ_BIN1[],
                    int MJ_BIN2[], float motorSpeed1, float motorSpeed2) {
  /*
     This function controls the speeds of two motors (of the same GR) that control one finger
     MotorFront is the motor actuating the flexion of the fingers
     MotorBack is the motor actuating the extension of the fingers
     target_force is the desired force for the trajectory
     sensData contains the current information of the FSRs
  */

  /*
     Currently assuming that the hand in flexion process
  */
  float err_fsr = sensData.fsr1 - target_force;
  float FORCE_ERR_MARGIN = 200; //millinewtons
  float loopTime = 0.02;
  float KP_F = 1;
  float KD_F = 1;

  while (abs(err_fsr) > FORCE_ERR_MARGIN) {
    float temp_command1 = KP_F * err_fsr + KD_F * (err_fsr / loopTime);

    float command_frt = (1 - tanh(temp_command1)) * motorSpeed1;
    float command_bck = (1 + tanh(temp_command1)) * motorSpeed2;

    // check if motor is running at max PWM
    if (motorSpeed1 == 255.0f || abs(command_frt) > 255.0f) {
      command_frt = 255 * sgn(command_frt);
    }
    if (motorSpeed2 == 255.0f || abs(command_bck) > 255.0f) {
      command_bck = 255 * sgn(command_bck);
    }

    //drive motors at computed speeds
    if (sgn(command_frt) == 1) {
      FWD_Motors(MJ_BIN1, abs(command_frt));
    } else {
      RVS_Motors(MJ_BIN1, abs(command_frt));
    }

    if (sgn(command_bck) == 1) {
      FWD_Motors(MJ_BIN2, abs(command_bck));
    } else {
      RVS_Motors(MJ_BIN2, abs(command_bck));
    }

    getSensorData(sensData);
    err_fsr = sensData.fsr1 - target_force;
  }
}

/*
  void computeFingerAngle(sensors_event_t& acc1, sensors_event_t& gyro1,
                        sensors_event_t& acc2, sensors_event_t& gyro2,
                        sensors_event_t& acc3, sensors_event_t& gyro3,
                        sensors_event_t& acc4, sensors_event_t& gyro4, fingAngles& fin_angles) {

  // update the filter, which computes orientation
  filter0.updateIMU(gyro1.gyro.x, gyro1.gyro.y, gyro1.gyro.z,
                    acc1.acceleration.x, acc1.acceleration.y, acc1.acceleration.z);
  filter1.updateIMU(gyro2.gyro.x, gyro2.gyro.y, gyro2.gyro.z,
                    acc2.acceleration.x, acc2.acceleration.y, acc2.acceleration.z);
  filter2.updateIMU(gyro3.gyro.x, gyro3.gyro.y, gyro3.gyro.z,
                    acc3.acceleration.x, acc3.acceleration.y, acc3.acceleration.z);
  filter3.updateIMU(gyro4.gyro.x, gyro4.gyro.y, gyro4.gyro.z,
                    acc4.acceleration.x, acc4.acceleration.y, acc4.acceleration.z);
  //compute angle
  float DegToRad = 3.14159265 / 180.0;
  float roll0 = filter0.getRoll() * DegToRad;
  float roll1 = filter1.getRoll() * DegToRad;
  float roll2 = filter2.getRoll() * DegToRad;
  float roll3 = filter3.getRoll() * DegToRad;

  fin_angles.a1 = roll1 - roll0;
  fin_angles.a2 = roll2 - roll1;
  fin_angles.a3 = roll3 - roll2;
  }
*/

void computeFingerAngle(Madgwick head, sensors_event_t& acc, sensors_event_t& gyr,
                        AnglesComps& BiasIMU, EulerAngIMU& IMUAngs) {

  // update the filter, which computes orientation. The axis needs to be flipped
  head.updateIMU((gyr.gyro.x - BiasIMU.gX), (gyr.gyro.y - BiasIMU.gY), (gyr.gyro.z - BiasIMU.gZ),
                 acc.acceleration.x, acc.acceleration.y, acc.acceleration.z);

  //compute inclination of IMU
  float DegToRad = 3.14159265 / 180.0;
  IMUAngs.roll = head.getRoll() * DegToRad;
  IMUAngs.pitch = head.getPitch() * DegToRad;
}

void computeFingerAngleCF(sensors_event_t& acc, sensors_event_t& gyr, AnglesComps& anglesHist, AnglesComps& BiasIMU) {
  float RADIANS_TO_DEGREES = 180 / 3.14159;
  float dt = anglesHist.d_time;

  //sgn(acc.acceleration.y) *,

  float aY_angle = atan2(1 * acc.acceleration.x, sgn(acc.acceleration.z) * sqrt(pow(acc.acceleration.y, 2) + pow(acc.acceleration.z, 2))) * RADIANS_TO_DEGREES; //roll
  float aX_angle = atan2(acc.acceleration.y, sgn(acc.acceleration.z) * sqrt(pow(acc.acceleration.x, 2) + pow(acc.acceleration.z, 2))) * RADIANS_TO_DEGREES; //pitch
  float aZ_angle = atan2(sqrt(pow(acc.acceleration.x, 2) + pow(acc.acceleration.y, 2)), acc.acceleration.z ) * RADIANS_TO_DEGREES; //yaw

  // Compute the (filtered) gyro angles
  //float dt = microsPerReading;
  float gX_angle = (gyr.gyro.x - BiasIMU.gX) * RADIANS_TO_DEGREES * dt + anglesHist.aX;
  float gY_angle = (gyr.gyro.y - BiasIMU.gY) * RADIANS_TO_DEGREES * dt + anglesHist.aY;
  float gZ_angle = (gyr.gyro.z - BiasIMU.gZ) * RADIANS_TO_DEGREES * dt + anglesHist.aZ;

  // Compute the drifting gyro angles
  float gXuf_angle = (gyr.gyro.x - BiasIMU.gX) * RADIANS_TO_DEGREES * dt + anglesHist.gX;
  float gYuf_angle = (gyr.gyro.y - BiasIMU.gY) * RADIANS_TO_DEGREES * dt + anglesHist.gY;
  float gZuf_angle = (gyr.gyro.z - BiasIMU.gZ) * RADIANS_TO_DEGREES * dt + anglesHist.gZ;

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.98;
  float angle_x = (1.0 - alpha) * gX_angle + alpha * aX_angle;
  float angle_y = (1.0 - alpha) * gY_angle + alpha * aY_angle;
  float angle_z = gZ_angle;

  //Store last readings and time
  anglesHist.aX = angle_x;
  anglesHist.aY = angle_y;
  anglesHist.aZ = angle_z;
  anglesHist.gX = gXuf_angle;
  anglesHist.gY = gYuf_angle;
  anglesHist.gZ = gZuf_angle;
}

void butControl(int pinNum) {
  const int MJ3_BIN1 = 33, MJ3_BIN2 = 12, MJ4_BIN1 = 36, MJ4_BIN2 = 37;
  int M_B[] = {MJ3_BIN1, MJ3_BIN2, MJ4_BIN1, MJ4_BIN2};
  int M_one[] = {MJ3_BIN1, MJ3_BIN2};
  int M_two[] = {MJ4_BIN1, MJ4_BIN2};

  if (digitalRead(pinNum)) { //high due to pullup
    STOP_Motors(M_B);
  }
  else { // pin is low due to pressed button
    FWD_Motors(M_one, 200.0); //using motors in J3, J4
    RVS_Motors(M_two, 200.0); //
  }
}

void butControlRVS(int pinNum) {
  const int MJ3_BIN1 = 33, MJ3_BIN2 = 12, MJ4_BIN1 = 36, MJ4_BIN2 = 37;
  int M_B[] = {MJ3_BIN1, MJ3_BIN2, MJ4_BIN1, MJ4_BIN2};
  int M_one[] = {MJ3_BIN1, MJ3_BIN2};
  int M_two[] = {MJ4_BIN1, MJ4_BIN2};

  if (digitalRead(pinNum)) { //high due to pullup
    STOP_Motors(M_B);
  }
  else { // pin is low due to pressed button
    RVS_Motors(M_one, 150.0); //using motors in J3, J4
    FWD_Motors(M_two, 150.0); //
  }
}
