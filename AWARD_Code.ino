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

//need to double check these for servocity motors, units in kg.mm/A
float Kt_10 = 1 / 0.82;
float Kt_15 = 1 / 0.56;
float Kt_30 = 1 / 0.31;
float Kt_50 = 1 / 0.19;

EncderCounts_t ticks;
encRevs_t rots;
float goalRots = 40; //92, 90, 20, 80
int gr = 30.0f; int cpr = 12.0;
long int goalTicks = goalRots * gr * cpr; // GR = 15;

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
FsrScpData_t sensorData;
state_t fingState;

//initialize for kinematics and dynamics
double M[9], C[9], B[9], G[3], finger[9];
double state_pos[3] = {.1, 1, 2};
double fin_length[3] = {3, 4, 5};

state_t desired_state;

//IMU needs
Adafruit_LSM6DS33 lsm6ds33;
#define SAMPLE_RATE 100.0 // in Hz
unsigned long millisPerReading, millisPrevious;

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

float roll, roll1, roll_last, pitch, pitch_last, yaw, yaw_last, yaw_diff;
sensors_event_t accel1, accel2, accel3, accel4, accel5, accel6, accel7, accel8, accel1_last;
sensors_event_t gyro1, gyro2, gyro3, gyro4, gyro5, gyro6, gyro7, gyro8, gyro1_last;
sensors_event_t temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp1_last;

fingAngles_t fin_angles;
AnglesComps_t anglesNew1, anglesNew2, anglesNew3, anglesNew6;
AnglesComps_t BiasIMU1, BiasIMU2, BiasIMU3, BiasIMU6;
EulerAngIMU_t IMUAngs1, IMUAngs2, IMUAngs3, IMUAngs6;

float start_time_2; float start_time;

// initialize variables to pace updates to correct rate
float dt = (1 / SAMPLE_RATE);

unsigned long t_now;
unsigned long t_dur;
unsigned long t_tic, t_toc, d_t_C; //timing calibrations

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

  millisPerReading = 1000 * dt; //delta_t in milliseconds.
  millisPrevious = millis();

  //calibrate IMUs
  //Serial.println("Calibrating IMUs...");
  /*
     Run calibration routine once. Values will be stored in the EEPROM or hardcorded for
     fast experiments. EEPROM allows for data to be stored even after the device
     resets
  */

  /*
    t_tic = micros();
    calibrateIMUs(lsm6ds33, 1, accel1, gyro1, temp1, BiasIMU1);
    calibrateIMUs(lsm6ds33, 2, accel2, gyro2, temp2, BiasIMU2);
    calibrateIMUs(lsm6ds33, 3, accel3, gyro3, temp3, BiasIMU3);
    calibrateIMUs(lsm6ds33, 6, accel6, gyro6, temp6, BiasIMU6);
    t_toc = micros();
    d_t_C = (t_toc - t_tic);
  */

  /*
    calibrateIMUs(lsm6ds33, 5, accel1, gyro1, temp1, BiasIMU1);
    calibrateIMUs(lsm6ds33, 8, accel2, gyro2, temp2, BiasIMU2);
  */

  //cast the values to the serial monitor and also save/write to the EEPROM

  /****
     THULANI = Need to double check and make sure it works.

    Need to divide by 4 because analog inputs range from
    0 to 1023 and each byte of the EEPROM can only hold a
    value from 0 to 255.

    int val = analogRead(0) / 4;

    Write the value to the appropriate byte of the EEPROM.
    these values will remain there when the board is
    turned off.

    EEPROM.write(addr, val);

    Advance to the next address, when at the end restart at the beginning.
    Rather than hard-coding the length, you should use the pre-provided length function.
    This will make your code portable to all AVR processors.

    addr = addr + 1;
    if(addr == EEPROM.length())
    addr = 0;
  ****/
  /*
    Serial.print(BiasIMU1.aX); Serial.print(",");
    Serial.print(BiasIMU1.aY); Serial.print(",");
    Serial.print(BiasIMU1.aZ); Serial.print(",");
    Serial.print(BiasIMU1.gX); Serial.print(",");
    Serial.print(BiasIMU1.gY); Serial.print(",");
    Serial.print(BiasIMU1.gZ); Serial.print(",");
    Serial.println();
    Serial.print(BiasIMU2.aX); Serial.print(",");
    Serial.print(BiasIMU2.aY); Serial.print(",");
    Serial.print(BiasIMU2.aZ); Serial.print(",");
    Serial.print(BiasIMU2.gX); Serial.print(",");
    Serial.print(BiasIMU2.gY); Serial.print(",");
    Serial.print(BiasIMU2.gZ); Serial.print(",");
    Serial.println();
    Serial.print(BiasIMU3.aX); Serial.print(",");
    Serial.print(BiasIMU3.aY); Serial.print(",");
    Serial.print(BiasIMU3.aZ); Serial.print(",");
    Serial.print(BiasIMU3.gX); Serial.print(",");
    Serial.print(BiasIMU3.gY); Serial.print(",");
    Serial.print(BiasIMU3.gZ); Serial.print(",");
    Serial.println();
    Serial.print(BiasIMU6.aX); Serial.print(",");
    Serial.print(BiasIMU6.aY); Serial.print(",");
    Serial.print(BiasIMU6.aZ); Serial.print(",");
    Serial.print(BiasIMU6.gX); Serial.print(",");
    Serial.print(BiasIMU6.gY); Serial.print(",");
    Serial.print(BiasIMU6.gZ); Serial.print(",");
    Serial.println();
    Serial.print(d_t_C, 3); Serial.print(",");
    Serial.println();
  */

  //Update the biam angles using the calibration from above.EEPROM or hard code
  BiasIMU1.aX = -2.39;  BiasIMU1.aY = -4.73;  BiasIMU1.aZ = -7.04;
  BiasIMU1.gX = 0.37; BiasIMU1.gY = -0.30;  BiasIMU1.gZ = -0.22;

  BiasIMU2.aX = -0.75;  BiasIMU2.aY = -3.02;  BiasIMU2.aZ = -7.87;
  BiasIMU2.gX = 0.28; BiasIMU2.gY = -0.30;  BiasIMU2.gZ = -0.33;

  BiasIMU3.aX = 2.19; BiasIMU3.aY = -2.25;  BiasIMU3.aZ = -8.04;
  BiasIMU3.gX = 1.38; BiasIMU3.gY = -1.06;  BiasIMU3.gZ = -0.58;

  BiasIMU6.aX = -1.28;  BiasIMU6.aY = -3.67;  BiasIMU6.aZ = -7.85;
  BiasIMU6.gX = 0.31; BiasIMU6.gY = -0.30;  BiasIMU6.gZ = -0.27;


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

  //finger position
  int posFingr = 1;

  t_dur = millis(); t_now = millis();

  // control
  trackingControlMM(ticks.m3, ticks.m4, m_FNT3, m_FNT4, goalTicks, t_dur, fingState, posFingr);
  trackingControlMM(ticks.m3, ticks.m4, m_FNT3, m_FNT4, 0, t_dur, fingState, posFingr);

}

void loop(void) {
  //butControl(pinNum); //CW for top motor (undo from 1 pos)
  //butControlRVS(pinNum); //CCW for top motor
  t_now = millis();
  if (t_now - millisPrevious >= millisPerReading) {

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

        Serial.print(anglesNew1.aX, 3); Serial.print(",");
        Serial.print(anglesNew2.aX, 3); Serial.print(",");
        Serial.print(t_now * 1e-6, 3); //48
        Serial.println();

        //dataFile.close();

        //}
    */
    //Serial.println("Reading IMUs...");
    //Continously read data
    readIMUData(lsm6ds33, 1, accel1, gyro1, temp1, BiasIMU1);
    readIMUData(lsm6ds33, 2, accel2, gyro2, temp2, BiasIMU2);
    readIMUData(lsm6ds33, 3, accel3, gyro3, temp3, BiasIMU3);
    readIMUData(lsm6ds33, 6, accel6, gyro6, temp6, BiasIMU6);

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

    computeFingerAngleCF(accel1, gyro1, anglesNew1, BiasIMU1);
    computeFingerAngleCF(accel2, gyro2, anglesNew2, BiasIMU2);
    computeFingerAngleCF(accel3, gyro3, anglesNew3, BiasIMU3);
    computeFingerAngleCF(accel6, gyro6, anglesNew6, BiasIMU6);

    /*
        // control
        Serial.print("Target of 30"); Serial.println();
        trackingControlMM(ticks.m3, ticks.m4, m_FNT3, m_FNT4, goalTicks, t_now);
        Serial.print("Target of 0"); Serial.println();
        trackingControlMM(ticks.m3, ticks.m4, m_FNT3, m_FNT4, 0, t_now); */

    //tsaPosControl(2.1068, 0.015, 16.5, m_FNT3, m_FNT4, t_now); //units have to match

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

          dataFile.print(anglesNew1.aX, 3); dataFile.print(",");
          dataFile.print(anglesNew2.aX, 3); dataFile.print(",");
          dataFile.print(anglesNew3.aX, 3); dataFile.print(",");
          dataFile.print(anglesNew6.aX, 3); dataFile.print(",");

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

          Serial.print(anglesNew1.aX, 3); Serial.print(",");
          Serial.print(anglesNew2.aX, 3); Serial.print(",");
          Serial.print(anglesNew3.aX, 3); Serial.print(",");
          Serial.print(anglesNew6.aX, 3); Serial.print(","); */

    //for debugging only
    /*
        Serial.print(ticks.m3); Serial.print(",");
        Serial.print(rots.m3); Serial.print(",");
        Serial.print(ticks.m4); Serial.print(",");
        Serial.print(rots.m4); Serial.print(",");

        Serial.print(sensorData.amps3); Serial.print(",");
        Serial.print(sensorData.amps4); Serial.print(",");

        Serial.println();
    */

    /*
      Serial.print(sensorData.amps3); Serial.print(",");
      Serial.print(sensorData.amps4); Serial.print(",");
      Serial.print(sensorData.scp1); Serial.print(",");

      Serial.print(t_now * 1e-6, 3);
      Serial.println();
      } */
  }

  // increment previous time, so we keep proper pace
  millisPrevious = millisPrevious + millisPerReading;

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


void getEncCounts(EncderCounts_t& mTicks, encRevs_t& mRots) {
  /*
     Assumes that motors are of the same gear ratio. If different,
     this needs to be updated.

  */

  const float gr = 30.0; // confirm with motors being used
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

void getSensorData(FsrScpData_t& data) { // Read the data from the ADS 1115.

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
  long scp1 = ads2.readADC_SingleEnded(2);
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

  //convert Voltage to force
  data.scp1 = Voltage2Force(scp1);
}

void tsaPosControl(float travelGoal, float radius, float L0, int MJ_BIN_m1[], int MJ_BIN_m2[], unsigned long duration, state_t& fingJoints, int initPos) {

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
  int GR = 30.0f; int CPR = 12.0;
  long int targetTicks = needRots * CPR * GR;
  //position control using the computed rotations
  trackingControlMM(ticks.m3, ticks.m4, MJ_BIN_m1, MJ_BIN_m2, targetTicks, duration, fingJoints, initPos);
}


void trackingControlMM(long m1_ticks, long m2_ticks, int MJ_BIN_m1[], int MJ_BIN_m2[],
                       long int targetTicks, unsigned long duration, state_t& fingJoints, int initPos) {
  /*
     This function controls two antagonistic motors to attain a desired position of a finger
     In this case, it is assumed that the motors are of the same gear ratio, hence speed
     If the motors are of the different gear ratios, the faster motor needs to be slowed down
     by scaling the computed PWM. The scaling factor will be determined experimentally

     It is assmued that the motor for M3 is winding clockwise to form form
     Hardcode this into the state/state of the driving function
  */

  //initialize time
  unsigned long startTime, endTime;
  double timeChange;

  int ER_MARGIN = 1000;
  long int err_m1 = abs(ticks.m3) - targetTicks;// m1_ticks
  long int err_m2 = abs(ticks.m4) - targetTicks; //m2_ticks
  long int Err_m = 0;

  //get the direction flags
  int m1_flg = sgn(ticks.m3);
  int m2_flg = sgn(ticks.m4);

  int m1_flg_new = m1_flg;
  int m2_flg_new = m2_flg;

  if (err_m1 > err_m2) {
    Err_m = err_m1;
  } else {
    Err_m = err_m2;
  }

  // define PID control gains
  float KP = 0.0003;
  float KD = 0;
  double loopTime = 0.02; //

  // initialize position, velocity and acceleration
  float theta_0 = 0.0f; float theta_1 = 0.0f;
  float theta_2 = 0.0f; float theta_3 = 0.0f;

  //initialize filtered angles
  float theta_0f = 0.0f; float theta_1f = 0.0f;
  float theta_2f = 0.0f; float theta_3f = 0.0f;

  float angl_1 = 0.0f; float angl_2 = 0.0f; float angl_3 = 0.0f;
  float angl_1_old = 0.0f; float angl_2_old = 0.0f; float angl_3_old = 0.0f;

  float dot_angl_1 = 0.0f; float dot_angl_2 = 0.0f; float dot_angl_3 = 0.0f;
  float dot_angl_1_old = 0.0f; float dot_angl_2_old = 0.0f; float dot_angl_3_old = 0.0f;

  float dDot_angl_1 = 0.0f; float dDot_angl_2 = 0.0f; float dDot_angl_3 = 0.0f;
  float dDot_angl_1_old = 0.0f; float dDot_angl_2_old = 0.0f; float dDot_angl_3_old = 0.0f;
  unsigned long nowTime;

  while (abs(Err_m) > ER_MARGIN) {
    startTime = micros();
    double temp_command1 = KP * Err_m + KD * (Err_m / loopTime); //looptime is 1/frequency

    // normalize to PWM using tanh
    double vel_PD = tanh(temp_command1) * 255;
    /*
        if (abs(vel_PD) > 255 * 0.5) { //cap max velocity @ 50%
          vel_PD = 0.5 * vel_PD;
        }*/

    if (abs(vel_PD) < 50) { //minimum PWM to overcome friction
      vel_PD  = 50;
    }

    if (initPos == 0) {//finger is starting at the folded state
      if (abs(Err_m) > ER_MARGIN) { //may have overshot target
        if (sgn(Err_m) > 0) { //err was increasing, hence overshot, reverse
          FWD_Motors(MJ_BIN_m2, abs(vel_PD)); //motors move opposite each other
          RVS_Motors(MJ_BIN_m1, abs(vel_PD));
        }
        else {//err was negative, continue
          RVS_Motors(MJ_BIN_m2, abs(vel_PD)); //motors move opposite each other
          FWD_Motors(MJ_BIN_m1, abs(vel_PD));
        }
      }

      if (abs(Err_m) < ER_MARGIN) { //may not have reached target
        if (sgn(Err_m) > 0) { // err was positive, increasing -- continue
          RVS_Motors(MJ_BIN_m2, abs(vel_PD));
          FWD_Motors(MJ_BIN_m1, abs(vel_PD));
        }
        else { //err was negative reverse
          FWD_Motors(MJ_BIN_m2, abs(vel_PD));
          RVS_Motors(MJ_BIN_m1, abs(vel_PD));
        }
      }
    }

    if (initPos == 1) { // finger is starting at the straight configuration
      if (abs(Err_m) > ER_MARGIN) { //may have overshot target
        if (sgn(Err_m) > 0) { //err was increasing, hence overshot, reverse
          FWD_Motors(MJ_BIN_m1, abs(vel_PD)); //motors move opposite each other
          RVS_Motors(MJ_BIN_m2, abs(vel_PD));
        }
        else {//err was negative, continue
          RVS_Motors(MJ_BIN_m1, abs(vel_PD)); //motors move opposite each other
          FWD_Motors(MJ_BIN_m2, abs(vel_PD));
        }
      }

      if (abs(Err_m) < ER_MARGIN) { //may not have reached target
        if (sgn(Err_m) > 0) { // err was positive, increasing -- continue
          RVS_Motors(MJ_BIN_m1, abs(vel_PD));
          FWD_Motors(MJ_BIN_m2, abs(vel_PD));
        }
        else { //err was negative reverse
          FWD_Motors(MJ_BIN_m1, abs(vel_PD));
          RVS_Motors(MJ_BIN_m2, abs(vel_PD));
        }
      }
    }

    //keep data from previous set. Only care about angles
    angl_1_old = fingJoints.pos.x;
    angl_2_old = fingJoints.pos.y;
    angl_3_old = fingJoints.pos.z;

    dot_angl_1_old = fingJoints.vel.x;
    dot_angl_2_old = fingJoints.vel.y;
    dot_angl_3_old = fingJoints.vel.z;

    dDot_angl_1_old = fingJoints.acc.x;
    dDot_angl_2_old = fingJoints.acc.y;
    dDot_angl_3_old = fingJoints.acc.z;

    // read sensor data
    readIMUData(lsm6ds33, 1, accel1, gyro1, temp1, BiasIMU1);
    readIMUData(lsm6ds33, 2, accel2, gyro2, temp2, BiasIMU2);
    readIMUData(lsm6ds33, 3, accel3, gyro3, temp3, BiasIMU3);
    readIMUData(lsm6ds33, 6, accel6, gyro6, temp6, BiasIMU6);

    //compute angles using updated sensor info
    computeFingerAngleCF(accel1, gyro1, anglesNew1, BiasIMU1);
    computeFingerAngleCF(accel2, gyro2, anglesNew2, BiasIMU2);
    computeFingerAngleCF(accel3, gyro3, anglesNew3, BiasIMU3);
    computeFingerAngleCF(accel6, gyro6, anglesNew6, BiasIMU6);

    //get needed angles and compute derivatives
    theta_0 = anglesNew1.aX + 180;
    theta_1 = anglesNew2.aX + 180;
    theta_2 = anglesNew3.aX + 180;
    theta_3 = anglesNew6.aX + 180;

    //compute joint angles
    fingJoints.pos.x = theta_1 - theta_0;
    fingJoints.pos.y = theta_2 - theta_1;
    fingJoints.pos.z = theta_3 - theta_2;

    endTime = micros();
    timeChange = (endTime - startTime) * 1e-6;// dt is in seconds
    loopTime = timeChange;

    dot_angl_1 = (fingJoints.pos.x - angl_1_old) / timeChange;
    dot_angl_2 = (fingJoints.pos.y - angl_2_old) / timeChange;
    dot_angl_3= (fingJoints.pos.z - angl_3_old) / timeChange;

    fingJoints.vel.x = 0.8*dot_angl_1_old + 0.0155*dot_angl_1;
    fingJoints.vel.y = 0.8*dot_angl_2_old + 0.0155*dot_angl_2;
    fingJoints.vel.z = 0.8*dot_angl_3_old + 0.0155*dot_angl_3;

    dDot_angl_1 = (fingJoints.vel.x - dot_angl_1_old) / timeChange;
    dDot_angl_2 = (fingJoints.vel.y - dot_angl_2_old) / timeChange;
    dDot_angl_3 = (fingJoints.vel.z - dot_angl_3_old) / timeChange;

    fingJoints.acc.x = 0.8*dDot_angl_1_old + 0.0155*dDot_angl_1;
    fingJoints.acc.y = 0.8*dDot_angl_2_old + 0.0155*dDot_angl_2;
    fingJoints.acc.z = 0.8*dDot_angl_3_old + 0.0155*dDot_angl_3;

    duration = millis();

    getEncCounts(ticks, rots);
    getSensorData(sensorData);

    // we know where these motors are connected for their encoders
    err_m1 = abs(ticks.m3) - targetTicks;
    err_m2 = abs(ticks.m4) - targetTicks;

    //get the flags
    m1_flg_new = sgn(ticks.m3);
    m2_flg_new = sgn(ticks.m4);

    if (abs(err_m1) > abs(err_m2)) {
      Err_m = err_m1;
    } else {
      Err_m = err_m2;
    }

    /*
       check for change of direction of the ticks. This allows for controlling motor
       rotations regardless of the initial direction of the motors
    */

    if ((m1_flg_new != m1_flg)  || (m2_flg_new != m2_flg) ) { //
      if (abs(Err_m) < ER_MARGIN) { //
        Err_m = -1 * Err_m;
      }
    }

    //print data/write data to SD card
/*
    File dataFile = SD.open("datalog2.txt", FILE_WRITE);  // open the file.
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

      dataFile.print(anglesNew1.aX, 3); dataFile.print(",");
      dataFile.print(anglesNew2.aX, 3); dataFile.print(",");
      dataFile.print(anglesNew3.aX, 3); dataFile.print(",");
      dataFile.print(anglesNew6.aX, 3); dataFile.print(",");

      dataFile.print(ticks.m3); dataFile.print(",");
      dataFile.print(rots.m3); dataFile.print(",");
      dataFile.print(ticks.m4); dataFile.print(",");
      dataFile.print(rots.m4); dataFile.print(",");

      dataFile.print(sensorData.amps3); dataFile.print(",");
      dataFile.print(sensorData.amps4); dataFile.print(",");
      dataFile.print(sensorData.scp1); dataFile.print(",");

      dataFile.print(duration * 1e-3, 3); dataFile.print(",");

      dataFile.print(theta_0); dataFile.print(",");
      dataFile.print(theta_1); dataFile.print(",");
      dataFile.print(theta_2); dataFile.print(",");
      dataFile.print(theta_3); dataFile.print(",");

      dataFile.print(theta_0f); dataFile.print(",");
      dataFile.print(theta_1f); dataFile.print(",");
      dataFile.print(theta_2f); dataFile.print(",");
      dataFile.print(theta_3f); dataFile.print(",");

      dataFile.print(fingJoints.pos.x); dataFile.print(",");
      dataFile.print(fingJoints.pos.y); dataFile.print(",");
      dataFile.print(fingJoints.pos.z); dataFile.print(",");

      dataFile.print(fingJoints.vel.x); dataFile.print(",");
      dataFile.print(fingJoints.vel.y); dataFile.print(",");
      dataFile.print(fingJoints.vel.z); dataFile.print(",");

      dataFile.print(fingJoints.acc.x); dataFile.print(",");
      dataFile.print(fingJoints.acc.y); dataFile.print(",");
      dataFile.print(fingJoints.acc.z); dataFile.print(",");

      dataFile.println();
      dataFile.close();
    }
    */

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
    Serial.print(gyro6.gyro.z); Serial.print(",");//24

    Serial.print(anglesNew1.aX, 3); Serial.print(",");
    Serial.print(anglesNew2.aX, 3); Serial.print(",");
    Serial.print(anglesNew3.aX, 3); Serial.print(",");
    Serial.print(anglesNew6.aX, 3); Serial.print(",");

    Serial.print(ticks.m3); Serial.print(",");
    Serial.print(rots.m3); Serial.print(",");
    Serial.print(ticks.m4); Serial.print(",");
    Serial.print(rots.m4); Serial.print(","); 
 
    Serial.print(sensorData.amps3); Serial.print(",");
    Serial.print(sensorData.amps4); Serial.print(",");
    Serial.print(sensorData.scp1); Serial.print(",");

    Serial.print(duration * 1e-3, 3); Serial.print(",");//36

    Serial.print(theta_0); Serial.print(",");
    Serial.print(theta_1); Serial.print(",");
    Serial.print(theta_2); Serial.print(",");
    Serial.print(theta_3); Serial.print(",");

    Serial.print(theta_0f); Serial.print(",");
    Serial.print(theta_1f); Serial.print(",");
    Serial.print(theta_2f); Serial.print(",");
    Serial.print(theta_3f); Serial.print(",");

    Serial.print(fingJoints.pos.x); Serial.print(","); //41
    Serial.print(fingJoints.pos.y); Serial.print(",");
    Serial.print(fingJoints.pos.z); Serial.print(",");

    Serial.print(fingJoints.vel.x); Serial.print(",");
    Serial.print(fingJoints.vel.y); Serial.print(",");
    Serial.print(fingJoints.vel.z); Serial.print(",");

    Serial.print(fingJoints.acc.x); Serial.print(",");
    Serial.print(fingJoints.acc.y); Serial.print(","); //remove comma and timeChange after debugging
    Serial.print(fingJoints.acc.z); Serial.print(",");
    Serial.print(timeChange);

    Serial.println(); 
   /*
        Serial.print(ticks.m3); Serial.print(",");
        Serial.print(rots.m3); Serial.print(",");
        Serial.print(ticks.m4); Serial.print(",");
        Serial.print(rots.m4); Serial.print(",");
        Serial.print(err_m1); Serial.print(",");
        Serial.print(err_m2); Serial.print(",");
        Serial.print(Err_m); Serial.print(",");
        Serial.print(targetTicks / (12 * 30)); Serial.print(",");
        Serial.print(duration * 1e-3, 3);
        Serial.println(); */
    
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

void forceControlPD(FsrScpData_t& sensData, float target_force, int MJ_BIN1[],
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

void computeFingerAngleCF(sensors_event_t& acc, sensors_event_t& gyr, AnglesComps_t& anglesHist, AnglesComps_t& BiasIMU) {
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
    RVS_Motors(M_two, 150.0); //using motors in J3, J4
    RVS_Motors(M_one, 150.0); //
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
    /*RVS_Motors(M_two, 150.0); //using motors in J3, J4
      FWD_Motors(M_one, 150.0); // opposite direction
    */

    FWD_Motors(M_two, 150.0); //using motors in J3, J4
    FWD_Motors(M_one, 150.0); // opposite direction
  }
}

void computedTorqueController (double state_pos[3], double state_vel[3], double torque,
                               state_t& theta_d, float currTime, trq_t tau_comp,
                               double M[9], double C[9], double B[9], double G[3]) {

  /*
     state_pos is the current joint angles of the fingers. specifically [JointAngles.theta_1, JointAngles.theta_2, JointAngles.theta_3];
     state_vel is the current joint speeds. differentiate state_pos above
     theta_d holds the desired position and velocity and accelaration eg. theta_d.pos.x
     M, C, B, G are defined as global variables hence can be accessed outside the scope of this function
     Can be re-inialized here if they are to be defined as local variables.
  */

  dynamics(state_pos, fin_length, M, C, B, G); //update M,C,B,G given state_pos and fin_length

  //create appropriate dimensions for matrices

  tmm::Scalar M_1[3][3] = {
    {M[0], M[1], M[2]},
    {M[3], M[4], M[5]},
    {M[6], M[7], M[8]}
  };

  tmm::SquareMatrix<3> M_m(M_1);

  tmm::Scalar C_1[3][3] = {
    {C[0], C[1], C[2]},
    {C[3], C[4], C[5]},
    {C[6], C[7], C[8]}
  };

  tmm::SquareMatrix<3> C_m(C_1);

  tmm::Scalar B_1[3][3] = {
    {B[0], B[1], B[2]},
    {B[3], B[4], B[5]},
    {B[6], B[7], B[8]}
  };

  tmm::SquareMatrix<3> B_m(B_1);

  tmm::Scalar G_1[3][1] = {
    {G[0]},
    {G[1]},
    {G[2]}
  };

  tmm::Matrix<3, 1> G_m(G_1);

  /*
     please note that x=theta_1, y=theta_2,z = theta_3
  */

  //------------------------------------//
  const tmm::Scalar K_p[3][3] = {
    {55, 0, 0},
    {0, 55, 0},
    {0, 0, 55}
  };
  tmm::SquareMatrix<3> Kp(K_p);

  const tmm::Scalar K_v[3][3] = {
    {15, 0, 0},
    {0, 15, 0},
    {0, 0, 15}
  };
  tmm::SquareMatrix<3> Kv(K_v);

  float theta_err_1 = theta_d.pos.x - state_pos[0];
  float theta_err_2 = theta_d.pos.y - state_pos[1];
  float theta_err_3 = theta_d.pos.z - state_pos[2];

  float theta_err_dot_1 = theta_d.vel.x - state_vel[0];
  float theta_err_dot_2 = theta_d.vel.y - state_vel[1];
  float theta_err_dot_3 = theta_d.vel.z - state_vel[2];

  tmm::Scalar err_1[3][1] = {
    {theta_err_1},
    {theta_err_1},
    {theta_err_1}
  };

  tmm::Matrix<3, 1> err(err_1);

  tmm::Scalar err_dot_1[3][1] = {
    {theta_err_dot_1},
    {theta_err_dot_2},
    {theta_err_dot_3}
  };

  tmm::Matrix<3, 1> err_dot(err_dot_1);

  tmm::Scalar acc_des_1[3][1] = {
    {theta_d.acc.x},
    {theta_d.acc.y},
    {theta_d.acc.z}
  };

  tmm::Matrix<3, 1> acc_des(acc_des_1);

  //  float u = -1 * Kv * err_dot - Kp * err;

  tmm::Matrix<3, 1> U = Kv * -1 * err_dot - Kp * err;

  tmm::Matrix<3, 1> tau_c = M_m * (acc_des - U) + G_m;

  tau_comp.tau_1 = tau_c[0][1];
  tau_comp.tau_2 = tau_c[1][1];
  tau_comp.tau_3 = tau_c[2][1];



  /*
     note that joints cannot generate infinitely large torques or forces. Need to set a torque_limit
  */

}

void computeJointAngles(jntAngl_t& JointAngles, AnglesComps_t& Angle1, AnglesComps_t& Angle2, AnglesComps_t& Angle3, AnglesComps_t& Angle4 ) {
  /*
     Computes the actual finger joints from the measured four IMU angles of each finger
  */

  float theta_1 = Angle1.aX + 180;
  float theta_2 = Angle2.aX + 180;
  float theta_3 = Angle3.aX + 180;
  float theta_4 = Angle4.aX + 180;

  JointAngles.theta_1 = theta_2 - theta_1;
  JointAngles.theta_2 = theta_3 - theta_2;
  JointAngles.theta_3 = theta_4 - theta_3;

}
