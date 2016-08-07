//  ||||||||||||||||||||||||||||||||||||||||||||||||
//  ||  Vectored Thrust Quadrotor Flight Control  ||
//  ||||||||||||||||||||||||||||||||||||||||||||||||
//
//  Built to run on a arduino MEGA2560 with a MPU6050
//  sensor and motion processor.
//
//  Library uses I2Cdev device library and DMP Motion
//  apps v2.0 by Jeff Rowberg.
//  https://github.com/jrowberg/i2cdevlib
//  <jeff@rowberg.net>
//
//  Author: Ignacio Maldonado
//  Last Update: July 20th, 2016

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// ================================================================
//              INCLUDES
// ================================================================
#include <math.h>
#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "MPU6050_6Axis_MotionApps20.h"

// ================================================================
//              DEFINITIONS AND GLOBAL VARIABLE SETUP
// ================================================================

#define SERVO_LIMIT_HIGH 250
#define SERVO_LIMIT_LOW (-SERVO_LIMIT_HIGH)
#define SERVO_NULL 1500
#define ESC_LIMIT_HIGH 2400
#define ESC_LIMIT_LOW 600
#define ESC_RANGE (ESC_LIMIT_HIGH - ESC_LIMIT_LOW)

#define SLIDE_INTEGRAL_LIMIT_HIGH 2000
#define SLIDE_INTEGRAL_LIMIT_LOW (-SLIDE_INTEGRAL_LIMIT_HIGH)
#define SPIN_INTEGRAL_LIMIT_HIGH 100
#define SPIN_INTEGRAL_LIMIT_LOW (-SPIN_INTEGRAL_LIMIT_HIGH)

#define STATE_VARS 12
#define SLIDE_VAR_FIRST 0
#define SLIDE_VAR_LAST 6
#define SPIN_VAR_FIRST 6
#define SPIN_VAR_LAST 12

#define SERVO_FIRST 0
#define SERVO_LAST 4
#define ESC_FIRST 4
#define ESC_LAST 8
#define OUTPUTS 8

#define SENS_SAT 1500
#define MOTORSTART 600

MPU6050 mpu;  //MPU struct init

/**************************
  ACTUATOR INSTANTIATION
***************************/

Servo ACTUATORS[OUTPUTS];

// MPU control/status vars

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
float euler[3];         // [psi, theta, phi]    Euler angle container
VectorInt16 ypr;        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float asdf[3];

/***********************
   CONTROL PARAMETERS
***********************/

/* * * * * * * * * * * * * * * * * * * * * * * * * *
 * Matrices an control parameters calculated       *
 * using the following topology:                   *
 *                                                 *
 *      (1)                             (2)        *
 *        \\                           //          *
 *          \\                       //            *
 *        ^   \\                   //              *
 *       +X     \\               //                *
 *        |       \\   Front   //                  *
 *        |        |===========|                   *
 * <- +Y -O        |           |      0-Pitch-->   *
 *          +Z     |    Top    |      |            *
 *                 |           |     Roll          *
 *                 |           |      |            *
 *                 |===========|      V            *
 *                //           \\                  *          
 *              //               \\                *       
 *            //                   \\              *
 *          //                       \\            *   
 *        //                           \\          * 
 *      (4)                             (3)        * 
 *                                                 *
 *  [Top-Down View of quadcopter]                  *
 *                                                 *
 * * * * * * * * * * * * * * * * * * * * * * * * * */

//slide matrix        X   dX      Y    dY      Z     dZ     ||  Roll   dRoll      Pitch   dPitch   Yaw  dYaw
float Kain[8][12] = {{0, -1.5293, 0,  -1.5293, 0,     0,         0,      0,         0,      0,      0,   1.2455}, // Servo 1
                     {0,  1.5293, 0,  -1.5293, 0,     0,         0,      0,         0,      0,      0,   1.2455}, //       2
                     {0, -1.5293, 0,   1.5293, 0,     0,         0,      0,         0,      0,      0,   1.2455}, //       3
                     {0,  1.5293, 0,   1.5293, 0,     0,         0,      0,         0,      0,      0,   1.2455}, //       4
                     {0,  0,      0,   0,      1.158, 1.3017, -100.158, -5.0485, -100.158,  5.0485, 0,   0},      // ESC   1
                     {0,  0,      0,   0,      1.158, 1.3017,  100.158,  5.0485, -100.158,  5.0485, 0,   0},      // ESC   2
                     {0,  0,      0,   0,      1.158, 1.3017,  100.158,  5.0485,  100.158, -5.0485, 0,   0},      // ESC   3
                     {0,  0,      0,   0,      1.158, 1.3017, -100.158, -5.0485,  100.158, -5.0485, 0,   0}       // ESC   4
};

//state variables
float states[STATE_VARS];
float outs[OUTPUTS];
float out[OUTPUTS];
float theta_x = 0;
float theta_y = 0;
float accel_x;
float accel_y;
float accel_z;

// offset variables
float rolloffset = 0;
float pitchoffset = 0;
float axbias = 0;
float aybias = 0;
float azbias = 0;

//proportional error bins
float err[STATE_VARS] = {0, //     X
                         0, // Vel X
                         0, //     Y
                         0, // Vel Y
                         0, //     Z
                         0, // Vel Z
                         0, // Roll
                         0, // Roll Rate
                         0, // Pitch
                         0, // Pitch Rate
                         0, // Yaw
                         0  // Yaw Rate
                        };

//integral error bins
float errint[STATE_VARS] = {0, //     X
                            0, // Vel X
                            0, //     Y
                            0, // Vel Y
                            0, //     Z
                            0, // Vel Z
                            0, // Roll
                            0, // Roll Rate
                            0, // Pitch
                            0, // Pitch Rate
                            0, // Yaw
                            0  // Yaw Rate
                           };

//gain variable for integral control
float intcntrl[STATE_VARS] = {0,   //     X
                              0,   // Vel X
                              0,   //     Y
                              0,   // Vel Y
                              50,   //     Z <================INTEGRAL CONTROL TOGGLE=================================================================================
                              0,   // Vel Z
                              0.05, // Roll
                              0,   // Roll Rate
                              0.05, // Pitch
                              0,   // Pitch Rate
                              0,   // Yaw
                              0    // Yaw Rate
                             };

//reference "signals"
float ref[STATE_VARS] = {0, //     X
                         0, // Vel X  
                         0, //     Y
                         0, // Vel Y
                         1, //     Z
                         0, // Vel Z
                         0, // Roll
                         0, // Roll Rate
                         0, // Pitch
                         0, // Pitch Rate
                         0, // Yaw
                         0  // Yaw Rate
                        };

uint32_t timer;
int kaka = 0;
/***********************
   INTERRUPT ROUTINE
***********************/

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
//                      SETUP AND CALIBRATION
// ================================================================

void setup() {

  delay(10000);

  /***********************
       ACTUATOR INIT
  ***********************/

  ACTUATORS[0].attach(6);  // SERVO 1
  ACTUATORS[1].attach(7);  //       2
  ACTUATORS[2].attach(3);  //       3
  ACTUATORS[3].attach(4);  //       4
  
  ACTUATORS[4].attach(9);  // ESC   1
  ACTUATORS[5].attach(8);  //       2
  ACTUATORS[6].attach(11); //       3
  ACTUATORS[7].attach(10); //       4
  
  /***********************
     ESC CALIBRATION
  ***********************/
  for (int i = ESC_FIRST; i < ESC_LAST; ACTUATORS[i++].writeMicroseconds(ESC_LIMIT_HIGH));

  delay(3000);

  for (int i = ESC_FIRST; i < ESC_LAST; ACTUATORS[i++].writeMicroseconds(ESC_LIMIT_LOW));
  
  delay(1000);

  Wire.begin(); //begin serial comms

  mpu.initialize();  // initialize device
  Serial.begin(250000);

  devStatus = mpu.dmpInitialize();
  mpu.setFullScaleGyroRange(0);

  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  /***********************
      ACTUATOR ZEROING
  ***********************/

  for (int i = 0; i < OUTPUTS ; out[i++] = 0);

  for (int i = SERVO_FIRST; i < SERVO_LAST; ACTUATORS[i++].writeMicroseconds(SERVO_NULL));
  
  for (int i = ESC_FIRST; i < ESC_LAST; ACTUATORS[i++].writeMicroseconds(ESC_LIMIT_LOW));

  /***********************
        GYRO OFFSET
  ***********************/
  int gxoffset = 55;
  int gyoffset = 1;
  int gzoffset = -3;
  
  for (int i = 0; i < 500; i++)
  {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02)
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      delay(1);
      fifoCount -= packetSize;
      mpu.dmpGetGyro( &ypr , fifoBuffer);
      delay(1);
      
      if (ypr.x < 0)
      {
        ++gxoffset;
      }
      else
      {
        --gxoffset;
      }

      if (ypr.y < 0)
      {
        ++gyoffset;
      }
      else
      {
        --gyoffset;
      }
      if (ypr.z < 0)
      {
        ++gzoffset;
      }
      else
      {
        --gzoffset;
      }
      
      mpu.setXGyroOffset(gxoffset);
      mpu.setYGyroOffset(gyoffset);
      mpu.setZGyroOffset(gzoffset);

//      Serial.println(i);
    }
  }

  /***********************
        ACCEL OFFSET
  ***********************/
  int axoffset = -1984;
  int ayoffset = -5752;
  int azoffset = 60;

  for (int i = 0; i < 500; i++)
  {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02)
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      delay(1);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      delay(1);
      mpu.dmpGetAccel(&aa, fifoBuffer);

      accel_x = (aa.x) / 1;
      accel_y = (aa.y) / 1;
      accel_z = (aa.z) / 1;

      if (accel_x < -0)
      {
        ++axoffset;
      }
      else if (accel_x > 10)
      {
        --axoffset;
      }

      if (accel_y < -0)
      {
        ++ayoffset;
      }
      else if (accel_y > 10)
      {
        --ayoffset;
      }
      if (accel_z < 20)
      {
        ++azoffset;
      }
      else if (accel_z > 30)
      {
        --azoffset;
      }
      
      mpu.setXAccelOffset(axoffset);
      mpu.setYAccelOffset(ayoffset);
      mpu.setZAccelOffset(azoffset);

    }
  }


    for (int i = 0; i < 1000; i++)
  {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02)
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      delay(2);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      delay(2);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      
      float Ts = (float)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();

      float low_pass = Ts / (5 + Ts);
      
      if (abs(aa.x) < SENS_SAT && abs(aa.y) < SENS_SAT && abs(aa.z) < SENS_SAT)
      {    
      axbias = ((1 - low_pass) * axbias + low_pass * (float)aa.x);
      aybias = ((1 - low_pass) * aybias + low_pass * (float)aa.y);
      }
      
//      Serial.print(axbias);
//      Serial.print(" ");
//      Serial.println(aybias);
    }
  }

  /***********************
       ANGLE OFFSET
  ***********************/
  
  timer = micros();
  
  for (int i = 0; i < 1000; i++)
  {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02)
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      delay(2);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      delay(2);
      float Ts = (float)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();
      
      float low_pass = Ts / (1 + Ts);
      
      if (abs(aa.x) < SENS_SAT && abs(aa.y) < SENS_SAT && abs(aa.z) < SENS_SAT)
      {
      axbias = ((1 - low_pass) * axbias + low_pass * (float)aa.x);
      aybias = ((1 - low_pass) * aybias + low_pass * (float)aa.y);

      accel_x = ((1 - low_pass) * accel_x + low_pass * ((float)aa.x - axbias)) / 1.1;
      accel_y = ((1 - low_pass) * accel_y + low_pass * ((float)aa.y - aybias)) / 1.1;
      accel_z = ((1 - low_pass) * accel_z + low_pass *  (float)aa.z) / 1;
      }
      
      float g_norm = sqrt(powf(accel_x,2) + powf(accel_y,2) + powf(accel_z,2));
    
      VectorFloat gravity;
    
      gravity.x = (accel_x)/g_norm;
      gravity.y = (accel_y)/g_norm;
      gravity.z = (accel_z)/g_norm;

      mpu.dmpGetYawPitchRoll(asdf, &q, &gravity);

      low_pass = Ts / (1 + Ts);
      
      rolloffset = ((1 - low_pass) * rolloffset + low_pass * (asdf[2]));
      pitchoffset = ((1 - low_pass) * pitchoffset + low_pass * (asdf[1]));

//      Serial.print(rolloffset*(180/3.14));
//      Serial.print(" ");
//      Serial.println(pitchoffset*(180/3.14));
    }
  }

  timer = micros();
}

//================================================================
//                   MAIN LOOP
//================================================================
void loop() {
  kaka++;
  /***********************
      GET MEASUREMENTS
  ***********************/

  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    delay(5);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    delay(5);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    delay(5);
    mpu.dmpGetGyro( &ypr , fifoBuffer);
    delay(5);
    
    /**********************************
        MEASUREMENT PREPROCESSING
    ***********************************/

    float Ts = (float)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    float low_pass = Ts / (10 + Ts);
    
    if (abs(aa.x) < SENS_SAT && abs(aa.y) < SENS_SAT && abs(aa.z) < SENS_SAT)
    {
    axbias = ((1 - low_pass) * axbias + low_pass * (float)aa.x);
    aybias = ((1 - low_pass) * aybias + low_pass * (float)aa.y);
    
    low_pass = Ts / (0.1 + Ts);
    
    accel_x = ((1 - low_pass) * accel_x + low_pass * ((float)aa.x - axbias)) / 2;
    accel_y = ((1 - low_pass) * accel_y + low_pass * ((float)aa.y - aybias)) / 2;
    accel_z = ((1 - low_pass) * accel_z + low_pass *  (float)aa.z) / 1;
    }
    
    float g_norm = sqrt(powf(accel_x,2) + powf(accel_y,2) + powf(accel_z,2));
    
    VectorFloat gravity;
    
    low_pass = Ts / (1 + Ts);
    
    gravity.x = ((1 - low_pass) * gravity.x + low_pass * accel_x/g_norm)/3;
    gravity.y = ((1 - low_pass) * gravity.y + low_pass * accel_y/g_norm)/3;
    gravity.z = ((1 - low_pass) * gravity.z + low_pass * accel_z/g_norm)/1;

    mpu.dmpGetYawPitchRoll(asdf, &q, &gravity);
    
    low_pass = Ts / (0.1 + Ts);

    if (abs(ypr.x) < SENS_SAT && abs(ypr.y) < SENS_SAT && abs(ypr.z) < SENS_SAT)
    { 
    states[7] = ((1 - low_pass) * states[7] + (low_pass) * ypr.x);
    states[9] = ((1 - low_pass) * states[9] + (low_pass) * ypr.y);
    states[11] = ((1 - low_pass) * states[11] + (low_pass) * ypr.z);
    }
    
    low_pass = Ts / (10 + Ts);
      
    rolloffset  = ((1 - low_pass) * rolloffset  + low_pass * (asdf[2]));
    pitchoffset = ((1 - low_pass) * pitchoffset + low_pass * (asdf[1]));

    low_pass = Ts / (0.5 + Ts);

    states[6] = ((1 - low_pass) * states[6] + low_pass * (asdf[2] - rolloffset));
    states[8] = ((1 - low_pass) * states[8] + low_pass * (asdf[1] - pitchoffset));
    states[10] = (1 - low_pass) * states[10] + low_pass * states[11];

    low_pass = Ts / (1 + Ts);

    states[1] = ((1 - low_pass) * states[1] + low_pass * accel_x) / 2;
    states[3] = ((1 - low_pass) * states[3] + low_pass * accel_y) / 2;
    states[5] = ((1 - low_pass) * states[5] + low_pass * accel_z) / 5;

    states[0] = ((1 - low_pass) * states[0] + low_pass * states[1]);
    states[2] = ((1 - low_pass) * states[2] + low_pass * states[3]);
    states[4] = ((1 - low_pass) * states[4] + low_pass * states[5]);

    /***********************
       OUPUT CALCULATION
    ***********************/

    for (int i = 0; i < OUTPUTS ; outs[i++] = 0); //Reset outputs to zero

    if (kaka > MOTORSTART)
    {
    for (int j = 0; j < STATE_VARS ; j++)
    {
      err[j] = (states[j] - ref[j]);
      errint[j] += (err[j] * intcntrl[j]);
    }
    /*    SERVO OUTPUT FROM SLIDE     */

    for (int i = SERVO_FIRST; i < SERVO_LAST; i++)
    {
      for (int j = SLIDE_VAR_FIRST; j < SLIDE_VAR_LAST ; j++)
      {
        outs[i] -= Kain[i][j] * (err[j]) * 3 ; // <============================================= SERVO AUX GAIN SLIDE
      }
      if (outs[i] > SERVO_LIMIT_HIGH)
      {
        outs[i] = SERVO_LIMIT_HIGH;
      }
      if (outs[i] < SERVO_LIMIT_LOW)
      {
        outs[i] = SERVO_LIMIT_LOW;
      }
    }

    /*    ESC OUTPUT FROM SLIDE      */

    for (int i = ESC_FIRST; i < ESC_LAST; i++)
    {
      for (int j = SLIDE_VAR_FIRST; j < SLIDE_VAR_LAST ; j++)
      {
        if (errint[j] > SLIDE_INTEGRAL_LIMIT_HIGH)
        {
          errint[j] = SLIDE_INTEGRAL_LIMIT_HIGH;
        }
        if (errint[j] < SLIDE_INTEGRAL_LIMIT_LOW)
        {
          errint[j] = SLIDE_INTEGRAL_LIMIT_LOW;
        }
        outs[i] -= Kain[i][j] * ((errint[j]) + err[j] * 1) * 1; // <==================================== ESC AUX GAIN SLIDE
      }
      if (outs[i] > ESC_RANGE)
      {
        outs[i] = ESC_RANGE;
      }
      if (outs[i] < 0)
      {
        outs[i] = 0;
      }
    }

    /*    SERVO OUTPUT FROM SPIN    */

    for (int i = SERVO_FIRST; i < SERVO_LAST; i++)
    {
      for (int j = SPIN_VAR_FIRST; j < SPIN_VAR_LAST ; j++)
      {
        outs[i] -= Kain[i][j] * (err[j]) * 0.3; // <=========================================== SERVO AUX GAIN SPIN
      }
      if (outs[i] > SERVO_LIMIT_HIGH)
      {
        outs[i] = SERVO_LIMIT_HIGH;
      }
      if (outs[i] < SERVO_LIMIT_LOW)
      {
        outs[i] = SERVO_LIMIT_LOW;
      }
    }

    /*    ESC OUTPUT FROM SPIN     */

    for (int i = ESC_FIRST; i < ESC_LAST ; i++)
    {
      for (int j = SPIN_VAR_FIRST; j < SPIN_VAR_LAST ; j++)
      {
        if (errint[j] > SPIN_INTEGRAL_LIMIT_HIGH)
        {
          errint[j] = SPIN_INTEGRAL_LIMIT_HIGH;
        }
        if (errint[j] < SPIN_INTEGRAL_LIMIT_LOW)
        {
          errint[j] = SPIN_INTEGRAL_LIMIT_LOW;
        }
        outs[i] -= Kain[i][j] * ((errint[j] * 0.5) + (err[j] * 0.3)); // <============================= ESC AUX GAIN SPIN
      }
      if (outs[i] > ESC_RANGE)
      {
        outs[i] = ESC_RANGE;
      }
      if (outs[i] < 0)
      {
        outs[i] = 0;
      }
    }
    }
    /***********************
          OUPUT STAGE
    ***********************/
    
    /*    OUTPUT FILTERING    */

    low_pass = Ts / (0.07 + Ts);

    for (int i = SERVO_FIRST; i < SERVO_LAST; i++)
    {
      out[i] = (int)((1 - low_pass) * out[i] + low_pass * outs[i]);
    }

    low_pass = Ts / (0.01 + Ts);
    
    for (int i = ESC_FIRST; i < ESC_LAST; i++)
    {
      out[i] = (int)((1 - low_pass) * out[i] + low_pass * outs[i]);
    }

    /*   OUTPUT ASSIGNMENT */
    for (int i = SERVO_FIRST; i < SERVO_LAST; i++)
    {
    ACTUATORS[i].writeMicroseconds(SERVO_NULL + out[i]);
    }

    for (int i = ESC_FIRST; i < ESC_LAST; i++)
    {
    ACTUATORS[i].writeMicroseconds(ESC_LIMIT_LOW + out[i]); 
    }

//    Serial.print(aa.x);
//    Serial.print(" ");
//    Serial.print(aa.y);
//    Serial.print(" ");
//    Serial.print(aa.z);
//    Serial.print(" ");
//
//    
//    Serial.print(accel_x);
//    Serial.print("\t");
//    Serial.print(accel_y);
//    Serial.print("\t");
//    Serial.print(accel_z);
//    Serial.print("\t");

    
    for (int i = 4; i < 8; i+= 1)
    {
      Serial.print(out[i],0);
      Serial.print(" ");
    }

//    for (int i = 0; i < 12; i+= 1)
//    {
//      Serial.print(states[i],DEC);
//      Serial.print("\t");
//    }
    Serial.println(" ");
  }
}
