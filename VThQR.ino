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
//  Last Update: July 13th, 2016

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

#define SLIDE_INTEGRAL_LIMIT_HIGH 1200
#define SLIDE_INTEGRAL_LIMIT_LOW (-SLIDE_INTEGRAL_LIMIT_HIGH)
#define TORQUE_INTEGRAL_LIMIT_HIGH 500
#define TORQUE_INTEGRAL_LIMIT_LOW (-TORQUE_INTEGRAL_LIMIT_HIGH)

MPU6050 mpu;  //MPU struct init

/**************************
  ACTUATOR INSTANTIATION
***************************/

Servo ACT1;
Servo ACT2;
Servo ACT3;
Servo ACT4;

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

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
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
VectorInt16 ypr;        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float asdf[3];

/***********************
   CONTROL VARIABLES
***********************/

float Ks[8][6] = {{ 0, 1.5293, 0, -1.5293, 0, 0}, //slide matrix
  { 0, -1.5293, 0, -1.5293, 0, 0},
  { 0, -1.5293, 0, 1.5293, 0, 0},
  { 0, 1.5293, 0, 1.5293, 0, 0},
  {0, 0, 0, 0, 1.158, 1.3017},
  {0, 0, 0, 0, 1.158, 1.3017},
  {0, 0, 0, 0, 1.158, 1.3017},
  {0, 0, 0, 0, 1.158, 1.3017}
};

float Kt[8][6] = {{0, 0, 0, 0, 0, 1.2455},        //torque matrix
  {0, 0, 0, 0, 0, 1.2455},
  {0, 0, 0, 0, 0, 1.2455},
  {0, 0, 0, 0, 0, 1.2455},
  { -1.158, -10.0485, 1.158, 10.0485, 0, 0},
  { 1.158, 10.0485, 1.158, 10.0485, 0, 0},
  { 1.158, 10.0485, -1.158, -10.0485, 0, 0},
  { -1.158, -10.0485, -1.158, -10.0485, 0, 0}
};

//state variables
float states[12];
float outs[8];
float out[8];
float theta_x = 0;
float theta_y = 0;
float accel_x;
float accel_y;
float accel_z;

// offset variables
int axoffset = -1752;
int ayoffset = -5768;
int azoffset = 40;
int gxoffset = 1630;
int gyoffset = -1340;
int gzoffset = -1252;
float rolloffset = 0;
float pitchoffset = 0;

//proportional error bins
float err[12] = {0, //     X
                 0, // Vel X
                 0, //     Y
                 0, // Vel Y
                 0, //     Z
                 0, // Vel Z
                 0, // Pitch
                 0, // Pitch Rate
                 0, // Roll
                 0, // Roll Rate
                 0, // Yaw
                 0  // Yaw Rate
                };

//integral error bins
float errint[12] = {0, //     X

                    0, // Vel X
                    0, //     Y
                    0, // Vel Y
                    0, //     Z
                    0, // Vel Z
                    0, // Pitch
                    0, // Pitch Rate
                    0, // Roll
                    0, // Roll Rate
                    0, // Yaw
                    0  // Yaw Rate
                   };

//gain variable for integral control
float intcntrl[12] = {0, //     X
                      0, // Vel X
                      0, //     Y
                      0, // Vel Y
                      0.1, //     Z          <================INTEGRAL CONTROL TOGGLE=================================================================================
                      0, // Vel Z
                      0.1, // Pitch
                      0, // Pitch Rate
                      0.1, // Roll
                      0, // Roll Rate
                      0, // Yaw
                      0  // Yaw Rate
                     };

//reference "signals"
float ref[12] = {0, //     X
                 0, // Vel X
                 0, //     Y
                 0, // Vel Y
                 1, //     Z
                 0, // Vel Z
                 0, // Pitch
                 0, // Pitch Rate
                 0, // Roll
                 0, // Roll Rate
                 0, // Yaw
                 0  // Yaw Rate
                };

uint32_t timer;

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

  ACT1.attach(6);
  ACT2.attach(7);
  ACT3.attach(3);
  ACT4.attach(4);
  ESC1.attach(9);
  ESC2.attach(8);
  ESC3.attach(11);
  ESC4.attach(10);

  /***********************
     ESC CALIBRATION
  ***********************/

  ESC1.writeMicroseconds(ESC_LIMIT_HIGH);
  ESC2.writeMicroseconds(ESC_LIMIT_HIGH);
  ESC3.writeMicroseconds(ESC_LIMIT_HIGH);
  ESC4.writeMicroseconds(ESC_LIMIT_HIGH);

  delay(3000);

  ESC1.writeMicroseconds(ESC_LIMIT_LOW);
  ESC2.writeMicroseconds(ESC_LIMIT_LOW);
  ESC3.writeMicroseconds(ESC_LIMIT_LOW);
  ESC4.writeMicroseconds(ESC_LIMIT_LOW);

  delay(1000);

  Wire.begin(); //begin serial comms

  mpu.initialize();  // initialize device
  Serial.begin(115200);

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

  for (int i = 0; i < 8 ; ++i)
  {
    out[i] = 0;
  }
  ACT1.writeMicroseconds(SERVO_NULL);
  ACT2.writeMicroseconds(SERVO_NULL);
  ACT3.writeMicroseconds(SERVO_NULL);
  ACT4.writeMicroseconds(SERVO_NULL);

  ESC1.writeMicroseconds(ESC_LIMIT_LOW);
  ESC2.writeMicroseconds(ESC_LIMIT_LOW);
  ESC3.writeMicroseconds(ESC_LIMIT_LOW);
  ESC4.writeMicroseconds(ESC_LIMIT_LOW);

  /***********************
        GYRO OFFSET
  ***********************/

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
      fifoCount -= packetSize;
      mpu.dmpGetGyro( &ypr , fifoBuffer);

      gxoffset = gxoffset - ypr.x / 2;
      gyoffset = gyoffset - ypr.y / 2;
      gzoffset = gzoffset - ypr.z / 2;

      mpu.setXGyroOffset(gxoffset);
      mpu.setYGyroOffset(gyoffset);
      mpu.setZGyroOffset(gzoffset);

      /*
            Serial.print(i);
            Serial.print("\t || \t");
            Serial.print(gyro_x);
            Serial.print(" \t");
            Serial.print(gyro_y);
            Serial.print(" \t");
            Serial.println(gyro_z);
      */
      delay(1);
    }
  }

  /***********************
        ACCEL OFFSET
  ***********************/

  for (int i = 0; i < 1000; ++i)
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
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);

      accel_x = aa.x ;
      accel_y = aa.y ;
      accel_z = aa.z ;

      if (accel_x < 0)
      {
        ++axoffset;
      }
      else
      {
        --axoffset;
      }

      if (accel_y < 0)
      {
        ++ayoffset;
      }
      else
      {
        --ayoffset;
      }
      if (accel_z < 0)
      {
        ++azoffset;
      }
      else
      {
        --azoffset;
      }
      /*
            Serial.print(i);
            Serial.print("\t || \t");
            Serial.print(accel_x);
            Serial.print("\t");
            Serial.print(accel_y);
            Serial.print("\t");
            Serial.print(accel_z);
            Serial.println("\t");
      */
      mpu.setXAccelOffset(axoffset);
      mpu.setYAccelOffset(ayoffset);
      mpu.setZAccelOffset(azoffset);
      delay(1);
    }
  }

  /***********************
       ANGLE OFFSET
  ***********************/

  for (int i = 0; i < 500; ++i)
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
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(asdf, &q, &gravity);

      theta_x = asdf[2] - rolloffset;
      theta_y = asdf[1] - pitchoffset;

      rolloffset = rolloffset + theta_x / 2;
      pitchoffset = pitchoffset + theta_y / 2;
      /*
            Serial.print(i);
            Serial.print("\t || \t");
            Serial.print(theta_x);
            Serial.print("\t");
            Serial.print(theta_y);
            Serial.println("\t");
      */
      delay(1);
    }
  }
  timer = micros();
}

//================================================================
//                   MAIN LOOP
//================================================================
void loop() {

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
    mpu.dmpGetGravity(&gravity, &q);
    delay(5);
    mpu.dmpGetGyro( &ypr , fifoBuffer);
    delay(5);
    mpu.dmpGetYawPitchRoll(asdf, &q, &gravity);
    delay(5);

    /**********************************
        MEASUREMENT PREPROCESSING
    ***********************************/

    float Ts = (float)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    float olpha = Ts / (0.02 + Ts);
    float alpha = Ts / (10 + Ts);
    float ylpha = Ts / (0.2 + Ts);
    float ulpha = Ts / (0.1 + Ts);

    accel_x = ((1 - ulpha) * accel_x + ulpha * aa.x);
    accel_y = ((1 - ulpha) * accel_y + ulpha * aa.y);
    accel_z = ((1 - ulpha) * accel_z + ulpha * aa.z) / 5;

    states[7] = (1 - olpha) * states[7] + (olpha) * ypr.x;
    states[9] = (1 - olpha) * states[9] + (olpha) * ypr.y;
    states[11] = (1 - olpha) * states[11] + (olpha) * ypr.z;

    states[6] = (1 - ylpha) * states[6] + ylpha * (asdf[2] - rolloffset);
    states[8] = (1 - ylpha) * states[8] + ylpha * (asdf[1] - pitchoffset);
    states[10] = (1 - ylpha) * states[10] + ylpha * states[11];

    states[1] = ((1 - olpha) * states[1] + olpha * accel_x) / 2;
    states[3] = ((1 - olpha) * states[3] + olpha * accel_y) / 2;
    states[5] = ((1 - olpha) * states[5] + olpha * accel_z) / 5;

    states[0] = ((1 - alpha) * states[0] + alpha * states[1]);
    states[2] = ((1 - alpha) * states[2] + alpha * states[3]);
    states[4] = ((1 - alpha) * states[4] + alpha * states[5]);

    /*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+
        states[0] = x;
        states[1] = vel_x;
        states[2] = y;
        states[3] = vel_y;
        states[4] = z;
        states[5] = vel_z;

        states[6] = theta_x;
        states[7] = wel_x;
        states[8] = theta_y;
        states[9] = wel_y;
        states[10] = theta_z;
        states[11] = wel_z;
      +*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*/

    /***********************
       OUPUT CALCULATION
    ***********************/

    for (int i = 0; i < 8 ; ++i) //Reset outputs to zero
    {
      outs[i] = 0;
    }

    for (int j = 0; j < 12 ; ++j)
    {
      err[j] = (states[j] - ref[j]);
      errint[j] += (err[j] * intcntrl[j]);
    }
    /*    SERVO OUTPUT FROM SLIDE     */

    for (int i = 0; i < 4 ; ++i)
    {
      for (int j = 0; j < 6 ; ++j)
      {
        outs[i] -= Ks[i][j] * (err[j]) * 1; // <=========================================== SERVO AUX GAIN SLIDE
      }
      /************************/
      if (outs[i] > SERVO_LIMIT_HIGH)
      {
        outs[i] = SERVO_LIMIT_HIGH;
      }
      /************************/
      if (outs[i] < SERVO_LIMIT_LOW)
      {
        outs[i] = SERVO_LIMIT_LOW;
      }
    }

    /*    ESC OUTPUT FROM SLIDE      */

    for (int i = 4; i < 8 ; ++i)
    {
      for (int j = 0; j < 6 ; ++j)
      {
        /************************/
        if (errint[j] > SLIDE_INTEGRAL_LIMIT_HIGH)
        {
          errint[j] = SLIDE_INTEGRAL_LIMIT_HIGH;
        }
        /************************/
        if (errint[j] < SLIDE_INTEGRAL_LIMIT_LOW)
        {
          errint[j] = SLIDE_INTEGRAL_LIMIT_LOW;
        }
        /************************/
        outs[i] -= Ks[i][j] * ((errint[j]) + err[j] * 1) ; // <============================ ESC AUX GAIN SLIDE
      }
      /************************/
      if (outs[i] > ESC_RANGE)
      {
        outs[i] = ESC_RANGE;
      }
      /************************/
      if (outs[i] < 0)
      {
        outs[i] = 0;
      }
    }

    /*    SERVO OUTPUT FROM TORQUE    */

    for (int i = 0; i < 4 ; ++i)
    {
      for (int j = 0; j < 6 ; ++j)
      {
        outs[i] -= Kt[i][j] * (err[j + 6]) * 1; // <====================================== SERVO AUX GAIN TORQUE
      }
      /************************/
      if (outs[i] > SERVO_LIMIT_HIGH)
      {
        outs[i] = SERVO_LIMIT_HIGH;
      }
      /************************/
      if (outs[i] < SERVO_LIMIT_LOW)
      {
        outs[i] = SERVO_LIMIT_LOW;
      }
    }

    /*    ESC OUTPUT FROM TORQUE     */

    for (int i = 4; i < 8 ; ++i)
    {
      for (int j = 0; j < 6 ; ++j)
      {
        /************************/
        if (errint[j + 6] > TORQUE_INTEGRAL_LIMIT_HIGH)
        {
          errint[j + 6] = TORQUE_INTEGRAL_LIMIT_HIGH;
        }
        /************************/
        if (errint[j + 6] < TORQUE_INTEGRAL_LIMIT_LOW)
        {
          errint[j + 6] = TORQUE_INTEGRAL_LIMIT_LOW;
        }
        /************************/
        outs[i] -= Kt[i][j] * ((errint[j + 6] * 1) + (err[j + 6] * 1)); // <================ ESC AUX GAIN TORQUE
      }
      /************************/
      if (outs[i] > ESC_RANGE)
      {
        outs[i] = ESC_RANGE;
      }
      /************************/
      if (outs[i] < 0)
      {
        outs[i] = 0;
      }
      //delay(1);
    }

    /*    OUTPUT FILTERING    */

    float elpha = Ts / (0.05 + Ts);

    for (int i = 0; i < 4 ; ++i)
    {
      out[i] = (1 - elpha) * out[i] + elpha * outs[i];
    }

    elpha = Ts / (1 + Ts);

    for (int i = 4; i < 8 ; ++i)
    {
      out[i] = (1 - elpha) * out[i] + elpha * outs[i];
    }

    /*********************************
       SERIAL PRINTING FOR DEBUGGING
    **********************************/

    //
    //              Serial.print((accel_x), 6);
    //              Serial.print(" ");
    //              Serial.print((accel_y), 6);
    //              Serial.print(" ");
    //              Serial.print((accel_z), 6);
    //              Serial.print(" ");
    //              Serial.print(0);


    for (int i = 6; i < 10; i += 2)
    {
      Serial.print(states[i] * (180/3.14), 3);
      Serial.print(" || ");

      //            Serial.print(errint[i], 3);
      //            Serial.print(" || ");
      //
      //            Serial.print(err[i], 3);
      //            Serial.print(" || ");
    }

    /*
      for (int i = 0; i < 8 ; ++i)
      {
      Serial.print(out[i], 0);
      Serial.print("\t");
      }
    */
    Serial.println(" ");

    /***********************
       OUPUT ASSIGNMENT
    ***********************/

    ACT1.writeMicroseconds(SERVO_NULL + out[0]);
    ACT2.writeMicroseconds(SERVO_NULL + out[1]);
    ACT3.writeMicroseconds(SERVO_NULL + out[2]);
    ACT4.writeMicroseconds(SERVO_NULL + out[3]);

    ESC1.writeMicroseconds(ESC_LIMIT_LOW + out[4]);
    ESC2.writeMicroseconds(ESC_LIMIT_LOW + out[5]);
    ESC3.writeMicroseconds(ESC_LIMIT_LOW + out[6]);
    ESC4.writeMicroseconds(ESC_LIMIT_LOW + out[7]);

    delay(0);
  }
}
