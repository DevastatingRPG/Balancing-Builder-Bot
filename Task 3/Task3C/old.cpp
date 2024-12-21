#include <Arduino.h>
#include <Encoder.h>
#include <L298NX2.h>
#include <SoftwareSerial.h>
#include <util/atomic.h>

#include "ArduPID.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// //bluetooth
const int RX = A1;
const int TX = A0;

// Motor A
const int ENA = 6;         // Pin 1 of L293D IC
const int inputPin1 = A2;  // Pin 15 of L293D IC
const int inputPin2 = A3;  // Pin 10 of L293D IC

// Motor B
const int ENB = 5;
const int inputPin3 = 9;  // Pin  7 of L293D IC
const int inputPin4 = 4;  // Pin  2 of L293D IC

// Encoders
const int ENCA = 7;
const int ENCB = 2;
const int ENCAL = 8;
const int ENCBL = 3;

volatile int posiL = 0;
volatile int posiR = 0;

MPU6050 mpu;

// MPU6050
const int INTERRUPT_PIN = 2;
bool blinkState;

struct PIDParams {
  float Kp;
  float Ki;
  float Kd;
};

// /---MPU6050 Control/Status Variables---/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

// // /---Orientation/Motion Variables---/
// Quaternion q;        // [w, x, y, z]         Quaternion container
// VectorInt16 gy;      // [x, y, z]            Gyro sensor measurements
// VectorFloat gravity; // [x, y, z]            Gravity vector
// float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// /------Interrupt detection routine------/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

float N = 100;
uint32_t T = 20;
// uint32_t lastTime = 0;
// float Kb = 1;
float Setpoint_Pitch, Input_Pitch, Output_Pitch, Error_Pitch;
float Setpoint_Pos, Input_Pos, Output_Pos, Error_Pos;

// unsigned long timer = 0;
// const unsigned long sampleTime = 10; // Sample time in milliseconds

SoftwareSerial bluetooth(RX, TX);  // RX, TX

L298NX2 motors(ENA, inputPin1, inputPin2, ENB, inputPin4, inputPin3);

// PIDParams pitch1 = {80, 3, 2};
// PIDParams pitch2 = {30, 90, 2};
// PIDParams pitch1 = {80, 3, 2};
// PIDParams pitch2 = {60, 90, 2};
PIDParams pitch1 = {80, 140, 0};
PIDParams pitch2 = {20, 70, 0.5};
PIDParams pos1 = {3, 0, 0};
PIDParams pos2 = {0.5, 0, 0};

PID_IC pitchPID(&Output_Pitch, pitch1.Kp, pitch1.Ki, pitch1.Kd, N, T);
PID_IC posPID(&Output_Pos, pos1.Kp, pos1.Ki, pos1.Kd, N, T);

// Encoder enc1(encA1, encB1);
// Encoder enc2(encA2, encB2);

// void readEncoderOne();
// void readEncoderTwo();

// String mode = "CONNY";

void setup() {
  // Serial.begin(115200);
  bluetooth.begin(9600);  // For HC-05 (default baud rate is 9600)

  // bluetooth.println("//bluetooth is ready!");
  // Serial.println("//bluetooth is ready!");

  // MPU 6050
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  // pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // bluetooth.println("DMP init");

  mpu.setXAccelOffset(1254);
  mpu.setYAccelOffset(497);
  mpu.setZAccelOffset(446);
  mpu.setXGyroOffset(32);
  mpu.setYGyroOffset(-80);
  mpu.setZGyroOffset(-31);

  // pinMode(ENCA, INPUT);
  // pinMode(ENCB, INPUT);

  // pinMode(ENCAL, INPUT);
  // pinMode(ENCBL, INPUT);

  // digitalWrite(7, HIGH);
  // digitalWrite(2, HIGH);
  // attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderOne, RISING);

  // digitalWrite(ENCAL, HIGH);
  // digitalWrite(ENCBL, HIGH);
  // attachInterrupt(digitalPinToInterrupt(ENCBL), readEncoderTwo, RISING);

  // byte status = mpu.begin();
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
  // while (status != 0)
  // {
  // } // stop everything if could not connect to MPU6050

  // Serial.println(F("Calculating offsets, do not move MPU6050"));
  // delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  // mpu.calcOffsets(); // gyro and accelero
  // mpu.setAccOffsets(-4714, 1237, 1238);
  // mpu.setGyroOffsets(63, 12, -35);

  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
  }
  // Serial.println("Done!\n");
  // timer = millis();

  // Setpoint_Pitch = -2.2;
  Setpoint_Pitch = 0;
  Setpoint_Pos = 0;

  // Input_Pitch = mpu.getAngleY();
  // Input_Pos = (enc2.read() + enc1.read()) / 2;

  pitchPID.setSaturation(-255, 255);
  posPID.setSaturation(-255, 255);
  // pitchPID.SetOutputLimits(-255, 255);
  // posPID.SetOutputLimits(-255, 255);

  // pitchPID.SetSampleTime(10);
  // posPID.SetSampleTime(10);

  // pitchPID.SetMode(AUTOMATIC);
  // posPID.SetMode(AUTOMATIC);

  // motors.setSpeed(100);
  // motors.reset();
  // motors.stop();

  // enc2.write(0);
  // enc1.write(0);
}

void loop() {
  /*
  // Check for data from //bluetooth (furthur remote use)
  if (//bluetooth.available()) {
    char btData = //bluetooth.read();

    // Filter out newline and carriage return characters
    if (btData != '\n' && btData != '\r') {
      Serial.print("Received from //bluetooth: ");
      Serial.println(btData);
    }
  }
  */
  // int pos = 0;
  // int posL = 0;
  // int posR = 0;
  float vel = 0;

  // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  //   posL = posiL;
  //   posR = posiR;
  //   pos = (posL + posR) / 2.0;
  // }

  if (!DMPReady)
    return;

  // bluetooth.println("Ready DMP");
  //  read a packet from FIFO. Get the Latest packet
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, FIFOBuffer);

    // yawGyroRate = gy.z;                   // rotation rate in degrees per second
    Input_Pitch = ypr[2] * 180 / M_PI;  // angle in degree
    // Input_Pos = pos;

    Error_Pitch = Input_Pitch - Setpoint_Pitch;
    // Error_Pos = Input_Pos - Setpoint_Pos;

    // Serial.print("Error Pitch: ");
    // // bluetooth.println(Error_Pitch);

    // Serial.print("Error Pos: ");
    // Serial.println(enc1.read());

    Error_Pitch *= 1;
    Error_Pos *= 0.8;

    // if (Error_Pitch > -2 && Error_Pitch < 2) {
    //   vel = 0;
    //   motors.setSpeed(0);
    //   motors.stop();
    // } else {
    if (fabs(Error_Pitch) < 90) {
      // bluetooth.println("AGGY");
      // if (mode == "CONNY") {
      //   // bluetooth.println("AGGY");
      // }
      // mode = "AGGY";
      pitchPID.setTunings(pitch2.Kp, pitch2.Ki, pitch2.Kd, N);
    } else if (fabs(Error_Pitch) > 90) {
      // bluetooth.println("CONNY");
      // if (mode == "AGGY") {
      //   // bluetooth.println("CONNY");
      // }
      // mode = "CONNY";
      pitchPID.setTunings(pitch1.Kp, pitch1.Ki, pitch1.Kd, N);
    }

    float *results = pitchPID.compute(Error_Pitch);
    vel = Output_Pitch;

    if (vel < 0) {
      motors.forward();
    } else {
      motors.backward();
    }
  }

  if (Error_Pos > 300) {
    posPID.setTunings(pos1.Kp, pos1.Ki, pos1.Kd, N);
  } else {
    posPID.setTunings(pos2.Kp, pos2.Ki, pos2.Kd, N);
  }

  posPID.compute(Error_Pos);

  motors.setSpeed(constrain(abs(vel), 0, 255));
  // motors.setSpeed(255);
  // bluetooth.println(vel);
  // bluetooth.println(Output_Pitch);
  // float *results = pitchPID.compute(Error_Pitch);

  // if (fabs(Error_Pitch) > 7)
  // {
  //     for (int i = 0; i < 3; i++)
  //     {
  //         switch (i)
  //         {
  //         case 0:
  //             bluetooth.print("P: ");
  //             bluetooth.print(results[i]);
  //             break;
  //         case 1:
  //             bluetooth.print("I: ");
  //             bluetooth.print(results[i]);
  //             break;
  //         case 2:
  //             bluetooth.print("D: ");
  //             bluetooth.println(results[i]);
  //             break;
  //         }
  //     }
  // }

  // bluetooth.println(Output_Pitch);

  // vel = Output_Pitch;
  // float vel = Output_Pitch + Output_Pos;
  // if (abs(vel) < 100 && mode == "AGGY") {
  //   // bluetooth.println("OOPSIE");
  //   for (int i = 0; i < 3; i++) {
  //     switch (i) {
  //       case 0:
  //         bluetooth.print("P: ");
  //         bluetooth.print(results[i]);
  //         break;
  //       case 1:
  //         bluetooth.print("I: ");
  //         bluetooth.print(results[i]);
  //         break;
  //       case 2:
  //         bluetooth.print("D: ");
  //         bluetooth.println(results[i]);
  //         break;
  //     }
  //   }
  // } else if (abs(vel) == 255) {
  //   // bluetooth.println("WOWW VERY DANGEROUS");
  // }

  // // bluetooth.println(Output_Pos);
  // bluetooth.println(vel);
  // bluetooth.println("");
}

// void readEncoderOne() {
//   int a = digitalRead(ENCA);
//   int b = digitalRead(ENCB);
//   if (b > a) {
//     posiL--;
//   } else {
//     posiL++;
//   }
// }

// void readEncoderTwo() {
//   int c = digitalRead(ENCAL);
//   int d = digitalRead(ENCBL);
//   if (d > c) {
//     posiR++;
//   } else {
//     posiR--;
//   }
// }