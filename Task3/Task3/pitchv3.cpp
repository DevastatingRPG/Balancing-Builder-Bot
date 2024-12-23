#include <Arduino.h>

// #include "ArduPID.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Encoder.h>
#include <L298NX2.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

// //bluetooth
const int RX = A1;
const int TX = A0;

// Motor A
const int ENA = 6;        // Pin 1 of L293D IC
const int inputPin1 = A2; // Pin 15 of L293D IC
const int inputPin2 = A3; // Pin 10 of L293D IC

// Motor B
const int ENB = 5;
const int inputPin3 = 9; // Pin  7 of L293D IC
const int inputPin4 = 4; // Pin  2 of L293D IC

// Encoders
const int encA1 = 2;
const int encB1 = 7;
const int encA2 = 3;
const int encB2 = 8;

// MPU6050
// const int INTERRUPT_PIN = 2;
bool blinkState;

// struct PIDParams
// {
//     float Kp;
//     float Ki;
//     float Kd;
// };

struct PIDParams
{
    float Kp;
    float Ki;
    float Kd;
    float _kp;
    float _ki;
    float _kd;
    float _N_d;
    float _T_sec;

    PIDParams(float Kp, float Ki, float Kd, float N_d, float T_sec)
        : Kp(Kp), Ki(Ki), Kd(Kd), _N_d(N_d), _T_sec(T_sec)
    {
        _kp = Kp;
        // _kd = (2 * Kd * _N_d) / (2 + _N_d * _T_sec);
        _kd = Kd;
        _ki = Ki;
        // _ki = 0.5 * Ki * _T_sec;
    }
};

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;        // [w, x, y, z]         Quaternion container
VectorInt16 gy;      // [x, y, z]            Gyro sensor measurements
VectorFloat gravity; // [x, y, z]            Gravity vector
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady()
{
    MPUInterrupt = true;
}

float N = 100;
uint32_t T = 20;
uint32_t lastTime = 0;
float Kb = 1;
double Setpoint_Pitch, Input_Pitch, Output_Pitch, Error_Pitch;
float Setpoint_Pos, Input_Pos, Output_Pos, Error_Pos;

unsigned long timer = 0;
const unsigned long sampleTime = 10; // Sample time in milliseconds

SoftwareSerial bluetooth(RX, TX); // RX, TX

MPU6050 mpu;

L298NX2 motors(ENA, inputPin1, inputPin2, ENB, inputPin4, inputPin3);

// PIDParams pitch1 = {80, 3, 2};
// PIDParams pitch2 = {30, 90, 2};
// PIDParams pitch1 = {80, 3, 2};
// PIDParams pitch2 = {60, 90, 2};
PIDParams pitch1(80, 140, 0, 100, 20);
PIDParams pitch2(20, 8, 0.5, 100, 20);
// PIDParams pos1 = {3, 0, 0};
// PIDParams pos2 = {0.5, 0, 0};

PID pitchPID(&Input_Pitch, &Output_Pitch, &Setpoint_Pitch, pitch1._kp, pitch1._ki, pitch1._kd, DIRECT);
// PID_BC posPID(&Output_Pos, 0, 0, 0, N, T, 1);

Encoder enc1(encA1, encB1);
Encoder enc2(encA2, encB2);

String mode = "CONNY";

void setup()
{
    // Serial.begin(115200);
    bluetooth.begin(9600); // For HC-05 (default baud rate is 9600)

    // Serial.println("//bluetooth is ready!");

    // MPU 6050
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    // bluetooth.println("DMP init");

    // mpu.setXAccelOffset(-4752);
    // mpu.setYAccelOffset(1219);
    // mpu.setZAccelOffset(1242);
    // mpu.setXGyroOffset(68);
    // mpu.setYGyroOffset(12);
    // mpu.setZGyroOffset(-38);

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

    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();
        DMPReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
    }
    // Serial.println("Done!\n");
    timer = millis();

    // Setpoint_Pitch = -2.2;
    Setpoint_Pitch = -0.1;
    Setpoint_Pos = 0;

    // Input_Pitch = mpu.getAngleY();
    // Input_Pos = (enc2.read() + enc1.read()) / 2;

    // pitchPID.setSaturation(-255, 255);
    // posPID.setSaturation(-255, 255);
    pitchPID.SetOutputLimits(-255, 255);
    //  //posPID.SetOutputLimits(-255, 255);

    pitchPID.SetSampleTime(20);
    // //posPID.SetSampleTime(10);

    pitchPID.SetMode(AUTOMATIC);
    // //posPID.SetMode(AUTOMATIC);

    motors.setSpeed(100);
    motors.reset();
    motors.stop();

    enc2.write(0);
    enc1.write(0);
}

void loop()
{
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
    if (!DMPReady)
        return;

    // bluetooth.println("Ready DMP");
    //  read a packet from FIFO. Get the Latest packet
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
    {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // yawGyroRate = gy.z;                   // rotation rate in degrees per second
        Input_Pitch = ypr[1] * 180 / M_PI; // angle in degree
        Input_Pos = (enc1.read() + enc2.read()) / 2;

        Error_Pos = Input_Pos - Setpoint_Pos;
        Error_Pitch = Input_Pitch - Setpoint_Pitch;

        // Serial.print("Error Pitch: ");
        // // bluetooth.println(Error_Pitch);

        // Serial.print("Error Pos: ");
        // Serial.println(enc1.read());

        if (fabs(Error_Pitch) > 11)
        {
            // bluetooth.println("AGGY");
            if (mode == "CONNY")
            {
                bluetooth.println("AGGY");
                tone(12, 1000, 100);
            }
            mode = "AGGY";
            pitchPID.SetTunings(pitch1._kp, pitch1._ki, pitch1._kd);
        }
        else
        {
            // bluetooth.println("CONNY");
            if (mode == "AGGY")
            {
                bluetooth.println("CONNY");
                tone(12, 1000, 100);
            }
            mode = "CONNY";
            pitchPID.SetTunings(pitch2._kp, pitch2._ki, pitch2._kd);
        }

        // if (Error_Pos > 300)
        // {
        //     // posPID.setTunings(pos1.Kp, pos1.Ki, pos1.Kd);
        // }
        // else
        // {
        //     // posPID.setTunings(pos2.Kp, pos2.Ki, pos2.Kd);
        // }

        bluetooth.println(Output_Pitch);
        pitchPID.Compute();

        float vel = Output_Pitch;

        if (vel < 0)
        {
            motors.forward();
        }
        else
        {
            motors.backward();
        }

        motors.setSpeed(constrain(abs(vel), 0, 255));

        // Send data over Bluetooth
        bluetooth.print(Input_Pitch);
        bluetooth.print(",");
        bluetooth.print(Error_Pitch);
        bluetooth.print(",");
        bluetooth.println(Output_Pitch);
        // motors.setSpeed(255);
    }
}
