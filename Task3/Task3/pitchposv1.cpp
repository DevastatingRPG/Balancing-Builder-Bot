// PID Library
#include <PID_v1.h>

// MPU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// //bluetooth
const int RX = A1;
const int TX = A0;

// Motor A
const int ENA = 6; // Pin 1 of L293D IC
int motor1Pin1 = A2;
int motor1Pin2 = A3;

// Motor B
const int ENB = 5;
int motor2Pin1 = 4;
int motor2Pin2 = 9;

// Encoders
const int encA1 = 2;
const int encB1 = 7;
const int encA2 = 3;
const int encB2 = 8;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// Encoder Pulse Counts
int wheel_pulse_count_left = 0;
int wheel_pulse_count_right = 0;

#define PID_MIN_LIMIT -255          // Min limit for PID
#define PID_MAX_LIMIT 255           // Max limit for PID
#define PID_SAMPLE_TIME_IN_MILLI 10 // This is PID sample time in milliseconds

#define SETPOINT_PITCH_ANGLE_OFFSET 0

#define MIN_ABSOLUTE_SPEED 0 // Min motor speed

double Setpoint_Pitch, Input_Pitch, Output_Pitch, Error_Pitch;
// double Setpoint_Yaw, Input_Yaw, Output_Yaw;
double Setpoint_Pos, Input_Pos, Output_Pos, Error_Pos;

struct PIDParams
{
    float Kp;
    float Ki;
    float Kd;
    float _kp;
    float _ki;
    float _kd;

    PIDParams(float Kp, float Ki, float Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd)
    {
        _kp = Kp;
        _kd = Kd;
        _ki = Ki;
    }
};

PIDParams pitchCon(13, 185, 0.6);
PIDParams posCon(0, 0, 0);
PIDParams posAgg(0.005, 0, 0);
// PIDParams yaw(0.5, 0.5, 0);

PID pitchPID(&Input_Pitch, &Output_Pitch, &Setpoint_Pitch, pitchCon._kp, pitchCon._ki, pitchCon._kd, DIRECT);
PID posPID(&Input_Pos, &Output_Pos, &Setpoint_Pos, posAgg._kp, posAgg._ki, posAgg._kd, DIRECT);

void rotateMotor(int speed1, int speed2);
void mot_rencoder_right();
void mot_rencoder_left();

void setupPID()
{
    Setpoint_Pitch = SETPOINT_PITCH_ANGLE_OFFSET;
    pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

    posPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    posPID.SetMode(AUTOMATIC);
    posPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void setupMotors()
{
    pinMode(ENA, OUTPUT);
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    rotateMotor(0, 0);
}

void setupMPU()
{

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(1254);
    mpu.setYAccelOffset(497);
    mpu.setZAccelOffset(446);
    mpu.setXGyroOffset(32);
    mpu.setYGyroOffset(-80);
    mpu.setZGyroOffset(-31);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
    }
}

void setupEncoders()
{
    pinMode(encB1, INPUT_PULLUP);
    digitalWrite(encB1, HIGH);
    // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

    pinMode(encA1, INPUT_PULLUP);
    digitalWrite(encA1, HIGH);
    attachInterrupt(digitalPinToInterrupt(encA1), mot_rencoder_left, RISING);
    // ---------------------------------------------- ///

    //------Encoder HW Interrupt setup-----//
    pinMode(encB2, INPUT_PULLUP); // 10
    digitalWrite(encB2, HIGH);
    // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

    pinMode(encA2, INPUT_PULLUP); // 11
    digitalWrite(encA2, HIGH);
    attachInterrupt(digitalPinToInterrupt(encA2), mot_rencoder_right, RISING);
}

void setup()
{
    // This is to set up motors
    setupMotors();
    // This is to set up MPU6050 sensor
    setupMPU();
    // This is to set up PID
    setupPID();
    // Encoder Setup
    setupEncoders();
}

bool stopping = false;

void loop()
{
    if (!dmpReady)
    {
        return;
    }

    float vel = 0;
    // Error_Pos = (wheel_pulse_count_left + wheel_pulse_count_right) / 2;
    Error_Pos = wheel_pulse_count_left;
    if (abs(Error_Pos) > 100)
    {
        Input_Pos = Error_Pos;
        posPID.Compute(true);
        Setpoint_Pitch = -Output_Pos;
        stopping = true;
    }
    else
    {
        Setpoint_Pitch = 0;
        stopping = false;
    }

    // read a packet from FIFO. Get the Latest packet
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        Input_Pitch = ypr[2] * 180 / M_PI; // angle in degree

        pitchPID.Compute(true);

        vel += Output_Pitch;

#ifdef PRINT_DEBUG_BUILD
        Serial.println("The gyro  before ");
        Serial.println(pitchGyroAngle);
        Serial.println("The setpoints ");
        Serial.println(setpointPitchAngle);
        Serial.println("The pid output ");
        Serial.println(pitchPIDOutput);
        delay(500);
#endif
    }

    if (stopping)
    {
        vel *= 0.8;
    }
    rotateMotor(vel, vel);
}

void rotateMotor(int speed1, int speed2)
{
    if (speed1 < 0)
    {
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
    }
    else if (speed1 >= 0)
    {
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
    }

    if (speed2 < 0)
    {
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, HIGH);
    }
    else if (speed2 >= 0)
    {
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
    }

    speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
    speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;

    speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 255);
    speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 255);

    analogWrite(ENA, speed1);
    analogWrite(ENB, speed2);
}

// ISR for motor encoder
void mot_rencoder_left()
{
    // if (digitalRead(encodPinBR) == HIGH) {
    if (digitalRead(encA1) > digitalRead(encB1))
    {
        wheel_pulse_count_left = wheel_pulse_count_left + 1;
    }
    else
    {
        wheel_pulse_count_left = wheel_pulse_count_left - 1;
    }
}

void mot_rencoder_right()
{
    // if (digitalRead(encodPinBR) == HIGH) {
    if (digitalRead(encA2) > digitalRead(encB2))
    {
        wheel_pulse_count_right = wheel_pulse_count_right - 1;
    }
    else
    {
        wheel_pulse_count_right = wheel_pulse_count_right + 1;
    }
}
