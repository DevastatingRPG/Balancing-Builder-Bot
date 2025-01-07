// PID Library
#include <PID_v1.h>
#include <Servo.h>

// MPU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <SoftwareSerial.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Define SoftwareSerial pins
SoftwareSerial bluetooth(A1, A0); // RX, TX

// Create Servo objects
Servo servo1;
Servo servo2;

// Bluetooth
const int RX = A1; // RX: Bluetooth receive pin
const int TX = A0; // TX: Bluetooth transmit pin

// Motor A
const int ENA = 6;   // ENA: Enable pin for Motor A
int motor1Pin1 = A2; // motor1Pin1: Control pin 1 for Motor A
int motor1Pin2 = A3; // motor1Pin2: Control pin 2 for Motor A

// Motor B
const int ENB = 5;  // ENB: Enable pin for Motor B
int motor2Pin1 = 4; // motor2Pin1: Control pin 1 for Motor B
int motor2Pin2 = 9; // motor2Pin2: Control pin 2 for Motor B

// Encoders
const int encA1 = 2; // encA1: Encoder 1 channel A
const int encB1 = 7; // encB1: Encoder 1 channel B
const int encA2 = 3; // encA2: Encoder 2 channel A
const int encB2 = 8; // encB2: Encoder 2 channel B

MPU6050 mpu; // mpu: MPU6050 sensor object

// MPU control/status vars
bool dmpReady = false;  // dmpReady: Flag to indicate if DMP is ready
uint8_t mpuIntStatus;   // mpuIntStatus: Interrupt status byte from MPU
uint8_t devStatus;      // devStatus: Return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // fifoBuffer: FIFO storage buffer

// orientation/motion vars
Quaternion q;        // q: Quaternion container [w, x, y, z]
VectorFloat gravity; // gravity: Gravity vector [x, y, z]
float ypr[3];        // ypr: Yaw/Pitch/Roll container and gravity vector [yaw, pitch, roll]
VectorInt16 gy;      // gy: Gyro sensor measurements [x, y, z]

volatile bool mpuInterrupt = false; // mpuInterrupt: Flag to indicate MPU interrupt

// Encoder Pulse Counts
int wheel_pulse_count_left = 0;  // wheel_pulse_count_left: Pulse count for left wheel encoder
int wheel_pulse_count_right = 0; // wheel_pulse_count_right: Pulse count for right wheel encoder

#define PID_MIN_LIMIT -180          // PID_MIN_LIMIT: Minimum limit for PID output
#define PID_MAX_LIMIT 180           // PID_MAX_LIMIT: Maximum limit for PID output
#define PID_SAMPLE_TIME_IN_MILLI 10 // PID_SAMPLE_TIME_IN_MILLI: PID sample time in milliseconds

#define SETPOINT_PITCH_ANGLE_OFFSET 0 // SETPOINT_PITCH_ANGLE_OFFSET: Setpoint pitch angle offset

#define MIN_ABSOLUTE_SPEED 0 // MIN_ABSOLUTE_SPEED: Minimum motor speed

double Setpoint_Pitch, Input_Pitch, Output_Pitch, Error_Pitch; // PID variables for pitch control
double Setpoint_Yaw, Input_Yaw, Output_Yaw;                    // PID variables for yaw control
double Setpoint_Pos, Input_Pos, Output_Pos, Error_Pos;         // PID variables for position control

/**
 * Struct: PIDParams
 * Description: Structure to hold PID parameters.
 */
struct PIDParams
{
    float Kp;  // Kp: Proportional gain
    float Ki;  // Ki: Integral gain
    float Kd;  // Kd: Derivative gain
    float _kp; // _kp: Internal proportional gain
    float _ki; // _ki: Internal integral gain
    float _kd; // _kd: Internal derivative gain

    /**
     * Constructor: PIDParams
     * Input: Kp - Proportional gain
     *        Ki - Integral gain
     *        Kd - Derivative gain
     * Logic: Initializes the PID parameters and internal variables.
     */
    PIDParams(float Kp, float Ki, float Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd)
    {
        _kp = Kp;
        _kd = Kd;
        _ki = Ki;
    }
};

// PID parameter instances
PIDParams pitchCon(25, 300, 2.3);      // pitchCon: PID parameters for pitch control
PIDParams posAgg(0.015, 0, 0.003);     // posAgg: PID parameters for position control
PIDParams posBoost(0.015, 0.5, 0.003); // posBoost
PIDParams yaw(0.5, 0.5, 0);            // yaw: PID parameters for yaw control

// PID controller instances
PID pitchPID(&Input_Pitch, &Output_Pitch, &Setpoint_Pitch, pitchCon._kp, pitchCon._ki, pitchCon._kd, DIRECT);
PID posPID(&Input_Pos, &Output_Pos, &Setpoint_Pos, posAgg._kp, posAgg._ki, posAgg._kd, DIRECT);
PID yawPID(&Input_Yaw, &Output_Yaw, &Setpoint_Yaw, yaw._kp, yaw._ki, yaw._kd, DIRECT);

/**
 * Function Name: rotateMotor
 * Input: speed1 - Speed for motor 1
 *        speed2 - Speed for motor 2
 * Logic: Controls the direction and speed of the motors based on the input speeds.
 * Example Call: rotateMotor(100, 100);
 */
void rotateMotor(int speed1, int speed2, int turning = 0)
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

    speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 180);
    speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 180);

    if (turning == 1)
    {
        speed1 += 30;
        speed2 -= 30;
    }
    else if (turning == 2)
    {
        speed2 += 30;
        speed1 -= 30;
    }

    analogWrite(ENA, speed1 * 0.89);
    analogWrite(ENB, speed2);
}

/**
 * Function Name: mot_rencoder_left
 * Logic: Interrupt service routine for the left motor encoder. Updates the pulse count based on the encoder signals.
 * Example Call: attachInterrupt(digitalPinToInterrupt(encA1), mot_rencoder_left, RISING);
 */
void mot_rencoder_left()
{
    if (digitalRead(encA1) > digitalRead(encB1))
    {
        wheel_pulse_count_left = wheel_pulse_count_left + 1;
    }
    else
    {
        wheel_pulse_count_left = wheel_pulse_count_left - 1;
    }
}

/**
 * Function Name: mot_rencoder_right
 * Logic: Interrupt service routine for the right motor encoder. Updates the pulse count based on the encoder signals.
 * Example Call: attachInterrupt(digitalPinToInterrupt(encA2), mot_rencoder_right, RISING);
 */
void mot_rencoder_right()
{
    if (digitalRead(encA2) > digitalRead(encB2))
    {
        wheel_pulse_count_right = wheel_pulse_count_right - 1;
    }
    else
    {
        wheel_pulse_count_right = wheel_pulse_count_right + 1;
    }
}

/**
 * Function Name: setupPID
 * Logic: Initializes the PID controllers with the setpoints, output limits, and sample times.
 * Example Call: setupPID();
 */
void setupPID()
{
    Setpoint_Pitch = SETPOINT_PITCH_ANGLE_OFFSET;
    pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

    Setpoint_Yaw = 0;
    yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    yawPID.SetMode(AUTOMATIC);
    yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

    Setpoint_Pos = 0;
    posPID.SetOutputLimits(-2, 2);
    posPID.SetMode(AUTOMATIC);
    posPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

/**
 * Function Name: setupMotors
 * Logic: Configures the motor control pins as outputs and initializes the motors to a stopped state.
 * Example Call: setupMotors();
 */
void setupMotors()
{
    // Motor A
    pinMode(ENA, OUTPUT);
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);

    // Motor B
    pinMode(ENB, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    rotateMotor(0, 0, 0);
}

void setupManipulator()
{
    // Attach the servos to the respective pins
    servo1.attach(10);
    servo2.attach(11);
    servo1.write(0);
    servo2.write(0);
}

/**
 * Function Name: setupMPU
 * Logic: Initializes the MPU6050 sensor, sets the offsets, and enables the DMP.
 * Example Call: setupMPU();
 */
void setupMPU()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_ILTIN_FASTWIRE
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

    if (devStatus == 0)
    {
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
    }
    else
    {
        // ERROR!
    }
}

/**
 * Function Name: setupEncoders
 * Logic: Configures the encoder pins as inputs with pull-up resistors and attaches interrupt service routines.
 * Example Call: setupEncoders();
 */
void setupEncoders()
{
    // Encoder 1
    pinMode(encA1, INPUT_PULLUP);
    digitalWrite(encA1, HIGH);
    attachInterrupt(digitalPinToInterrupt(encA1), mot_rencoder_left, RISING);

    pinMode(encB1, INPUT_PULLUP);
    digitalWrite(encB1, HIGH);

    // Encoder 2
    pinMode(encA2, INPUT_PULLUP);
    digitalWrite(encA2, HIGH);
    attachInterrupt(digitalPinToInterrupt(encA2), mot_rencoder_right, RISING);

    pinMode(encB2, INPUT_PULLUP);
    digitalWrite(encB2, HIGH);
}

/**
 * Function Name: setup
 * Logic: Calls the setup functions for motors, MPU6050, PID controllers, and encoders.
 * Example Call: setup();
 */
void setup()
{
    // Motor Setup
    setupMotors();
    // MPU6050 Setup
    setupMPU();
    // PID Setup
    setupPID();
    // Encoder Setup
    setupEncoders();
    // Manipulator setup
    // setupManipulator();

    Serial.begin(115200);
    Serial.println("Helo");
    bluetooth.begin(9600); // For HC-05 (default baud rate is 9600)
    bluetooth.println("Wagwan");
    // tone(12, 1000, 100);
}

bool stopping = false; // stopping: Flag to indicate if the robot is stopping
bool moving = false;
bool boost = false;
float turning = 0;

void loop()
{
    if (!dmpReady)
    {
        return;
    }

    float vel = 0; // vel: Velocity for motor control

    if (bluetooth.available() > 0)
    {
        uint8_t btData = bluetooth.read();
        // if (btData != '\n' && btData != '\r') {
        // Serial.print("Received from Bluetooth: ");
        // Serial.println(btData);
        // if (bluetooth.isListening())
        //   Serial.println("portOne is listening!");
        // if (bluetooth.overflow())
        //   Serial.println("portOne overflow!");

        switch (btData)
        {
        case 1:
            Setpoint_Pos = 350;
            wheel_pulse_count_left = 0;
            wheel_pulse_count_right = 0;
            tone(12, 1000, 100);
            // moving = true;
            turning = 0;
            boost = false;
            bluetooth.flush();
            break;
        case 4:
            Setpoint_Pos = -350;

            // Setpoint_Pitch = -0.02; // Move backward
            wheel_pulse_count_left = 0;
            wheel_pulse_count_right = 0;
            tone(12, 1000, 100);
            // moving = true;
            turning = 0;
            boost = false;
            bluetooth.flush();
            break;
        case 2:
            moving = true;
            wheel_pulse_count_left = 0;
            wheel_pulse_count_right = 0;
            tone(12, 1000, 100);
            turning = 1; // Rotate left
            boost = false;
            bluetooth.flush();
            break;
        case 3:
            moving = true;
            wheel_pulse_count_left = 0;
            wheel_pulse_count_right = 0;
            tone(12, 1000, 100);
            turning = 2; // Rotate right
            boost = false;
            bluetooth.flush();
            break;
        case 9:
            moving = false; // Stop
            wheel_pulse_count_left = 0;
            wheel_pulse_count_right = 0;
            turning = 0;
            boost = false;
            tone(12, 1000, 100);
            bluetooth.flush();
            break;
        case 5:
            // servo1.write(30); // Arm Close
            for (int angle = 0; angle <= 30; angle++)
            {
                servo1.write(angle);
                // servo2.write(angle);
                delay(15); // Small delay for smooth movement
            }
            bluetooth.flush();
            break;
        case 6:
            // servo1.write(0); // Arm Open
            for (int angle = 30; angle >= 0; angle--)
            {
                servo1.write(angle);
                // servo2.write(angle);
                delay(15); // Small delay for smooth movement
            }
            bluetooth.flush();
            break;
        case 7:
            // servo2.write(0); // Arm Up
            for (int angle = 0; angle <= 40; angle++)
            {
                // servo1.write(angle);
                servo2.write(angle);
                delay(15); // Small delay for smooth movement
            }
            bluetooth.flush();
            break;
        case 8:
            // servo2.write(40); // Arm Down
            for (int angle = 40; angle >= 0; angle--)
            {
                // servo1.write(angle);
                servo2.write(angle);
                delay(15); // Small delay for smooth movement
            }
            bluetooth.flush();
            break;
        case 0:
            Setpoint_Pos = 500;
            wheel_pulse_count_left = 0;
            wheel_pulse_count_right = 0;
            tone(12, 1000, 100);
            // moving = true;
            turning = 0;
            bluetooth.flush();
            boost = true;
            break;
        case 10:
            Setpoint_Pos = -500;

            // Setpoint_Pitch = -0.02; // Move backward
            wheel_pulse_count_left = 0;
            wheel_pulse_count_right = 0;
            tone(12, 1000, 100);
            // moving = true;
            turning = 0;
            bluetooth.flush();

            boost = true;
            break;
        }
        bluetooth.println(btData);
    }

    if (boost)
    {
        // posPID.SetTunings(posBoost._kp, posBoost._ki, posBoost._kd);
        posPID.SetOutputLimits(PID_MIN_LIMIT - 50, PID_MAX_LIMIT + 50);
    }
    else
    {
        // posPID.SetTunings(posAgg._kp, posAgg._ki, posAgg._kd);
        posPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
    }

    Error_Pos = (wheel_pulse_count_left + wheel_pulse_count_right) / 2;

    // Only apply Position PID if Error is high to avoid overreaction and not moving
    if (!moving && abs(Error_Pos) > 10)
    {
        Input_Pos = Error_Pos;
        posPID.Compute(true);
        Setpoint_Pitch = -Output_Pos;
        stopping = true;
        pitchPID.SetTunings(pitchCon._kp, pitchCon.Ki, pitchCon._kd);
    }
    else if (!moving)
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
        mpu.dmpGetGyro(&gy, fifoBuffer);

        Input_Yaw = gy.z;                  // rotation rate in degrees per second
        Input_Pitch = ypr[1] * 180 / M_PI; // angle in degree

        pitchPID.Compute(true);
        yawPID.Compute(true);

        vel += Output_Pitch;
    }

    // Reduce vel if position PID is applicable
    if (stopping)
    {
        vel *= 0.8;
    }
    if (moving)
    {
        vel += 0.7;
    }
    // if (boost)
    // {
    //   vel = (vel / fabs(vel)) * (fabs(vel) + 20);
    // }
    rotateMotor(vel, vel, turning);
    // Serial.println(".");
    delay(7);
}