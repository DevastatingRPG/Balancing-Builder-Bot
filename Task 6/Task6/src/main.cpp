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
SoftwareSerial bluetooth(A1, A0);  // RX, TX

// Create Servo objects
Servo servo1;  // Claw
Servo servo2;  // Up down Arm

// Bluetooth
const int RX = A1;  // RX: Bluetooth receive pin
const int TX = A0;  // TX: Bluetooth transmit pin

// Motor A
const int ENA = 6;    // ENA: Enable pin for Motor A
int motor1Pin1 = A2;  // motor1Pin1: Control pin 1 for Motor A
int motor1Pin2 = A3;  // motor1Pin2: Control pin 2 for Motor A

// Motor B
const int ENB = 5;   // ENB: Enable pin for Motor B
int motor2Pin1 = 4;  // motor2Pin1: Control pin 1 for Motor B
int motor2Pin2 = 9;  // motor2Pin2: Control pin 2 for Motor B

// Encoders
const int encA1 = 2;  // encA1: Encoder 1 channel A
const int encB1 = 7;  // encB1: Encoder 1 channel B
const int encA2 = 3;  // encA2: Encoder 2 channel A
const int encB2 = 8;  // encB2: Encoder 2 channel B

MPU6050 mpu;  // mpu: MPU6050 sensor object

// MPU control/status vars
bool dmpReady = false;   // dmpReady: pickup_phase to indicate if DMP is ready
uint8_t mpuIntStatus;    // mpuIntStatus: Interrupt status byte from MPU
uint8_t devStatus;       // devStatus: Return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64];  // fifoBuffer: FIFO storage buffer

// orientation/motion vars
Quaternion q;         // q: Quaternion container [w, x, y, z]
VectorFloat gravity;  // gravity: Gravity vector [x, y, z]
float ypr[3];         // ypr: Yaw/Pitch/Roll container and gravity vector [yaw, pitch, roll]
VectorInt16 gy;       // gy: Gyro sensor measurements [x, y, z]

volatile bool mpuInterrupt = false;  // mpuInterrupt: pickup_phase to indicate MPU interrupt

// Encoder Pulse Counts
int wheel_pulse_count_left = 0;   // wheel_pulse_count_left: Pulse count for left wheel encoder
int wheel_pulse_count_right = 0;  // wheel_pulse_count_right: Pulse count for right wheel encoder

#define PID_MIN_LIMIT -255           // PID_MIN_LIMIT: Minimum limit for PID output
#define PID_MAX_LIMIT 255            // PID_MAX_LIMIT: Maximum limit for PID output
#define PID_SAMPLE_TIME_IN_MILLI 10  // PID_SAMPLE_TIME_IN_MILLI: PID sample time in milliseconds

#define SETPOINT_PITCH_ANGLE_OFFSET 0  // SETPOINT_PITCH_ANGLE_OFFSET: Setpoint pitch angle offset
// #define SETPOINT_PITCH_ANGLE_OFFSETR 0.666

#define MIN_ABSOLUTE_SPEED 0  // MIN_ABSOLUTE_SPEED: Minimum motor speed

int rated = 10000;
double Setpoint_Pitch, Input_Pitch, Output_Pitch, Error_Pitch;  // PID variables for pitch control
double Setpoint_Pos, Input_Pos, Output_Pos, Error_Pos;          // PID variables for position control

/**
 * Struct: PIDParams
 * Description: Structure to hold PID parameters.
 */
struct PIDParams {
  float Kp;   // Kp: Proportional gain
  float Ki;   // Ki: Integral gain
  float Kd;   // Kd: Derivative gain
  float _kp;  // _kp: Internal proportional gain
  float _ki;  // _ki: Internal integral gain
  float _kd;  // _kd: Internal derivative gain

  /**
   * Constructor: PIDParams
   * Input: Kp - Proportional gain
   *        Ki - Integral gain
   *        Kd - Derivative gain
   * Logic: Initializes the PID parameters and internal variables.
   */
  PIDParams(float Kp, float Ki, float Kd)
      : Kp(Kp), Ki(Ki), Kd(Kd) {
    _kp = Kp;
    _kd = Kd;
    _ki = Ki;
  }
};

// PID parameter instances
PIDParams pitchAfterTurn(58, 500, 3.5);
PIDParams pitchCon(33, 300, 2.4);  // pitchCon: PID parameters for pitch control

PIDParams posAgg(0.005, 0, 0.003);  // posAgg: PID parameters for position control
PIDParams posMove(0.01, 0, 0.003);  // posAgg: PID parameters for position control

PIDParams whileMoving(30, 350, 2);

// PID controller instances
PID pitchPID(&Input_Pitch, &Output_Pitch, &Setpoint_Pitch, pitchCon._kp, pitchCon._ki, pitchCon._kd, DIRECT);
PID posPID(&Input_Pos, &Output_Pos, &Setpoint_Pos, posAgg._kp, posAgg._ki, posAgg._kd, DIRECT);

int picked = 0;
unsigned long timer = millis();

/**
 * Function Name: rotateMotor
 * Input: speed1 - Speed for motor 1
 *        speed2 - Speed for motor 2
 * Logic: Controls the direction and speed of the motors based on the input speeds.
 * Example Call: rotateMotor(100, 100);
 */
void rotateMotor(int speed1, int speed2, int turning = 0) {
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else if (speed1 >= 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  if (speed2 < 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (speed2 >= 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }

  speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
  speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;

  speed1 *= 1.04;
  speed2 *= 1;

  speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, PID_MAX_LIMIT);
  speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, PID_MAX_LIMIT);

  analogWrite(ENA, speed1);
  analogWrite(ENB, speed2);
}

/**
 * Function Name: mot_rencoder_left
 * Logic: Interrupt service routine for the left motor encoder. Updates the pulse count based on the encoder signals.
 * Example Call: attachInterrupt(digitalPinToInterrupt(encA1), mot_rencoder_left, RISING);
 */
void mot_rencoder_left() {
  if (digitalRead(encA1) > digitalRead(encB1)) {
    wheel_pulse_count_left = wheel_pulse_count_left + 1;
  } else {
    wheel_pulse_count_left = wheel_pulse_count_left - 1;
  }
}

/**
 * Function Name: mot_rencoder_right
 * Logic: Interrupt service routine for the right motor encoder. Updates the pulse count based on the encoder signals.
 * Example Call: attachInterrupt(digitalPinToInterrupt(encA2), mot_rencoder_right, RISING);
 */
void mot_rencoder_right() {
  if (digitalRead(encA2) > digitalRead(encB2)) {
    wheel_pulse_count_right = wheel_pulse_count_right - 1;
  } else {
    wheel_pulse_count_right = wheel_pulse_count_right + 1;
  }
}

/**
 * Function Name: setupPID
 * Logic: Initializes the PID controllers with the setpoints, output limits, and sample times.
 * Example Call: setupPID();
 */
void setupPID() {
  Setpoint_Pitch = SETPOINT_PITCH_ANGLE_OFFSET;
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  Setpoint_Pos = 0;
  posPID.SetOutputLimits(-3, 3);
  posPID.SetMode(AUTOMATIC);
  posPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

/**
 * Function Name: setupMotors
 * Logic: Configures the motor control pins as outputs and initializes the motors to a stopped state.
 * Example Call: setupMotors();
 */
void setupMotors() {
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

void setupManipulator() {
  // Attach the servos to the respective pins
  servo1.attach(10);
  servo2.attach(11);
  servo1.write(180);
  servo2.write(0);
}

/**
 * Function Name: setupMPU
 * Logic: Initializes the MPU6050 sensor, sets the offsets, and enables the DMP.
 * Example Call: setupMPU();
 */
void setupMPU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
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

  if (devStatus == 0) {
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  } else {
    // ERROR!
  }
}

/**
 * Function Name: setupEncoders
 * Logic: Configures the encoder pins as inputs with pull-up resistors and attaches interrupt service routines.
 * Example Call: setupEncoders();
 */
void setupEncoders() {
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
void setup() {
  // Motor Setup
  setupMotors();
  // MPU6050 Setup
  setupMPU();
  // PID Setup
  setupPID();
  // Encoder Setup
  setupEncoders();
  // Manipulator setup
  setupManipulator();

  Serial.begin(115200);
  Serial.println("Helo");
  bluetooth.begin(9600);  // For HC-05 (default baud rate is 9600)
  bluetooth.println("Wagwan");
  // tone(12, 1000, 100);
}

bool stopping = false;  // stopping: pickup_phase to indicate if the robot is stopping
bool moving = false;
int boost = 0;
int turning = 0;

int targetPosition1 = 180;
int targetPosition2 = 10;
unsigned long lastServoMoveTime = 0;
const int servoStepDelay = 20;

unsigned long turnStart;
unsigned long turnEnd;

bool rightturn = 0;

/**
 * Function Name: moveServo
 * Logic: Moves the servos to their target positions in small steps to ensure smooth movement. The function checks if the current time exceeds the last move time by the step delay, and if so, it updates the servo positions.
 * Example Call: moveServo();
 */
void moveServo() {
  unsigned long currentTime = millis();
  if (currentTime - lastServoMoveTime >= servoStepDelay) {
    lastServoMoveTime = currentTime;
    if (targetPosition1 != -1) {
      int currentPosition = servo1.read();
      if (currentPosition < targetPosition1) {
        servo1.write(min(currentPosition + 5, targetPosition1));
      } else if (currentPosition > targetPosition1) {
        servo1.write(max(currentPosition - 5, targetPosition1));
      }
      if (currentPosition == targetPosition1) {
        targetPosition1 = -1;  // Movement complete
      }
    }
    if (targetPosition2 != -1) {
      int currentPosition = servo2.read();
      if (currentPosition < targetPosition2) {
        servo2.write(min(currentPosition + 5, targetPosition2));
      } else if (currentPosition > targetPosition2) {
        servo2.write(max(currentPosition - 5, targetPosition2));
      }
      if (currentPosition == targetPosition2) {
        targetPosition2 = -1;  // Movement complete
      }
    }
  }
}

/**
 * Function Name: safety
 * Logic: Monitors the pitch angle and rate of change to ensure the robot's safety. If the pitch angle exceeds a threshold or changes too rapidly, the robot is stopped.
 * Example Call: safety();
 */
void safety() {
  Input_Pitch = ypr[1] * 180 / M_PI;  // angle in degree
  float rate = 0;
  float lastPitch = 0;
  float currPitch = Input_Pitch;

  unsigned long lastTime = 0;
  unsigned long currTime = millis();

  if (currTime - lastTime >= 500) {
    rate = abs(currPitch - lastPitch);
    if (rate > 8 || Input_Pitch > 10) {
      Setpoint_Pos = 0;
      Setpoint_Pitch = 0;
      moving = false;  // Stop
      wheel_pulse_count_left = 0;
      wheel_pulse_count_right = 0;
      turning = 0;
      stopping = false;
      boost = 0;
    }
    lastTime = currTime;
    lastPitch = currPitch;
  }
}

int pickup_phase = 0;
int drop_phase = 0;
unsigned long movement = 0;

void loop() {
  if (!dmpReady) {
    return;
  }

  float vel = 0;  // vel: Velocity for motor control
  int velL, velR;

  if (bluetooth.available() > 0) {
    uint8_t btData = bluetooth.read();
    switch (btData) {
      case 0:  // Boost Forward
        movement = millis();
        pitchPID.SetTunings(whileMoving._kp, whileMoving._kd, whileMoving._ki);
        Setpoint_Pos = 1000;
        wheel_pulse_count_left = 0;
        wheel_pulse_count_right = 0;
        rated = 500;
        turning = 0;
        stopping = false;
        bluetooth.flush();
        boost = 1;
        break;
      case 1:  // Forward
        movement = millis();
        pitchPID.SetTunings(whileMoving._kp, whileMoving._kd, whileMoving._ki);
        Setpoint_Pos = 300;
        stopping = false;
        rated = 500;
        wheel_pulse_count_left = 0;
        wheel_pulse_count_right = 0;
        turning = 0;
        bluetooth.flush();
        boost = 0;

        break;

      case 4:  // Backward
        movement = millis();
        pitchPID.SetTunings(whileMoving._kp, whileMoving._kd, whileMoving._ki);
        Setpoint_Pos = -300;
        wheel_pulse_count_left = 0;
        stopping = false;
        rated = 500;
        wheel_pulse_count_right = 0;
        turning = 0;
        bluetooth.flush();
        boost = 0;

        break;
      case 10:  // Boost Backward
        movement = millis();
        pitchPID.SetTunings(whileMoving._kp, whileMoving._kd, whileMoving._ki);
        Setpoint_Pos = -1000;
        wheel_pulse_count_left = 0;
        stopping = false;
        rated = 500;
        wheel_pulse_count_right = 0;
        turning = 0;
        bluetooth.flush();
        boost = -1;

        break;

      case 2:  // Rotate left
        stopping = false;
        rated = 200;
        turning = 1;
        turnStart = millis();
        bluetooth.flush();
        boost = 0;

        break;

      case 3:  // Rotate right
        rightturn = 1;
        rated = 200;
        turning = -1;
        turnStart = millis();
        bluetooth.flush();
        boost = 0;
        stopping = false;

        break;

      case 9:
        Setpoint_Pos = 0;
        moving = false;  // Stop
        wheel_pulse_count_left = 0;
        wheel_pulse_count_right = 0;
        turning = 0;
        stopping = false;

        bluetooth.flush();
        boost = 0;

        break;

      case 6:  // pickup
        pickup_phase = 1;
        targetPosition1 = 80;
        picked = 1;
        bluetooth.flush();
        break;

      case 5:  // dropoff
        picked = 0;
        targetPosition2 = 50;
        drop_phase = 1;

        bluetooth.flush();
        break;
      case 11:
        boost = 0;
        break;
      case 12:
        stopping = true;
        boost = 0;
        break;
      case 13:  // Buzzer
        tone(12, 1000, 1000);

      case 14:  // Slow Forward
        movement = millis();
        pitchPID.SetTunings(whileMoving._kp, whileMoving._kd, whileMoving._ki);
        Setpoint_Pos = 150;
        stopping = false;

        wheel_pulse_count_left = 0;
        wheel_pulse_count_right = 0;
        turning = 0;
        bluetooth.flush();
        boost = 0;

        break;

      case 15:  // Slow Backward
        movement = millis();
        pitchPID.SetTunings(whileMoving._kp, whileMoving._kd, whileMoving._ki);
        Setpoint_Pos = -150;
        wheel_pulse_count_left = 0;
        stopping = false;

        wheel_pulse_count_right = 0;
        turning = 0;
        bluetooth.flush();
        boost = 0;

        break;

      default:
        break;
    }
  }
  moveServo();

  switch (pickup_phase) {
    case 1:
      if (servo1.read() <= 170) {
        targetPosition2 = 50;
        pickup_phase = 2;
      }
      break;
    case 2:
      if (servo2.read() == 50) {
        targetPosition1 = 180;
        pickup_phase = 3;
      }
      break;
    case 3:
      if (servo1.read() == 180) {
        targetPosition2 = 10;
        pickup_phase = 0;
      }
      break;

    default:
      break;
  }

  switch (drop_phase) {
    case 1:
      if (servo2.read() == 50) {
        targetPosition1 = 140;
        drop_phase = 2;
      }
      break;
    case 2:
      if (servo1.read() == 140) {
        targetPosition2 = 10;
        targetPosition1 = 180;
        drop_phase = 0;
      }
      break;
    default:
      break;
  }

  safety();

  if (millis() - movement > 200) {
    pitchPID.SetTunings(pitchCon._kp, pitchCon._ki, pitchCon._kd);
    movement = 0;
  }

  Error_Pos = (wheel_pulse_count_left + wheel_pulse_count_right) / 2;

  // More stable when stop
  if (Setpoint_Pos == 0 || abs(Error_Pos - Setpoint_Pos) < 30) {
    posPID.SetTunings(posAgg._kp, posAgg._ki, posAgg._kd);
    boost = 0;
  } else {
    posPID.SetTunings(posMove._kp, posMove._ki, posMove._kd);
  }

  // Only apply Position PID if Error is high to avoid overreaction and not moving
  if (turning) {
    Setpoint_Pitch = SETPOINT_PITCH_ANGLE_OFFSET;
    stopping = true;
  } else {
    if (abs(Error_Pos) > 20) {
      Input_Pos = Error_Pos;
      posPID.Compute(true);
      Setpoint_Pitch = -Output_Pos + SETPOINT_PITCH_ANGLE_OFFSET;
      stopping = true;
    } else {
      Setpoint_Pitch = SETPOINT_PITCH_ANGLE_OFFSET;
      stopping = false;
    }
  }

  if (millis() - turnEnd > 100) {
    pitchPID.SetTunings(pitchCon._kp, pitchCon.Ki, pitchCon._kd);
  }

  // read a packet from FIFO. Get the Latest packet
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Input_Pitch = ypr[1] * 180 / M_PI;  // angle in degree
    pitchPID.Compute(true);

    vel += Output_Pitch;
  }

  // Increase vel if boosting
  switch (boost) {
    case 1:
      if (vel > 0)
        vel *= 1.8;
      break;
    case -1:
      if (vel < 0)
        vel *= 1.8;
      break;
  }

  if (turning) {
    if ((millis() - turnStart) < 5) {
      rotateMotor(turning * 150, -turning * 150);
    }
    // End turn after duration is over
    else {
      turning = 0;
      wheel_pulse_count_left = 0;
      wheel_pulse_count_right = 0;
      Setpoint_Pos = 0;
      Setpoint_Pitch = SETPOINT_PITCH_ANGLE_OFFSET;
      turnEnd = millis();

      pitchPID.SetTunings(pitchCon._kp, pitchCon.Ki, pitchCon._kd);
      delay(50);
    }
  } else {
    // Apply different PID to turn end
    if (millis() - turnEnd < 100) {
      pitchPID.SetTunings(pitchAfterTurn._kp, pitchAfterTurn._ki, pitchAfterTurn._kd);
      rotateMotor(vel * 1.2, vel * 1.2, turning);
      wheel_pulse_count_left = 0;
      wheel_pulse_count_right = 0;
    } else {
      rotateMotor(vel * 1.2, vel * 1.2, turning);
    }
  }
  delay(7);
}