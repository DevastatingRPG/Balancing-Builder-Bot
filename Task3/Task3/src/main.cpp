#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
// #include <PID_v1.h>
#include <SoftwareSerial.h>

// Define SoftwareSerial pins
SoftwareSerial bluetooth(A1, A0); // RX, TX

MPU6050 mpu(Wire);
unsigned long timer = 0;

float complementaryAlpha = 0.95; // Complementary filter weight
float angleFilteredX = 0.0;      // Filtered angle for X-axis
float angleFilteredY = 0.0;      // Filtered angle for Y-axis
float dt = 0.0;                  // Time difference

unsigned long lastPosTime = 0;             // Stores the last timestamp for position error calculation
int lastPulseLeft = 0, lastPulseRight = 0; // Stores the last pulse counts for wheels

const int inputPin1 = A2; // Pin 15 of L293D IC
const int inputPin2 = A3; // Pin 10 of L293D IC
                          // Motor B
const int inputPin3 = 9;  // Pin  7 of L293D IC
const int inputPin4 = 4;  // Pin  2 of L293D IC
int EN1 = 6;              // Pin 1 of L293D IC
int EN2 = 5;

float setpoint_pitch = 0.0;
float setpoint_pos = 0.0;

float pitch_error = 0.0;
float pos_error = 0.0;

float pitch_gain = 0.0;
float pos_gain = 0.0;

// Define encoder pins
#define encodPinAR 7 // Encoder A pin (C2) // A1
#define encodPinBR 2 // Encoder B pin (C1) // A2//2

#define encodPinAL 8 // Encoder A pin (C2) ------ working
#define encodPinBL 3 // Encoder B pin (C1) ------ working

// Variable for wheel angle
int wheel_pulse_count_left = 0;
int wheel_pulse_count_right = 0;

double roll, pitch, yaw;

class PID
{
private:
    float kp, ki, kd;
    float prev_err, integral;

public:
    PID(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), prev_err(0), integral(0) {}

    float calculate(float err, float dt)
    {
        float proportional = kp * err;
        integral += ki * err * dt;
        float derivative = kd * (err - prev_err) / dt;

        float output = proportional + integral + derivative;

        return output;
    }
};

PID pitchPID(6.0, 0.0, 0.0);
PID posPID(1.0, 0.0, 0.0);

void setup()
{
    Serial.begin(9600);
    bluetooth.begin(9600); // For HC-05 (default baud rate is 9600)

    Serial.println("Bluetooth is ready!");

    // MPU 6050
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0)
    {
    } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets(); // gyro and accelero
    Serial.println("Done!\n");
    timer = millis();

    // MOTOR
    pinMode(EN1, OUTPUT); // where the motor is connected to
    pinMode(EN2, OUTPUT); // where the motor is connected to
    pinMode(inputPin1, OUTPUT);
    pinMode(inputPin2, OUTPUT);
    pinMode(inputPin3, OUTPUT);
    pinMode(inputPin4, OUTPUT);
    analogWrite(EN1, 100); // sets the motors speed
    analogWrite(EN2, 100);

    // ENCODER
    //------Encoder HW Interrupt setup-----//
    pinMode(8, INPUT_PULLUP);
    digitalWrite(8, HIGH);
    // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

    pinMode(3, INPUT_PULLUP);
    digitalWrite(3, HIGH);
    attachInterrupt(digitalPinToInterrupt(3), mot_rencoder_left, RISING);
    // ---------------------------------------------- ///

    //------Encoder HW Interrupt setup-----//
    pinMode(7, INPUT_PULLUP); // 10
    digitalWrite(7, HIGH);
    // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

    pinMode(2, INPUT_PULLUP); // 11
    digitalWrite(2, HIGH);
    attachInterrupt(digitalPinToInterrupt(2), mot_rencoder_right, RISING);
    // ---------------------------------------------- ///
}

void loop()
{
    // // Check for data from Bluetooth (furthur remote use)
    // if (bluetooth.available()) {
    //   char btData = bluetooth.read();

    //   // Filter out newline and carriage return characters
    //   if (btData != '\n' && btData != '\r') {
    //     Serial.print("Received from Bluetooth: ");
    //     Serial.println(btData);
    //   }
    // }

    // UPDATE ANGLES
    mpu.update();
    unsigned long currentMillis = millis();
    dt = (currentMillis - timer) / 1000.0; // Convert to seconds
    timer = currentMillis;
    float accelAngleX = mpu.getAccAngleX();
    float accelAngleY = mpu.getAccAngleY();
    float gyroRateX = mpu.getGyroX(); // deg/s
    float gyroRateY = mpu.getGyroY(); // deg/s
    // Complementary filter for X-axis
    angleFilteredX = complementaryAlpha * (angleFilteredX + gyroRateX * dt) +
                     (1 - complementaryAlpha) * accelAngleX;

    // Complementary filter for Y-axis
    angleFilteredY = complementaryAlpha * (angleFilteredY + gyroRateY * dt) +
                     (1 - complementaryAlpha) * accelAngleY;

    // Print filtered angles
    Serial.print(F("Filtered Angle X: "));
    Serial.print(angleFilteredX);
    Serial.print(F("\tFiltered Angle Y: "));
    Serial.println(angleFilteredY);
    delay(10);

    float dt = 0.01;

    pitch_error = setpoint_pitch - angleFilteredY;
    pos_error = poserror();

    float pitch_gain = pitchPID.calculate(pitch_error, dt);
    float pos_gain = posPID.calculate(pos_error, dt);

    float vel = pitch_gain - pos_gain;

    int motorSpeed = constrain(map(abs(vel), 0, 100, 0, 255), 0, 255); // Scale velocity to PWM range

    if (vel > 0)
    {
        digitalWrite(inputPin1, HIGH);
        digitalWrite(inputPin2, LOW);
        digitalWrite(inputPin3, LOW);
        digitalWrite(inputPin4, HIGH);
    }
    else
    {
        digitalWrite(inputPin1, LOW);
        digitalWrite(inputPin2, HIGH);
        digitalWrite(inputPin3, HIGH);
        digitalWrite(inputPin4, LOW);
    }

    // Set motor speeds
    analogWrite(EN1, motorSpeed);
    analogWrite(EN2, motorSpeed);

    Serial.print(wheel_pulse_count_left);
    Serial.print(" ; ");
    Serial.println(wheel_pulse_count_right);

    double pitch = angleFilteredY;
    bluetooth.println("pitch:");
    bluetooth.println(pitch);
    bluetooth.println("");

    bluetooth.println("Pitch error:");
    bluetooth.println(pitch_error);
    bluetooth.println("");

    bluetooth.println("Pos error:");
    bluetooth.println(pos_error);
    bluetooth.println("");

    bluetooth.println("Pitch gain:");
    bluetooth.println(pitch_gain);
    bluetooth.println("");

    bluetooth.println("Pos gain:");
    bluetooth.println(pos_gain);
    bluetooth.println("");
}

// ISR for motor encoder
void mot_rencoder_left()
{
    // if (digitalRead(encodPinBR) == HIGH) {
    if (digitalRead(encodPinBL) > digitalRead(encodPinAL))
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
    if (digitalRead(encodPinBR) > digitalRead(encodPinAR))
    {
        wheel_pulse_count_right = wheel_pulse_count_right + 1;
    }
    else
    {
        wheel_pulse_count_right = wheel_pulse_count_right - 1;
    }
}

// float poserror() {
//   int lastPulse = 0;
//   if (millis() - lastPosTime >= 100) {
//     int left = wheel_pulse_count_left - lastPulse;
//     // int right = wheel_pulse_count_right - lastPulseRight;

//     // float avg = (left + right)/2.0;

//     pos_error = setpoint_pos - left;

//     // Update stored values for the next iteration
//     lastPulse = wheel_pulse_count_left;
//     // lastPulseRight = wheel_pulse_count_right;
//     lastPosTime = millis();  // Update the timestamp

//     return pos_error;
//   }
//   return 0.0;
// }

float wheel_diameter = 0.045;    // Example: 6.5 cm wheel diameter
int pulses_per_revolution = 700; // Replace with your encoder's PPR
float wheel_circumference = 3.14 * wheel_diameter;
float distance_per_pulse = wheel_circumference / pulses_per_revolution;

float poserror()
{
    if (millis() - lastPosTime >= 100)
    { // Update every 100ms
        int deltaLeft = wheel_pulse_count_left - lastPulseLeft;
        int deltaRight = wheel_pulse_count_right - lastPulseRight;

        // Calculate displacement in meters for each wheel
        float displacementLeft = deltaLeft * distance_per_pulse;
        float displacementRight = deltaRight * distance_per_pulse;

        // Average displacement
        float avg_displacement = (displacementLeft + displacementRight) / 2.0;

        // Calculate position error
        pos_error += avg_displacement;

        // Store current pulse counts for next calculation
        lastPulseLeft = wheel_pulse_count_left;
        lastPulseRight = wheel_pulse_count_right;

        // Update timestamp
        lastPosTime = millis();
    }
    return setpoint_pos - pos_error; // Position error relative to setpoint
}
