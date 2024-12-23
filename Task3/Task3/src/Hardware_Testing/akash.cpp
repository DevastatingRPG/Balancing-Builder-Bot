// #include "Wire.h"
// #include <MPU6050_light.h>
// #include <Encoder.h>
// #include <SoftwareSerial.h>

// // Define SoftwareSerial pins
// SoftwareSerial bluetooth(A1, A0); // RX, TX

// MPU6050 mpu(Wire);
// float overall_pos = 0;
// unsigned long timer = 0;
// float complementaryAlpha = 0.95; // Complementary filter weight
// float angleFilteredX = 0.0;      // Filtered angle for X-axis
// float angleFilteredY = 0.0;      // Filtered angle for Y-axis
// float pdt = 0.0;                 // Time difference for PID

// int lastPulseLeft = 0, lastPulseRight = 0; // Stores the last pulse counts for wheels

// const int inputPin1 = A2; // Motor A pins
// const int inputPin2 = A3;
// const int inputPin3 = 9; // Motor B pins
// const int inputPin4 = 4;
// int EN1 = 6; // Motor A PWM
// int EN2 = 5; // Motor B PWM

// float setpoint_pitch = 0.0;
// float setpoint_pos = 0.0;

// float pitch_error = 0.0;
// float pos_error = 0.0;

// float pitch_gain = 0.0;
// float pos_gain = 0.0;

// long curPosRight = 0;
// long curPosLeft = 0;

// // Define encoder pins
// #define encodPinAR 7
// #define encodPinBR 2
// #define encodPinAL 8
// #define encodPinBL 3

// // Create encoder objects
// Encoder encRight(encodPinAR, encodPinBR);
// Encoder encLeft(encodPinAL, encodPinBL);

// long oldPosRight = 0;
// long oldPosLeft = 0;

// // Constants for distance calculation
// const float wheelDiameter = 0.043; // in meters
// const int pulsesPerRevolution = 3; // pulses per revolution

// class PID
// {
// private:
//     float kp, ki, kd;
//     float prev_err, integral;

// public:
//     PID(float kp, float ki, float kd)
//         : kp(kp), ki(ki), kd(kd), prev_err(0), integral(0) {}

//     float calculate(float err, float pdt)
//     {
//         float proportional = kp * err;
//         integral += ki * err * pdt;
//         float derivative = kd * (err - prev_err) / pdt;
//         float response = proportional + integral - derivative;
//         prev_err = err;

//         return response;
//     }
// };

// PID pitchPID(12.0, 0.0, 0.0);
// PID posPID(20.0, 0.0, 0.0);

// float calculateDistance(float pulses)
// {
//     // Calculate wheel circumference
//     float circumference = PI * wheelDiameter;
//     float distancePerPulse = circumference / pulsesPerRevolution;
//     float distance = distancePerPulse * pulses;
//     return distance;
// }

// float pitchError()
// {
//     // Update MPU angles
//     mpu.update();
//     unsigned long currentMillis = millis();
//     float dt = (currentMillis - timer) / 1000.0; // Time difference in seconds

//     // Get accelerometer angles
//     float accelAngleX = mpu.getAccAngleX();
//     float accelAngleY = mpu.getAccAngleY();

//     // Get gyroscope rates
//     float gyroRateX = mpu.getGyroX();
//     float gyroRateY = mpu.getGyroY();

//     // Complementary filter
//     angleFilteredX = complementaryAlpha * (angleFilteredX + gyroRateX * dt) + (1 - complementaryAlpha) * accelAngleX;
//     angleFilteredY = complementaryAlpha * (angleFilteredY + gyroRateY * dt) + (1 - complementaryAlpha) * accelAngleY;

//     pitch_error = setpoint_pitch - angleFilteredY;
//     return pitch_error;
// }

// float posError()
// {
//     // Read current encoder positions
//     volatile int curPosRight = encRight.read();
//     volatile int curPosLeft = -encLeft.read();

//     long deltaRight = curPosRight - oldPosRight;
//     long deltaLeft = curPosLeft - oldPosLeft;

//     // Calculate distances directly
//     float distanceRight = calculateDistance(deltaRight);
//     float distanceLeft = calculateDistance(deltaLeft);

//     // Update old encoder positions
//     oldPosRight = curPosRight;
//     oldPosLeft = curPosLeft;

//     // Calculate the average distance traveled
//     float averageDistance = (distanceRight + distanceLeft) / 2.0;

//     // Calculate position error
//     pos_error = setpoint_pos - averageDistance;
//     overall_pos += pos_error;
//     bluetooth.println(overall_pos);
//     Serial.println(overall_pos);
//     return overall_pos;
// }

// void setup()
// {
//     Serial.begin(9600);
//     bluetooth.begin(9600); // For HC-05 (default baud rate is 9600)

//     Serial.println("Bluetooth is ready!");

//     // MPU 6050
//     Wire.begin();
//     byte status = mpu.begin();
//     Serial.print(F("MPU6050 status: "));
//     Serial.println(status);
//     while (status != 0)
//     {
//     } // stop everything if could not connect to MPU6050

//     Serial.println(F("Calculating offsets, do not move MPU6050"));
//     delay(1000);
//     mpu.calcOffsets(); // gyro and accelero
//     Serial.println("Done!\n");
//     timer = millis();

//     // MOTOR
//     pinMode(EN1, OUTPUT);
//     pinMode(EN2, OUTPUT);
//     pinMode(inputPin1, OUTPUT);
//     pinMode(inputPin2, OUTPUT);
//     pinMode(inputPin3, OUTPUT);
//     pinMode(inputPin4, OUTPUT);
// }

// void loop()
// {
//     unsigned long sampleTime = 5;
//     unsigned long currentMillis = millis();

//     if (currentMillis - timer >= sampleTime)
//     {
//         // Calculate delta time (pdt) in seconds
//         pdt = (currentMillis - timer) / 1000.0;
//         timer = currentMillis; // Update timer

//         pitch_error = pitchError();
//         pos_error = posError();

//         pitch_gain = pitchPID.calculate(pitch_error, pdt);
//         pos_gain = posPID.calculate(pos_error, pdt);

//         // Combine gains for final velocity
//         int vel = pitch_gain - pos_gain;
//         vel *= 5;

//         // Set motor directions and speeds
//         if (vel < 0)
//         {
//             digitalWrite(inputPin1, LOW);
//             digitalWrite(inputPin2, HIGH);
//             digitalWrite(inputPin3, HIGH);
//             digitalWrite(inputPin4, LOW);
//         }
//         else
//         {
//             digitalWrite(inputPin1, HIGH);
//             digitalWrite(inputPin2, LOW);
//             digitalWrite(inputPin3, LOW);
//             digitalWrite(inputPin4, HIGH);
//         }

//         vel = constrain(abs(vel), 0, 255);

//         analogWrite(EN1, vel);
//         analogWrite(EN2, vel);

//         // Send data over Bluetooth
//         // bluetooth.println("Pitch gain:");
//         // bluetooth.println(pitch_gain);
//         // bluetooth.println("Pos gain:");
//         // bluetooth.println(pos_gain);
//         // bluetooth.println("Vel:");
//         bluetooth.println(vel);

//         Serial.flush();
//         bluetooth.flush();
//     }
// }
