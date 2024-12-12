#include <SoftwareSerial.h>

// Define SoftwareSerial pins
SoftwareSerial bluetooth(A1, A0); // RX, TX

void setup() {
  // Start serial communications
  Serial.begin(9600);        // For serial monitor
  bluetooth.begin(9600);     // For HC-05 (default baud rate is 9600)

  Serial.println("Bluetooth is ready!");
}

void loop() {
  // Check for data from Bluetooth
  if (bluetooth.available()) {
    char btData = bluetooth.read();
    Serial.print("Received from Bluetooth: ");
    Serial.println(btData);
  }

  // Check for data from Serial Monitor
  if (Serial.available()) {
    char serialData = Serial.read();
    bluetooth.print(serialData); // Send data to Bluetooth
    Serial.print("Sent to Bluetooth: ");
    Serial.println(serialData);
  }
}
