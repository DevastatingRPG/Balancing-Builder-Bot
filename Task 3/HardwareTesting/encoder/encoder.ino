// Define encoder pins
#define encodPinAR 7  // Encoder A pin for wheel1(C2) // A1
#define encodPinBR 2  // Encoder B pin for wheel 1(C1) // A2//2

#define encodPinAL 8  // Encoder A pin for wheel2(C2) ------ working
#define encodPinBL 3  // Encoder B pin for wheel2(C1) ------ working

// Variable for wheel angle
int wheel_pulse_count_left = 0;
int wheel_pulse_count_right = 0;

const int inputPin1 = A2;  // Pin 15 of L293D IC
const int inputPin2 = A3;  // Pin 10 of L293D IC
                           // Motor B
const int inputPin3 = 9;   // Pin 7 of L293D IC
const int inputPin4 = 4;   // Pin 2 of L293D IC
int EN1 = 6;               // Pin 1 of L293D IC
int EN2 = 5;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  //------Encoder HW Interrupt setup-----//
  pinMode(8, INPUT_PULLUP);
  digitalWrite(8, HIGH);
  // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

  pinMode(3, INPUT_PULLUP);
  digitalWrite(3, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), mot_rencoder_left, RISING);
  // ---------------------------------------------- ///

  //------Encoder HW Interrupt setup-----//
  pinMode(7, INPUT_PULLUP);  //10
  digitalWrite(7, HIGH);
  // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

  pinMode(2, INPUT_PULLUP);  //11
  digitalWrite(2, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), mot_rencoder_right, RISING);
  // ---------------------------------------------- ///

    // MOTOR
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPin2, OUTPUT);
  pinMode(inputPin3, OUTPUT);
  pinMode(inputPin4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(wheel_pulse_count_left);
  Serial.print(" ; ");
  Serial.println(wheel_pulse_count_right);
  // Serial.print(digitalRead(encodPinBR));
  // Serial.print(" ; ");
  // Serial.println(digitalRead(encodPinAR));
}

// ISR for motor encoder
void mot_rencoder_left() {
  // if (digitalRead(encodPinBR) == HIGH) {
  if (digitalRead(encodPinBL) > digitalRead(encodPinAL)) {
    wheel_pulse_count_left = wheel_pulse_count_left - 1;
  } else {
    wheel_pulse_count_left = wheel_pulse_count_left + 1;
  }
}

void mot_rencoder_right() {
  // if (digitalRead(encodPinBR) == HIGH) {
  if (digitalRead(encodPinBR) > digitalRead(encodPinAR)) {
    wheel_pulse_count_right = wheel_pulse_count_right + 1;
  } else {
    wheel_pulse_count_right = wheel_pulse_count_right - 1;
  }
}
