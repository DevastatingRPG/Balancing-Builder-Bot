#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <SoftwareSerial.h>

// Define SoftwareSerial pins
SoftwareSerial bluetooth(A1, A0);  // RX, TX

#define ENCA 7 // YELLOW
#define ENCB 2 // WHITE

#define ENCAL 8  // Encoder A pin for wheel2(C2) ------ working
#define ENCBL 3

#define PWM1 6
#define IN2 A2
#define IN1 A3

#define PWM2 5
#define IN3 9
#define IN4 4

volatile int posiL = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posiR = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);  // For HC-05 (default baud rate is 9600)

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  pinMode(ENCAL,INPUT);
  pinMode(ENCBL,INPUT);

  digitalWrite(7, HIGH);
  digitalWrite(2, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoderOne,RISING);

  digitalWrite(ENCAL, HIGH);
  digitalWrite(ENCBL, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCBL),readEncoderTwo,RISING);
  
  pinMode(PWM1,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  pinMode(PWM2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target = 120;

  // PID constants
  float kp = 5;
  float kd = 0.015;
  float ki = 0.01;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0;
  int posL = 0;
  int posR = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posL = posiL;
    posR = posiR;
    pos = (posL + posR) / 2.0;
  }
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM1,PWM2,IN1,IN2,IN3,IN4);
  // setMotor(dir,pwr,PWM2,IN3,IN4);

  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

  bluetooth.println(target);
  bluetooth.println(" ");
  bluetooth.println(pos);
  bluetooth.println();
}

void setMotor(int dir, int pwmVal, int pwm1, int pwm2, int in1, int in2, int in3, int in4){
  analogWrite(pwm1,pwmVal);
  analogWrite(pwm2,pwmVal);
  if(dir == -1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
  }
  else if(dir == 1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  }  
}

void readEncoderOne(){
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  if(b > a){
    posiL--;
  }
  else{
    posiL++;
  }
}

void readEncoderTwo(){
  int c = digitalRead(ENCAL);
  int d = digitalRead(ENCBL);
  if(d > c){
    posiR++;
  }
  else{
    posiR--;
  }
}