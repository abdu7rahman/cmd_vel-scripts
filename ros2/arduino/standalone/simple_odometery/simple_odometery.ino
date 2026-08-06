
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters


double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



// Encoder pin definitions
const int encoderPin1A = 31;
const int encoderPin1B = 32;
const int encoderPin2A = 35;
const int encoderPin2B = 37;
const int encoderPin3A = 39;
const int encoderPin3B = 41;
const int encoderPin4A = 43;
const int encoderPin4B = 45;

// Motor pin definitions
const int motor1pin1 = 34;
const int motor1pin2 = 36;
const int ENA = 10;
const int motor2pin1 = 38;
const int motor2pin2 = 40;
const int ENB = 11;
const int motor3pin1 = 26;
const int motor3pin2 = 28;
const int ENC = 8;
const int motor4pin1 = 30;
const int motor4pin2 = 32;
const int END = 9;

// Wheel and robot dimensions
const float wheelRadius = 0.05; // meters
const float robotRadius = 0.266; // meters
const int pulsesPerRevolution = 1300; // Encoder pulses per wheel revolution

// Encoder variables
volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
volatile long encoderValue3 = 0;
volatile long encoderValue4 = 0;

long lastEncoderValue1 = 0;
long lastEncoderValue2 = 0;
long lastEncoderValue3 = 0;
long lastEncoderValue4 = 0;

// Odometry variables
float x = 0.0;
float y = 0.0;
float theta = 0.0;

void setup() {
  Serial.begin(9600);

  Setpoint = 1300;
  // Encoder pin setup
  pinMode(encoderPin1A, INPUT);
  pinMode(encoderPin1B, INPUT);
  pinMode(encoderPin2A, INPUT);
  pinMode(encoderPin2B, INPUT);
  pinMode(encoderPin3A, INPUT);
  pinMode(encoderPin3B, INPUT);
  pinMode(encoderPin4A, INPUT);
  pinMode(encoderPin4B, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2B), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3A), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3B), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4A), updateEncoder4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4B), updateEncoder4, CHANGE);
}

void loop() {
   Input = encoderValue1;
    myPID.Compute();
  // Calculate odometry
  long dEncoder1 = encoderValue1 - lastEncoderValue1;
  long dEncoder2 = encoderValue2 - lastEncoderValue2;
  long dEncoder3 = encoderValue3 - lastEncoderValue3;
  long dEncoder4 = encoderValue4 - lastEncoderValue4;

  lastEncoderValue1 = encoderValue1;
  lastEncoderValue2 = encoderValue2;
  lastEncoderValue3 = encoderValue3;
  lastEncoderValue4 = encoderValue4;

  float dS1 = (dEncoder1 * 2 * PI * wheelRadius) / pulsesPerRevolution;
  float dS2 = (dEncoder2 * 2 * PI * wheelRadius) / pulsesPerRevolution;
  float dS3 = (dEncoder3 * 2 * PI * wheelRadius) / pulsesPerRevolution;
  float dS4 = (dEncoder4 * 2 * PI * wheelRadius) / pulsesPerRevolution;

  // Compute robot movement
  float dX = (dS1 + dS2 + dS3 + dS4) / 4.0;
  float dY = (-dS1 + dS2 - dS3 + dS4) / 4.0;
  float dTheta = (-dS1 + dS2 + dS3 - dS4) / (4.0 * robotRadius);

  // Update robot position
  static float x = 0.0;
  static float y = 0.0;
  static float theta = 0.0;

  x += dX * cos(theta) - dY * sin(theta);
  y += dX * sin(theta) + dY * cos(theta);
  theta += dTheta;

  // Print odometry data
  Serial.print("X: ");
  Serial.print(x);
  Serial.print("\tY: ");
  Serial.print(y);
  Serial.print("\tTheta: ");
  Serial.println(theta);

  delay(100); // Adjust as needed
}

void updateEncoder1() {
  static int lastEncoded1 = 0;
  int MSB = digitalRead(encoderPin1A);
  int LSB = digitalRead(encoderPin1B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded1 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue1++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue1--;
  lastEncoded1 = encoded;
}

void updateEncoder2() {
  static int lastEncoded2 = 0;
  int MSB = digitalRead(encoderPin2A);
  int LSB = digitalRead(encoderPin2B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded2 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2--;
  lastEncoded2 = encoded;
}

void updateEncoder3() {
  static int lastEncoded3 = 0;
  int MSB = digitalRead(encoderPin3A);
  int LSB = digitalRead(encoderPin3B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded3 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue3++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue3--;
  lastEncoded3 = encoded;
}

void updateEncoder4() {
  static int lastEncoded4 = 0;
  int MSB = digitalRead(encoderPin4A);
  int LSB = digitalRead(encoderPin4B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded4 << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue4++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue4--;
  lastEncoded4 = encoded;
}
