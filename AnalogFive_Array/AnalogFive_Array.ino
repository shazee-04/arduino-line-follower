#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// Defines Motor Controller Pins
#define AIN1 8
#define BIN1 7
#define AIN2 9
#define BIN2 6
#define PWMA 10
#define PWMB 5
#define STBY 4

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Setting Up Sensor Array
QTRSensors qtr;
uint16_t position;
const uint8_t sensorCount = 5;
uint16_t sensorValues[sensorCount];

int treshold = 500;

int L = 0;  // Left Motor
int R = 0;  // Right Motor
int error = 0;
int add = 0;

int P;
int I;
int D;
int lastError = 0;

float kp = 0.13;  // 0.0729
float ki = 0;     // 0.005
float kd = 0;     // 1

int speed = 225;
int maxSpeed = 255;
int minSpeed = 0;
int turnSpeed = 255;

int resistor;  // Variable Resistor
int z;

int b_in;  // Bluetooth Module


void setup() {

  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A5, A4, A3, A2, A1 }, sensorCount);
  // qtr.setEmitterPin(13);

  // Callibrating Sensors...
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    Serial.println("calibrating...");
  }
}

void loop() {

  // b_control();
  pid();    // PID and Controlling Motor Speeds
  print();  // Serial Printing Values
}

void pid() {
  resistor = map(analogRead(A0), 0, 1023, 0, 255);
  position = qtr.readLineWhite(sensorValues);
  error = 2000 - position;  // 3500 - position for 8bit Sensor Array

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  add = P * kp + I * ki + D * kd;  // Adds To Initial Motor Speeds

  L = speed + add;  // New Speed Of Left Motor
  R = speed - add;  // New Speed Of Right Motor

  if (L > maxSpeed) {
    L = maxSpeed;
  }
  if (R > maxSpeed) {
    R = maxSpeed;
  }
  if (L < minSpeed) {
    L = minSpeed;
  }
  if (R < minSpeed) {
    R = minSpeed;
  }
  drive(L, R);  // Drive Function
}

void print() {

  // Printing Sensor Values, Mortor Speeds And Resistor Values...
  Serial.print("QTR::");
  Serial.print(position);
  Serial.print("\t");

  Serial.print("Error::");
  Serial.print(error);
  Serial.print("\t");

  Serial.print("Add::");
  Serial.print(add);
  Serial.print("\t");

  for (uint16_t x = 0; x < sensorCount; x++) {
    Serial.print(sensorValues[x]);
    Serial.print("\t");
  }

  Serial.print("Resis::");
  Serial.println(resistor);
}

void drive(int L, int R) {

  // When Resistors At Max Taking Right Turns... (Shortcut-1)
  if (resistor > 245) {
    if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
      left();
      delay(500);
    } else if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] > treshold) {
      right();
      delay(400);
    } else if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] > treshold && sensorValues[4] > treshold) {
      right();
      delay(400);
    } else if (sensorValues[0] > treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
      left();
      delay(400);
    } else if (sensorValues[0] > treshold && sensorValues[1] > treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
      left();
      delay(400);
    } else {
      motor1.drive(L);
      motor2.drive(R);  
    }

    // When Resistors At Lowest Taking Right Turns... (Shortcut-2)
  } else if (resistor < 10) {
    if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
      right();
      delay(500);
    } else if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] > treshold) {
      right();
      delay(400);
    } else if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] > treshold && sensorValues[4] > treshold) {
      right();
      delay(400);
    } else if (sensorValues[0] > treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
      left();
      delay(400);
    } else if (sensorValues[0] > treshold && sensorValues[1] > treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
      left();
      delay(400);
    } else {
      motor1.drive(L);
      motor2.drive(R);
    }

    // When Resistors At Neither Max Nor Lowest Drives Forward...
  } else {
    // if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] > treshold) {
    //   right();
    //   delay(400);
    // } else if (sensorValues[0] < treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] > treshold && sensorValues[4] > treshold) {
    //   right();
    //   delay(400);
    // } else if (sensorValues[0] > treshold && sensorValues[1] < treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
    //   left();
    //   delay(400);
    // } else if (sensorValues[0] > treshold && sensorValues[1] > treshold && sensorValues[2] < treshold && sensorValues[3] < treshold && sensorValues[4] < treshold) {
    //   left();
    //   delay(400);
    // } else {
    motor1.drive(L);
    motor2.drive(R);
    // }
  }
}

void left() {
  // Turns Left...
  motor1.drive(0);
  motor2.drive(turnSpeed);
}

void right() {
  // Turns Right...
  motor1.drive(turnSpeed);
  motor2.drive(0);
}

void brake() {
  //Brake...
  motor1.brake();
  motor2.brake();
}