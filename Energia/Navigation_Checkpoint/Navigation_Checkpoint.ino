//#include "SR04.h"
//#include <Wire.h>
//#include <L3G.h>
#include "Encoder.h"
#include "SimpleRSLK.h"
int Sol[2];
// Car line following Sensors
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint16_t normalSpeed = 17;
uint16_t fastSpeed = 24;
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint8_t lineColor = DARK_LINE;

// First Sonar Sensor
#define TRIG1_PIN 10
#define ECHO1_PIN 9
int Sonar1_Dist;
int Left_Sonar;

// Second Sonar Sensor
#define TRIG2_PIN 33
#define ECHO2_PIN 34
int Sonar2_Dist;
int Right_Sonar;

// IR Sensors
int IRleft = 6;
int IRright = 4;
int IRmid = 5;
volatile byte state = LOW;

enum states {
  Turn_Left,
  Turn_Right,
  Reset_Position,
  Adjust_Vertical,
  Adjust_Horizontal
};

int Current_State;

void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); // msp430g2231 must use 4800
  // make the on-board pushbutton's pin an input pullup:
  pinMode(IRmid, INPUT_PULLUP); // NOTE: because this is a pullup, a 1 indicates no beacon detected, 0 is yes beacon detected
  attachInterrupt(digitalPinToInterrupt(IRmid), blink, CHANGE);
  pinMode(IRleft, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRleft), blink, CHANGE);
  pinMode(IRright, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRright), blink, CHANGE);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(TRIG2_PIN, OUTPUT);

  states Current_State = Adjust_Vertical;

}

// Sonar Sensors
int * Sonar_Sensors() {
  digitalWrite(TRIG1_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(TRIG1_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIG1_PIN, LOW);

  long timedelay = pulseIn(ECHO1_PIN, HIGH);
  Left_Sonar = 0.0343 * (timedelay / 2);

  Serial.print("Sensor 1 : ");
  Serial.println(Sonar1_Dist);

  delayMicroseconds(100);

  digitalWrite(TRIG2_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(TRIG2_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIG2_PIN, LOW);

  long td = pulseIn(ECHO2_PIN, HIGH);
  Right_Sonar = 0.0343 * (td / 2);
}

void blink() {
  state = !state;
}


void loop() {


  switch (Current_State) {
    case Turn_Left:
      // Satet
      break;
    case Turn_Right:
      // Satet
      break;
    case Reset_Position:
      // Satet
      break;

    case Adjust_Vertical:

      while ((Right_Sonar + Left_Sonar) / 2 > 126) {
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
        enableMotor(BOTH_MOTORS);
        setMotorSpeed(LEFT_MOTOR, 0);
        setMotorSpeed(RIGHT_MOTOR, 0);
        Sonar_Sensors();
      }
      setMotorSpeed(LEFT_MOTOR, normalSpeed);
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);

      while (( (Left_Sonar || Right_Sonar) > 45) && abs(Left_Sonar - Right_Sonar) > 2) {
        setMotorSpeed(LEFT_MOTOR, normalSpeed);
        setMotorSpeed(RIGHT_MOTOR, fastSpeed);
        Sonar_Sensors();
      }
      setMotorSpeed(LEFT_MOTOR, 0);
      setMotorSpeed(RIGHT_MOTOR, 0);
      Current_State = Adjust_Horizontal;
      break;

    case Adjust_Horizontal:

      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      while (getLeftWheelDir() < 45 && getRightWheelDir() > -45) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeed);
        setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      }

      setMotorSpeed(LEFT_MOTOR, 0);
      setMotorSpeed(RIGHT_MOTOR, 0);
      resetLeftEncoderCnt();
      resetRightEncoderCnt();
      Sonar_Sensors();
      readLineSensor(sensorVal);

      if ((Left_Sonar + Right_Sonar) / 2 > 87) {
        while (sensorVal[4] < 100) {

          setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
          setMotorSpeed(BOTH_MOTORS, normalSpeed);
          readLineSensor(sensorVal);
        }
      }
      else {
        while (sensorVal[4] < 100) {
          setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
          setMotorSpeed(BOTH_MOTORS, normalSpeed);
          readLineSensor(sensorVal);
        }
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      while (getLeftWheelDir() < 45 && getRightWheelDir() > -45) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeed);
        setMotorSpeed(RIGHT_MOTOR, normalSpeed);
      }
      setMotorSpeed(LEFT_MOTOR, 0);
      setMotorSpeed(RIGHT_MOTOR, 0);

      while ((Right_Sonar + Left_Sonar) / 2 <= 30) {
        uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
        if (linePos > 0 && linePos < 3000) {
          setMotorSpeed(LEFT_MOTOR, normalSpeed);
          setMotorSpeed(RIGHT_MOTOR, fastSpeed);
        } else if (linePos > 3500) {
          setMotorSpeed(LEFT_MOTOR, fastSpeed);
          setMotorSpeed(RIGHT_MOTOR, normalSpeed);
        } else {
          setMotorSpeed(LEFT_MOTOR, normalSpeed);
          setMotorSpeed(RIGHT_MOTOR, normalSpeed);
        }
      }

      break;

  }
}
