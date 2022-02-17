#include "SimpleRSLK.h"

//array of ints to keep track of when each bumper has been pressed
int buttonStates [6] = { -100000}; //time when pressed (initialized far in the past)
int currentTime = 0;
int motorSpeed = 10;

enum states {
  KEEP_DRIVING,
  AVOID_HEADON,
  AVOID_LEFTOBS,
  AVOID_RIGHTOBS,
  STOP
};

states curr_state = KEEP_DRIVING;


void setup() {
  Serial.begin(115200);

  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);

  pinMode(BP_SW_PIN_0, INPUT_PULLUP);
  pinMode(BP_SW_PIN_1, INPUT_PULLUP);
  pinMode(BP_SW_PIN_2, INPUT_PULLUP);
  pinMode(BP_SW_PIN_3, INPUT_PULLUP);
  pinMode(BP_SW_PIN_4, INPUT_PULLUP);
  pinMode(BP_SW_PIN_5, INPUT_PULLUP);

  void loop() {
    //////////////////////////////////////////////////////////////////////////////////////
    //Monitor Variables
    //////////////////////////////////////////////////////////////////////////////////////
    //keep track of each button
    if (!digitalRead(BP_SW_PIN_0)) {
      buttonStates[0] = millis();
    }
    if (!digitalRead(BP_SW_PIN_1)) {
      buttonStates[1] = millis();
    }
    if (!digitalRead(BP_SW_PIN_2)) {
      buttonStates[2] = millis();
    }
    if (!digitalRead(BP_SW_PIN_3)) {
      buttonStates[3] = millis();
    }
    if (!digitalRead(BP_SW_PIN_4)) {
      buttonStates[4] = millis();
    }
    if (!digitalRead(BP_SW_PIN_5)) {
      buttonStates[5] = millis();
    }

    //debug prints
    Serial.print(buttonStates[2] + '...');
    Serial.print(buttonStates[2] + '...');
    Serial.print(buttonStates[3] + '...');
    Serial.print(buttonStates[4] + '...');
    Serial.println(buttonStates[5]);

    //////////////////////////////////////////////////////////////////////////////////////////////
    //Identify State
    //////////////////////////////////////////////////////////////////////////////////////////////
    currentTime = millis();
    if ((currentTime - buttonStates[0] < 2000 || currentTime - buttonStates[1] < 2000) &&
        (currentTime - buttonStates[2] < 2000 || currentTime - buttonStates[3] < 2000) &&
        (currentTime - buttonStates[4] < 2000 || currentTime - buttonStates[5] < 2000) ) {
      curr_state = STOP;
    } else if (currentTime - buttonStates[0] < 2000 || currentTime - buttonStates[1] < 2000) {
      curr_state = AVOID_LEFTOBS;
    } else if (currentTime - buttonStates[2] < 2000 || currentTime - buttonStates[3] < 2000) {
      curr_state = AVOID_HEADON;
    } else if (currentTime - buttonStates[4] < 2000 || currentTime - buttonStates[5] < 2000) {
      curr_state = AVOID_RIGHTOBS;
    } else {
      curr_state = KEEP_DRIVING;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    //Action
    ///////////////////////////////////////////////////////////////////////////////////////////////
    switch (curr_state) {
      case (AVOID_HEADON):
        if (currentTime - buttonStates[2] < 1000 || currentTime - buttonStates[3] < 1000) {
          enableMotor(BOTH_MOTORS);
          setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
          setMotorSpeed(BOTH_MOTORS, motorSpeed);
        } else {
          enableMotor(BOTH_MOTORS);
          setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorSpeed(BOTH_MOTORS, motorSpeed);
        }
        break;
      case (AVOID_LEFTOBS):
        enableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
        setMotorSpeed(LEFT_MOTOR, motorSpeed / 2);
        setMotorSpeed(RIGHT_MOTOR, motorSpeed);
        break;
      case (AVOID_RIGHTOBS):
        enableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
        setMotorSpeed(RIGHT_MOTOR, motorSpeed / 2);
        setMotorSpeed(LEFT_MOTOR, motorSpeed);
        break;
      case (KEEP_DRIVING):
        enableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        setMotorSpeed(BOTH_MOTORS, motorSpeed);
        break;
    }
  }
