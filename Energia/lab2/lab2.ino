#include "SimpleRSLK.h"

//array of unsigned ints to keep track of when each bumper has been pressed
int buttonStates [6] = { -100000}; //time when pressed (initialized far in the past)

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

  // Initalize the current state to the default, keep driving.
}


void loop() {
  // put your main code here, to run repeatedly:

  ///////////////////////////////
  //Monitor Variables
  ///////////////////////////////

  //keep track of each button
  if (!digitalRead(BP_SW_PIN_0)) {
    buttonStates[0] = millis();
  } if (!digitalRead(BP_SW_PIN_1)) {
    buttonStates[1] = millis();
  } if (!digitalRead(BP_SW_PIN_2)) {
    buttonStates[2] = millis();
  } if (!digitalRead(BP_SW_PIN_3)) {
    buttonStates[3] = millis();
  } if (!digitalRead(BP_SW_PIN_4)) {
    buttonStates[4] = millis();
  } if (!digitalRead(BP_SW_PIN_5)) {
    buttonStates[5] = millis();
  }

  Serial.print(buttonStates[2]);
  Serial.print(buttonStates[3]);
  Serial.print(buttonStates[4]);
  Serial.println(buttonStates[5]);
  ///////////////////////////////
  //Identify State
  ///////////////////////////////
  if (millis() - buttonStates[2] < 2000 ||
      millis() - buttonStates[3] < 2000) {
    curr_state = AVOID_HEADON;
  }
  else {
    curr_state = KEEP_DRIVING;
  }
  /*if (buttonStates[0] || buttonStates[1]) {
    curr_state = AVOID_LEFTOBS;
    }
    else if (buttonStates[4] || buttonStates[5]) {
    curr_state = AVOID_RIGHTOBS;
  */

  ///////////////////////////////
  //Action
  ///////////////////////////////

  switch (curr_state) {
    case (AVOID_HEADON):
      //pauseMotor(BOTH_MOTORS);
      //setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
      //setMotorSpeed(BOTH_MOTORS, motorSpeed); // NOTE: RIGHT_MOTOR and LEFT_MOTOR are also defined in SimpleRSLK.h
      //enableMotor(BOTH_MOTORS);
      //delay(1500);
      //enableMotor(LEFT_MOTOR);
      //delay(1500);
      break;
    case (KEEP_DRIVING):
      //enableMotor(BOTH_MOTORS);
      //setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      //setMotorSpeed(BOTH_MOTORS, motorSpeed); // NOTE: RIGHT_MOTOR and LEFT_MOTOR are also defined in SimpleRSLK.h
      break;
  }
}
