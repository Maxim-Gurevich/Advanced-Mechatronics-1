
// Libraries
#include "Encoder.h"
#include "SimpleRSLK.h"

// Car line following Sensors
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint16_t normalSpeed = 13;
uint16_t fastSpeed = 24;
uint16_t normalSpeedright = 13 ;
uint16_t normalSpeedleft = 16.8 ;
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint8_t lineColor = DARK_LINE;
uint32_t linePos;

// Encoder Pins & Vars
uint16_t ela_pin = P10_5;
uint16_t elb_pin = P5_2;
uint16_t era_pin = P10_4;
uint16_t erb_pin = P5_0;
int Left_En;
int Right_En;

// Vars for when no beacons detected
int Shimmey = 0;
int Shimmey2 = 0;

// Vars for when we can't turn
int counter;
int i;
int const imax = 50; //Changes the size of the array
int Values[imax];

// Save which beacon was on
int Beacon;

// Right Sonar Sensor
uint16_t TRIG1_PIN  = P9_1;
uint16_t ECHO1_PIN  = P8_3;
int Right_Sonar;

// Left Sonar Sensor
uint16_t TRIG2_PIN = P9_3;
uint16_t ECHO2_PIN = P6_3;
int Left_Sonar;
int Sonar_Diff; // right - left

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
  Adjust_Horizontal,
  Shoot,
  IR_Sensors
};

states Current_State;

void setup() {

  Serial.begin(115200);

  //  make the on - board pushbutton's pin an input pullup:
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
  setupRSLK();
  Current_State = Adjust_Vertical;
  setupEncoder(ela_pin, elb_pin, era_pin, erb_pin);

  // Creating the values array
  for (i = 0; i < imax ; i++) {
    Values[i] = 0;
  }
  Values[imax - 1] = 1;
  Values[(imax / 2) - 1] = 1;
}

// Sonar Sensors Function
int Sonar_Sensors() {
  digitalWrite(TRIG1_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(TRIG1_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIG1_PIN, LOW);

  long timedelay = pulseIn(ECHO1_PIN, HIGH);
  Left_Sonar = 0.0343 * (timedelay / 2);

  delayMicroseconds(100);

  digitalWrite(TRIG2_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(TRIG2_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIG2_PIN, LOW);

  long td = pulseIn(ECHO2_PIN, HIGH);
  Right_Sonar = 0.0343 * (td / 2);
  Sonar_Diff = Right_Sonar - Left_Sonar;
  delay(1);
}

void Not_Turning() {

  
}

void blink() {
  state = !state;
}

void loop() {

  enableMotor(BOTH_MOTORS);

  switch (Current_State) {
    case Adjust_Vertical:
      Sonar_Sensors();
      Right_En = 0;
      Left_En = 0;
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      // When the robot is close to the backwall (can't turn), go backwards
      // we are going to take the last 3 encoder ticks (more if needed) till we move forward
      // first 3 are right encoder, last 3 are left

      while (Left_Sonar > 45 || Right_Sonar > 45) {
        // Checks the first and last encoder values found, if they are the same,
        // the wheels are not turning
        if (Values[0] == Values[(imax / 2) - 1] || Values[imax / 2] == Values[imax - 1]) {
          // if we cant turn we move backwards set distance 20 pulses (might need to change).
          setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
          setMotorSpeed(BOTH_MOTORS, normalSpeed);
          delay(500);
          setMotorSpeed(BOTH_MOTORS, 0);
          // This ensures the backup won't be called again (might not be needed)
          Values[0] = 0;
          Values[(imax / 2) - 1] = 1;
          Values[imax / 2] = 0;
          Values[imax - 1] = 1;

          //          values[0] = 0;
          //          values[1] = 0;
          //          values[2] = 0;
          //          values[3] = 0;
          //          values[4] = 1;
          //          values[5] = 0;
          //          values[6] = 0;
          //          values[7] = 0;
          //          values[8] = 0;
          //          values[9] = 1;
        }
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
        setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
        Right_En = getEncoderRightCnt();
        Left_En = getEncoderLeftCnt();

        // This will update the list after finding the encoder counts
        for (i = 1 ; i < imax ; i++) {
          Values[i - 1] = Values[i];
        }
        Values[(imax / 2) - 1] = Right_En;
        Values[imax - 1] = Left_En;

        //        values[0] = values[1];
        //        values[1] = values[2];
        //        values[2] = values[3];
        //        values[3] = values[4];
        //        values[4] = Right_En;
        //        values[5] = values[6];
        //        values[6] = values[7];
        //        values[7] = values[8];
        //        values[8] = values[9];
        //        values[9] = Left_En;
        Sonar_Sensors();
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      Right_En = 0;
      Left_En = 0;
      resetLeftEncoderCnt();
      resetRightEncoderCnt();

      while (abs(Sonar_Diff) > 1) {
        if (Sonar_Diff < 0) {
          setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorSpeed(LEFT_MOTOR, normalSpeed);
          setMotorSpeed(RIGHT_MOTOR, 0);
        }
        if (Sonar_Diff > 0) {
          setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorSpeed(RIGHT_MOTOR, normalSpeed);
          setMotorSpeed(LEFT_MOTOR, 0);
        }

        Sonar_Sensors();
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      Current_State = Adjust_Horizontal;
      break;

    case Adjust_Horizontal:
      Right_En = 0;
      Left_En = 0;
      resetLeftEncoderCnt();
      resetRightEncoderCnt();
      while (Right_En < 180 || Left_En < 180) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
        setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
        Right_En = getEncoderRightCnt();
        Left_En = getEncoderLeftCnt();
      }

      setMotorSpeed(BOTH_MOTORS, 0);
      Sonar_Sensors();
      readLineSensor(sensorVal);

      if ((Left_Sonar + Right_Sonar) / 2 > 87) {
        while (sensorVal[4] < 2500 && sensorVal[3] < 2500 && sensorVal[2] < 2000 && sensorVal[5] < 2000) {
          setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
          setMotorSpeed(BOTH_MOTORS, normalSpeed);
          readLineSensor(sensorVal);
        }
      }
      else {
        while (sensorVal[4] < 2500 && sensorVal[3] < 2500 && sensorVal[2] < 2000 && sensorVal[5] < 2000) {
          setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
          setMotorSpeed(BOTH_MOTORS, normalSpeed);
          readLineSensor(sensorVal);
        }
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      resetLeftEncoderCnt();
      resetRightEncoderCnt();
      Left_En = 0;
      Right_En = 0;
      while (Left_En < 180 || Right_En < 180) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
        setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
        Left_En = getEncoderLeftCnt();
        Right_En = getEncoderRightCnt();
      }

      setMotorSpeed(BOTH_MOTORS, 0);

      while ((Right_Sonar + Left_Sonar) / 2 < 30) {
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        linePos = getLinePosition(sensorCalVal, lineColor);
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
        Sonar_Sensors();
      }

      while ((Right_Sonar + Left_Sonar) / 2 > 30) {
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
        linePos = getLinePosition(sensorCalVal, lineColor);
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
        Sonar_Sensors();
      }
      Current_State = Reset_Position;
      setMotorSpeed(BOTH_MOTORS, 0);
      break;


    case Reset_Position:
      while ((Right_Sonar + Left_Sonar) / 2 < 42) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        linePos = getLinePosition(sensorCalVal, lineColor);
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
        Sonar_Sensors();
      }

      while ((Right_Sonar + Left_Sonar) / 2 > 42) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        linePos = getLinePosition(sensorCalVal, lineColor);
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
        Sonar_Sensors();
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      while (abs(Sonar_Diff) > 1) {
        if (Sonar_Diff < 0) {
          setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorSpeed(RIGHT_MOTOR, normalSpeed);
          setMotorSpeed(LEFT_MOTOR, 0);
        }
        if (Sonar_Diff > 0) {
          setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorSpeed(LEFT_MOTOR, normalSpeed);
          setMotorSpeed(RIGHT_MOTOR, 0);
        }
        Sonar_Sensors();
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      Current_State = IR_Sensors;
      break;

    case IR_Sensors:
      if (IRmid == 0) {
        Current_State = Shoot;
        Beacon = 1;
      } else if (IRleft == 0) {
        Current_State = Turn_Left;
        Beacon = 0;
      } else if (IRright == 0) {
        Current_State = Turn_Right;
        Beacon = 2;
      }
    //else {
    //          if (Shimmey < 5) {
    //            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    //            setMotorSpeed(LEFT_MOTOR, normalSpeed);
    //            setMotorSpeed(RIGHT_MOTOR, 0);
    //            delay(200);
    //            Shimmey += 1;
    //          }
    //        }


    case Turn_Left:
      resetLeftEncoderCnt();
      resetRightEncoderCnt();
      Left_En = 0;
      Right_En = 0;
      while (Left_En < 60 || Right_En < 60) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
        setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
        Left_En = getEncoderLeftCnt();
        Right_En = getEncoderRightCnt();
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      Current_State = Shoot;
      break;

    case Turn_Right:
      resetLeftEncoderCnt();
      resetRightEncoderCnt();
      Left_En = 0;
      Right_En = 0;
      while (Left_En < 60 || Right_En < 60) {
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
        setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
        Left_En = getEncoderLeftCnt();
        Right_En = getEncoderRightCnt();
      }
      setMotorSpeed(BOTH_MOTORS, 0);
      Current_State = Shoot;
      resetLeftEncoderCnt();
      resetRightEncoderCnt();
      Left_En = 0;
      Right_En = 0;
      break;

    case Shoot:
      delay(5000);
      if (Beacon == 0 && (Left_En < 60 || Right_En < 60) ) {
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
        setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
        Left_En = getEncoderLeftCnt();
        Right_En = getEncoderRightCnt();
        if ((Left_En >= 60 || Right_En >= 60)) {
          Current_State = Reset_Position;
        }
      }

      if (Beacon == 1) {
        Current_State = Reset_Position;
      }

      if (Beacon == 2 && (Left_En < 60 || Right_En < 60) ) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
        setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
        Left_En = getEncoderLeftCnt();
        Right_En = getEncoderRightCnt();
        if ((Left_En >= 60 || Right_En >= 60)) {
          Current_State = Reset_Position;
        }
      }
      break;

  }

}
