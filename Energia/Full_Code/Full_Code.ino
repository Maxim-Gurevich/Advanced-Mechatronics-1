//going in circle while shooting ()check)
// delay after each shot (check)
//timeout if going in wrong direction (check)
// emergency exit if not on black line (check)


// Libraries
#include "Encoder.h"
#include "SimpleRSLK.h"

// Car line following Sensors
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint16_t slowSpeed = 10;
uint16_t normalSpeed = 17;
uint16_t fastSpeed = 24;
uint16_t normalSpeedright = 17 ;
uint16_t normalSpeedleft = 17 ; //16.8
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint8_t lineColor = DARK_LINE;
uint32_t linePos;
bool Line = false;

// Encoder Pins & Vars
uint16_t ela_pin = P10_5;
uint16_t elb_pin = P5_2;
uint16_t era_pin = P10_4;
uint16_t erb_pin = P5_0;
int Left_En;
int Right_En;

// Vars for when we can't turn
int counter;
int i;
int const imax = 100; //Changes the size of the array
int Values[imax];

// Sonar_diff stuff
uint32_t Period = 5000;
uint32_t tStart;
float Align_Maxdiff;
int imax_Sonar = 20;
int Max_Sonar = 88;

bool Backward = false;

// Right Sonar Sensor
uint16_t TRIG1_PIN  = P9_1;
uint16_t ECHO1_PIN  = P8_3;
float Right_Sonar;

// Left Sonar Sensor
uint16_t TRIG2_PIN = P9_3;
uint16_t ECHO2_PIN = P6_3;
float Left_Sonar;
float Sonar_Diff; // right - left

// To execute the aligning steps only once:
bool Repeat = false;

//Shooting Vars:
uint16_t Enable_Motor = P5_6 ;
uint16_t Input1 = P6_6;
uint16_t Input2 = P6_7;
int Motor_Speed = 255 * 9 / 9;
uint16_t Touch_Pin = P5_1;
int Touch_Val;
unsigned long Current_Time;
int Shooting_Time = 300;

//IR Sensors
int IRleft = P8_4;
int IRright = P8_2;
int IRmid = P9_0;
int IRStateM;
int IRStateL;
int IRStateR;
volatile byte state = LOW;
int Beacon = 0;

//Wiggling Vars
int Wiggle_Time = 500;
bool Wiggle = false;
int Wiggle_Counter = 0;
unsigned long Current_Time2;

enum states {
  Left_Beacon,
  Right_Beacon,
  Shooting,
  Home,
  Just_Shot
};

states Current_State;

void setup() {
  delay(1000);
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize Sonar Pins
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(TRIG2_PIN, OUTPUT);

  //  Initialize Shooting Motor Pins
  pinMode(Enable_Motor, OUTPUT);
  pinMode(Input1, OUTPUT);
  pinMode(Input2, OUTPUT);

  //Touch Sensor Pin
  pinMode(Touch_Pin, INPUT_PULLUP);

  // IR Sensors
  pinMode(IRmid, INPUT_PULLUP); // NOTE: because this is a pullup, a 1 indicates no beacon detected, 0 is yes beacon detected
  pinMode(IRleft, INPUT_PULLUP);
  pinMode(IRright, INPUT_PULLUP);

  // Initialize the RSLK library and the encoder
  setupRSLK();
  setupEncoder(ela_pin, elb_pin, era_pin, erb_pin);

  // Creating the values array
  for (i = 0; i < imax ; i++) {
    Values[i] = 0;
  }
  Values[imax - 1] = 1;
  Values[(imax / 2) - 1] = 1;

  setMotorSpeed(BOTH_MOTORS, 0);
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
  Sonar_Sensors();
  Right_En = 0;
  Left_En = 0;
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  while (Left_Sonar > 45 || Right_Sonar > 45) {
    if (Values[0] == Values[(imax / 2) - 1] || Values[imax / 2] == Values[imax - 1]) {
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
      setMotorSpeed(BOTH_MOTORS, normalSpeed);
      delay(700);
      setMotorSpeed(BOTH_MOTORS, 0);
      // This ensures the backup won't be called again (might not be needed)
      Values[0] = 0;
      Values[(imax / 2) - 1] = 1;
      Values[imax / 2] = 0;
      Values[imax - 1] = 1;
    }
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
    Right_En = getEncoderRightCnt();
    Left_En = getEncoderLeftCnt();

    // This will update the list after finding the encoder counts
    for (i = 1 ; i < imax ; i++) {
      Values[i - 1] = Values[i];
    }
    Values[(imax / 2) - 1] = Right_En;
    Values[imax - 1] = Left_En;
    Sonar_Sensors();
  }
  setMotorSpeed(BOTH_MOTORS, 0);
  Right_En = 0;
  Left_En = 0;
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

}

void Back_Aligning() {
  for (tStart = millis() ; (millis() - tStart) < Period;) {
    Sonar_Sensors();
    while (abs(Sonar_Diff) > Align_Maxdiff) {
      if (Sonar_Diff < 0) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, normalSpeed * 0.9); // * abs(Sonar_Diff) / 4);
      }
      if (Sonar_Diff > 0) {
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, normalSpeed * 0.9); // * abs(Sonar_Diff) / 4);
      }

      Sonar_Sensors();
    }
    setMotorSpeed(BOTH_MOTORS, 0);
  }
}

void CCW_90() {
  Right_En = 0;
  resetRightEncoderCnt();
  while (Right_En < 370) {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(RIGHT_MOTOR, normalSpeedright);
    setMotorSpeed(LEFT_MOTOR, 0);
    Right_En = getEncoderRightCnt();
  }

  setMotorSpeed(BOTH_MOTORS, 0);
}

void Find_Line() {
  for (i = 1; i < imax_Sonar ; i++) {
    Sonar_Sensors();
    Max_Sonar = max(Max_Sonar, Right_Sonar);
  }
  readLineSensor(sensorVal);
  // Add stuff to go more backwards when > 95
  if ( Max_Sonar > 79) {
    Current_Time = millis();
    while ( (millis() - Current_Time < 2000) && sensorVal[4] < 2500 && sensorVal[3] < 2500 && sensorVal[2] < 2000 && sensorVal[5] < 2000) {
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
      setMotorSpeed(BOTH_MOTORS, normalSpeed);
      readLineSensor(sensorVal);
      Backward = true;
    }
  }
  else {
    Current_Time = millis();
    while ( (millis() - Current_Time < 2000) && sensorVal[4] < 2500 && sensorVal[3] < 2500 && sensorVal[2] < 2000 && sensorVal[5] < 2000) {
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      setMotorSpeed(BOTH_MOTORS, normalSpeed);
      readLineSensor(sensorVal);
    }
  }
  setMotorSpeed(BOTH_MOTORS, 0);
}

void CW_90() {
  resetLeftEncoderCnt();
  Left_En = 0;
  while (Left_En < 370) {
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(LEFT_MOTOR, normalSpeedleft);
    setMotorSpeed(RIGHT_MOTOR, 0);
    Left_En = getEncoderLeftCnt();
  }
  setMotorSpeed(BOTH_MOTORS, 0);
}

void Align_Vertical() {
  while ((Right_Sonar + Left_Sonar) / 2 < 20) {
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

  while ((Right_Sonar + Left_Sonar) / 2 > 20) {
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
  setMotorSpeed(BOTH_MOTORS, 0);
}

void IR_Sensors() {
  // Beacon = round(random(1.6, 4.4));
  IRStateM = analogRead(IRmid);
  IRStateL = analogRead(IRleft);
  IRStateR = analogRead(IRright);
  if (IRStateM < 80) {
    Beacon = 2; //Shoot
  } else if (IRStateL < 80) {
    Beacon = 1; //Turn Left
  } else if (IRStateR < 10) {
    Beacon = 3; //Turn right
  } else {
    Beacon = 4;
    Current_Time = millis();
  }
}

void Turn_Left() {
  Left_En = 0;
  resetLeftEncoderCnt();
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  while (Left_En < 60) {
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, 0);
    Left_En = getEncoderLeftCnt();
  }
  setMotorSpeed(BOTH_MOTORS, 0);
}

void Turn_Right() {
  resetLeftEncoderCnt();
  Left_En = 0;
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  while (Left_En < 60) {
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, 0);
    Left_En = getEncoderLeftCnt();
  }
  setMotorSpeed(BOTH_MOTORS, 0);
}

void blink() {
  state = !state;
}

void First_Load() {
  Touch_Val = analogRead(Touch_Pin);
  while (Touch_Val < 100) {
    digitalWrite(Input1, HIGH);
    digitalWrite(Input2, LOW);
    analogWrite(Enable_Motor, Motor_Speed);
    Touch_Val = analogRead(Touch_Pin);
  }
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  analogWrite(Enable_Motor, 0);
}

void loop() {
  enableMotor(BOTH_MOTORS);
  if (Repeat == false) {
    Not_Turning();
    Align_Maxdiff = 0.7;
    Back_Aligning(); //Works
    CCW_90(); //works
    while (!Line) {
      Find_Line(); //works
      if (sensorVal[4] = 2500) { // && sensorVal[3] < 2500 && sensorVal[2] < 2000 && sensorVal[5] < 2000) {
        Line = true;
      }
    }
    Current_Time = millis();
    if (Backward = true) {
      while (millis() - Current_Time < 500) {
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, normalSpeed);
      }
    }
    CW_90(); //works
    Align_Maxdiff = 0.7; //works
    Back_Aligning(); //works
    Align_Vertical(); //works
    Align_Maxdiff = 0.7; //works
    Back_Aligning(); //works
    First_Load();
    Repeat = true;
    Line = false;
    Current_State = Home;
  }

  switch (Current_State) {
    case Left_Beacon:
      Turn_Left();
      Current_Time = millis();
      Current_State = Shooting;
      break;

    case Right_Beacon:
      Turn_Right();
      Current_Time = millis();
      Current_State = Shooting;
      break;

    case Shooting:
      if (millis() - Current_Time < Shooting_Time) {
        analogWrite(Enable_Motor, Motor_Speed);
        digitalWrite(Input1, HIGH);
        digitalWrite(Input2, LOW);
      }
      else {
        analogWrite(Enable_Motor, 0);
        Right_En = 0;
        Left_En = 0;
        resetLeftEncoderCnt();
        resetRightEncoderCnt();
        First_Load();
        Current_State = Just_Shot;
      }
      break;

    case Just_Shot:
      if (Beacon == 1) {
        if (Left_En < 60) {
          setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorSpeed(LEFT_MOTOR, normalSpeed);
          setMotorSpeed(RIGHT_MOTOR, 0);
          Left_En = getEncoderLeftCnt();
        } else {
          Beacon = 0;
          Back_Aligning();
          Current_State = Home;
          break;
        }
      }

      if (Beacon == 2) {
        Beacon = 0;
        Back_Aligning();
        Current_State = Home;
        break;
      }

      if (Beacon == 3) {
        if (Left_En < 60) {
          setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
          setMotorSpeed(LEFT_MOTOR, normalSpeed);
          setMotorSpeed(RIGHT_MOTOR, 0);
          Left_En = getEncoderLeftCnt();
        } else {
          Beacon = 0;
          Back_Aligning();
          Current_State = Home;
          break;
        }
        Beacon = 0;
        Back_Aligning();
        Current_State = Home;
      }
      break;

    case Home:
      if (Beacon == 0) {
        IR_Sensors();
      }
      if (Beacon == 1) {
        Current_State = Left_Beacon;
      }
      if (Beacon == 2) {
        Current_Time = millis();
        Current_State = Shooting;
      }
      if (Beacon == 3) {
        Current_State = Right_Beacon;
      }

      if (Beacon == 4) {

        if ( (millis() - Current_Time) < Wiggle_Time && IRStateL == 1 && Wiggle_Counter == 0) {
          setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
          setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorSpeed(BOTH_MOTORS, normalSpeed);
          IRStateL = digitalRead(IRleft);
          Current_Time2 = millis();
        } else if (millis() - Current_Time2 < Wiggle_Time && Wiggle_Counter == 0 ) {
          setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
          setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
          setMotorSpeed(BOTH_MOTORS, normalSpeed);
        } else if (IRStateL == 0 && Wiggle_Counter == 0) {
          Beacon = 1;
        } else if (Wiggle_Counter == 0) {
          Beacon = 0;
          Wiggle_Counter = 1;
        }
      }
      if ( (millis() - Current_Time) < Wiggle_Time && IRStateR == 1 && Wiggle_Counter == 1) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, normalSpeed);
        IRStateR = digitalRead(IRright);
        Current_Time2 = millis();
      } else if (millis() - Current_Time2 < Wiggle_Time && Wiggle_Counter == 1 ) {
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorSpeed(BOTH_MOTORS, normalSpeed);
      } else if (IRStateR == 0 && Wiggle_Counter == 1) {
        Beacon = 3;
      } else if (Wiggle_Counter == 1) {
        Beacon = 0;
        Wiggle_Counter = 0;
      }

      if (sensorVal[4] < 1000 && sensorVal[3] < 1000 && sensorVal[2] < 1000 && sensorVal[5] < 1000) {
        Repeat = false;
      }
      Serial.print("Beacon: ");
      Serial.println(Beacon);
      break;
  }
  Serial.println(Current_State);
  Touch_Val = analogRead(Touch_Pin);
  if ( (Touch_Val > 200 ) && Current_State != Shooting ) {
    analogWrite(Enable_Motor, 0);
  } else {
    analogWrite(Enable_Motor, Motor_Speed);
  }

}
