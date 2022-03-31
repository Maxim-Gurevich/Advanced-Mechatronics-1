/*
   DC Motor Control Template
   Written By: Ann Majewicz Fey, Perry Scott, and inspired by online resources.

   In this code, you will first convert the encoder reading to revolutions per second
   and then you will take that variable as an input to you PID controller (already programmed
   as a function you can use). First, you will need to use Zeigler Nicholas and brute force
   to tune the gains of your PID controller. Notice how your motor response is affected by the
   different gain terms. Spend some time playing around with it to get a better understanding
   of real-time control systems!
*/

// Constant Definitions
int countsPerRev = 48;
double maxSpeed = 66.7; //in rev/sec (this is found experimentally, you may need to confirm the value)
double delayTime = 20; // in ms
int encoder0PosPrev = 0;
int encoderVel = 0;
int CommandSpeed = 0;

// Pin Definitions
const int PWMoutp = 19;
const int PWMoutn = 34;
const int PWMspeedPin = 38;
const int potentiometerPWMinput = 33;
const int encoder0PinA = 5;
const int encoder1PinA = 36;
const int encoder0PinB = 6;
const int encoder1PinB = 37;


// Variable Definitions
volatile signed int encoder0Pos = 0;
signed int encoderPosLast = 0;
int potPWMval = 0;
int motorPWMval = 0;

//PID constants (Use the Ziegler Nicholas Tuning Method as a first guess)
double kp = 4;
double ki = .05;//0.05
double kd = 0;

// PID Variables
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastSpeedError;
double motorSpeed, newMotorSpeed, setPoint;
double cumError, rateError;

void setup()
{
  // Don't Change
  Serial.begin(115200);
  pinMode(potentiometerPWMinput, INPUT_PULLUP);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(PWMoutp, OUTPUT);
  pinMode(PWMoutn, OUTPUT);
  pinMode(PWMspeedPin, OUTPUT);
  attachInterrupt(encoder0PinA, doEncoderA, RISING); // Interrupt is fired whenever button is pressed
  attachInterrupt(encoder1PinA, doEncoderA, FALLING);
  attachInterrupt(encoder0PinB, doEncoderB, RISING);
  attachInterrupt(encoder1PinB, doEncoderB, FALLING);

  // Pre-set the direction of the motors
  digitalWrite(PWMoutp, HIGH);
  digitalWrite(PWMoutn, LOW);
}


void loop()
{
    // USE FOR PID CONTROL
  setPoint = 30;                         //set point at zero rev/second

  // POTENTIOMETER CONTROL

  potPWMval = analogRead(potentiometerPWMinput);
  potPWMval = map(potPWMval, 0, 1023, -255, 255);
  if (abs(potPWMval) < 150) {
    potPWMval = 0;
  }
  //potPWMval = map(potPWMval,0,1023,-maxSpeed,maxSpeed);
  //setPoint = potPWMval;
  if (potPWMval < 0) {
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  }
  else
  {
    digitalWrite(PWMoutp, LOW);
    digitalWrite(PWMoutn, HIGH);
  }
  ///////////////////////////////////////////
  //CommandSpeed=encoderVel*Kp
  if (millis() > 0000 && millis() < 10000) {
    CommandSpeed = computePID(encoderVel);
    if (CommandSpeed > 255) {
    CommandSpeed = 255;
  } else if(CommandSpeed < -255) {
    CommandSpeed = -255;
  }
    Serial.print(encoderVel); // CHANGE THIS TO PLOT MOTOR SPEED
    Serial.print(" ,");
    Serial.println(CommandSpeed);
  } else {
    CommandSpeed = 0;
  }
  ///////////////////////////////////////////


  if (CommandSpeed != 0) {
    analogWrite(PWMspeedPin, abs(CommandSpeed));
  } else {
    analogWrite(PWMspeedPin, abs(potPWMval));
  }
  if (CommandSpeed > 0) {
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  } else {
    digitalWrite(PWMoutp, LOW);
    digitalWrite(PWMoutn, HIGH);
  }



  motorSpeed = -10; // Here you will need to compute the motor speed in counts/rev.
  // Hint: you just need to convert the encoder counts to Rev/Sec!

  // HERE YOU WILL WRITE YOUR CODE TO ENABLE THE PID CONTROLLER. Use the function below!!
  /* Initially, just have the PID controller follow the pre-defined
      setPoint variable. For fun, you can later consider how to tie the PID
      controller to the potenitometer value to have an adaptively changing setPoint.*/
  //analogWrite(PWMspeedPin, computePID(motorSpeed));

  //print out speed (currently is only plotting the pot value)
  encoderVel = encoder0Pos - encoder0PosPrev;
  encoder0PosPrev = encoder0Pos;


  encoderPosLast = encoder0Pos;
  delay(delayTime); // This is a delay to time the control loop. In the future, this should be a non-blocking version.

}

// **** DONT CHANGE THE FUNCTIONS BELOW ****
//================
// READ ENCODER A
//================
void doEncoderA() {
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  {
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}
//================
// READ ENCODER B
//================
void doEncoderB() {
  if (digitalRead(encoder0PinB) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinA) == LOW) {  // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos + 1;         // CCW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  {
    if (digitalRead(encoder0PinA) == LOW) {   // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos - 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos + 1;          // CCW
    }

  }
}
//================
// PID CONTROL FUNCTION (use this!)
//================
double computePID(double inp) {
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = setPoint - inp;                         // determine error
  cumError += error * elapsedTime;                // compute integral
  //if (cumError > 1000) {
 //   cumError = 1000;
 // } else if(cumError < -1000) {
 //   cumError = -1000;
 // }
  rateError = (error - lastSpeedError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cumError + kd * rateError; //PID output

  lastSpeedError = error;                            //remember current error
  previousTime = currentTime;                        //remember current time
  Serial.println(cumError);
  return out;                                        //have function return the PID output
}
