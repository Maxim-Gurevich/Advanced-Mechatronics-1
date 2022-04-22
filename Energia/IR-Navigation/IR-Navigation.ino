int IRmid = 5;
int IRleft = 6;
int IRright = 4;
volatile byte state = LOW;


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
}

/*NOTE: This is not good code. It's blocking code due to the delay. Turn this into
        non-blocking code by either defining a timer as in BlinkWithoutDelay or
        attaching an interrupt. If you want an interrupt, you can use any unused
        (white) P4.x pins. That information is found in both the base pin map and User Guide.
        Software guidance for using an interrupt can be found here:
        https://energia.nu/reference/en/language/functions/external-interrupts/attachinterrupt/
*/
void loop() {

  int IRStateM = digitalRead(IRmid);
  int IRStateL = digitalRead(IRleft);
  int IRStateR = digitalRead(IRright);
  Serial.println(' ');
  Serial.print(IRStateL); // Note - it's 0 if beacon is ON, 1 if beacon is OFF. Do some if or case statements to print out more meaningful information.
  Serial.print(IRStateM);
  Serial.print(IRStateR);
}

void blink() {
  state = !state;
}
