/*
  Analog Input
  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED)  connected to digital pin 13.
  The amount of time the LED will be on and off depends on
  the value obtained by analogRead().

  The circuit:
   Potentiometer attached to analog input 0
   center pin of the potentiometer to the analog pin
   one side pin (either one) to ground
   the other side pin to +3.3V
   LED anode (long leg) attached to digital output 13
   LED cathode (short leg) attached to ground

   Note: because most Arduinos have a built-in LED attached
  to pin 13 on the board, the LED is optional.


  Created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  http://arduino.cc/en/Tutorial/AnalogInput

*/

int sensorPin = A10;    // select the input pin for the potentiometer
int buttonPin = A15;    // select the input pin for the potentiometer
int ledPin = 19;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
int buttonValue = 0;
boolean buttonDebounce = false;

// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

int buttonCount = 0;
void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  // initialize both serial ports:
  Serial.begin(9600);
}


void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  buttonValue = analogRead(buttonPin);
  Serial.print(buttonCount);
  Serial.println(buttonDebounce);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > sensorValue) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
  }

  if (buttonValue > 100 && !buttonDebounce) {
    buttonCount = buttonCount + 1;
    buttonDebounce = true;
  } else if (buttonValue < 100) {
    buttonDebounce = false;
  }
  // set the LED with the ledState of the variable:
  if (buttonCount > 1) {
    digitalWrite(ledPin, ledState);
  } else {
    digitalWrite(ledPin, HIGH);
  }

}
