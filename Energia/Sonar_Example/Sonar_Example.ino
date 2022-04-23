#define trigPin 11
#define echoPin 10

void setup() {
  Serial.begin(9600);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
}

void loop() {
int duration;
float distance;
int conversion;
digitalWrite(trigPin,LOW);
delayMicroseconds(2);
digitalWrite(trigPin,HIGH);
delayMicroseconds(10);
digitalWrite(trigPin,LOW);
delayMicroseconds(2);
duration = pulseIn(echoPin,HIGH,10000000UL);
distance = duration*100/5882;
Serial.println(duration);
//boolean boop=digitalRead(echoPin);
//Serial.println(boop);
delay(1000);
  
}
