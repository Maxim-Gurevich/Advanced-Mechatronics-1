#define trigPin 12
#define echoPin 13

void setup() {
  Serial.begin(9600);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
}

void loop() {
int duration;
float distance;
int conversion;
digitalWrite(trigPin,HIGH);
delayMicroseconds(1000);
digitalWrite(trigPin,LOW);
duration = pulseIn(echoPin,HIGH);
distance = (duration/2) / 74.07;

Serial.println(distance);
delay(100);
  
}
