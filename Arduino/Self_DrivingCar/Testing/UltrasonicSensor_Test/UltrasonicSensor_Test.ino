#define Trigger 12
#define Echo 13


void setup() {
  Serial.begin(9600);
  pinMode(Echo, INPUT);
  pinMode(Trigger, OUTPUT);

}

void loop() {
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);

  digitalWrite(Trigger,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger,LOW);

  int distance = pulseIn(Echo,HIGH);

// Speed of sound is:
  // 13511.811023622 inches per second
  // 13511.811023622/10^6 inches per microsecond
  // 0.013511811 inches per microsecond
  // Taking the reciprocal, we have:
  // 74.00932414 microseconds per inch 
  // Below, we convert microseconds to inches by 
  // dividing by 74 and then dividing by 2
  // to account for the roundtrip time.
  distance = distance / 74 / 2;

  Serial.println(distance);
  delay(100);
}
