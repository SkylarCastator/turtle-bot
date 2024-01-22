/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Manages the data coming in from the ultrasonic distance sensor
**/
#define TRIGGER 12
#define ECHO 13

void setup() {
  Serial.begin(9600);
  pinMode(ECHO, INPUT);
  pinMode(TRIGGER, OUTPUT);

}

void loop() {
  int distance = 0;
  int average = 0;

  for (int i = 0; i < 4; i++)
  {
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIGGER,HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER,LOW);

    // Speed of sound is:
    // 13511.811023622 inches per second
    // 13511.811023622/10^6 inches per microsecond
    // 0.013511811 inches per microsecond
    // Taking the reciprocal, we have:
    // 74.00932414 microseconds per inch 
    // Below, we convert microseconds to inches by 
    // dividing by 74 and then dividing by 2
    // to account for the roundtrip time.
    distance = pulseIn(ECHO_PIN,HIGH);
    distance  = distance /74 /2;
    average += distance;
    delay(10);
  }
  
  distance = average /4;

  Serial.print("u ");
  Serial.print(distance);
  Serial.print("\n");
  Serial.println(distance);
}
