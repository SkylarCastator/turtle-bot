/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Manages the data coming in from the ultrasonic distance sensor
**/
#include "TimerOne.h"

#define TRIGGER_PIN 12
#define ECHO_PIN 13 

void setupDistanceSensor()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
}

int updateDistanceSensor()
{
  int distance = 0;
  int average = 0;

  for (int i = 0; i < 4; i++)
  {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    distance = pulseIn(ECHO_PIN,HIGH);
    distance  = distance /74 /2;
    average += distance;
    delay(10);
  }
  distance = average /4;

  Serial.print("u ");
  Serial.print(distance);
  Serial.print("\n");
  return distance;
}