#include "TimerOne.h"

#define TRIGGER_PIN 12
#define ECHO_PIN 13 

void Setup_Ultrasonic()
{
  pinMode(ECHO_PIN,INPUT);
  pinMode(TRIGGER_PIN,OUTPUT);
}

int Update_Ultrasonic()
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

  int distance_copy = distance;
  char str[] = "u ";
  char str_dist[10];

  sprintf(str_dist, "%d", distance_copy);\
  char add_new_line[] = "\n";
  strcat(str_dist, add_new_line);
  strcat(str, str_dist);

  EEBlue.write(str);
  return distance;
}