#include "UltrasonicRangeSensor.h"

  UltrasonicRangeSensor::UltrasonicRangeSensor(byte triggerPin, byte echoPin) {
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
  }

  void UltrasonicRangeSensor::init()
  {
    pinMode(this->echoPin, INPUT);
    pinMode(this->triggerPin, OUTPUT);
  }

  int UltrasonicRangeSensor::getDistance()
  {
    int distance = 0;
    int average = 0;

    for (int i = 0; i < 4; i++)
    {
      digitalWrite(this->triggerPin, LOW);
      delayMicroseconds(2);

      digitalWrite(this->triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(this->triggerPin, LOW);

      distance = pulseIn(this->echoPin,HIGH);
      distance  = distance /74 /2;
      average += distance;
      delay(10);
    }
    distance = average /4;

    return distance;
  }