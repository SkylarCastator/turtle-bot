#include "Motor.h"

Motor::Motor(byte enablePin, byte motorPin1, byte motorPin2)
{
  this->enablePin = enablePin;
  this->motorPin1 = motorPin1;
  this->motorPin2 = motorPin2;
  pinMode(this->enablePin, OUTPUT);
  pinMode(this->motorPin1, OUTPUT);
  pinMode(this->motorPin2, OUTPUT);
}

void Motor::enableMotor()
{
  digitalWrite (this->enablePin, HIGH);
}

void Motor::driveForward() {
  enableMotor();
  digitalWrite (this->motorPin1, LOW);
  digitalWrite (this->motorPin2, HIGH);
}

void Motor::driveReverse() {
  enableMotor();
  digitalWrite (this->motorPin1,HIGH);
  digitalWrite (this->motorPin2,LOW);   
}

void Motor::disableMotor() {
  digitalWrite (this->enablePin, LOW);
}