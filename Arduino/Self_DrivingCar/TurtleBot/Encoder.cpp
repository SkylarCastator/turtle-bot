#include "Encoder.h"

Encoder::Encoder(byte interruptPin, int encoderN)
{
  this->interruptPin = interruptPin;
  this->encoderN = encoderN;
}

void Encoder::init()
{
  pinMode(this->interruptPin, INPUT);
  
  Timer1.initialize(1000000); // set timer for 1 sec
  //attachInterrupt(digitalPinToInterrupt(this->interruptPin), this->interruptEvent, RISING);
  //Timer1.attachInterrupt(interruptTimer);
}

void Encoder::interruptEvent()
{
  this->counter++;
}

void Encoder::interruptTimer()
{
  Timer1.detachInterrupt();
  this->rpm = (60.00) * (float(this->counter) / float(this->encoderN));
  this->counter = 0;
  //Timer1.attachInterrupt(interruptTimer);
}


