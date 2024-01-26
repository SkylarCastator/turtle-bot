#include "Encoder.h"

Encoder::Encoder(byte interruptPin, int encoderN)
{
  this->interruptPin = interruptPin;
  this->encoderN = encoderN;
  pinMode(this->interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(this->interruptPin), interruptEvent, RISING);
}

void Encoder::update(unsigned long time)
{
  if ((time - this->timer) > this->measureTimer)
  {
    this->timer = time;
    interruptTimer();
  }
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
}


