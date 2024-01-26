#include "Encoder.h"

Encoder::Encoder(byte interruptPin, int encoderN)
{
  this->interruptPin = interruptPin;
  this->encoderN = encoderN;
  pinMode(this->interruptPin, INPUT);
  switch (interruptPin)
  {
    case 2:
      attachInterrupt(digitalPinToInterrupt(this->interruptPin), interruptEvent0, RISING);
      this->instances[0] = this;
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(this->interruptPin), interruptEvent1, RISING);
      this->instances[1] = this;
      break;
  }
  
}

void Encoder::update(unsigned long time)
{
  if ((time - this->timer) > this->measureTimer)
  {
    this->timer = time;
    interruptTimer();
  }
}

static void Encoder::interruptEvent0()
{
  if (Encoder::instances [0] != NULL)
  {
    Encoder::instances[0]->countUp();
  }
}

static void Encoder::interruptEvent1()
{
  if (Encoder::instances [1] != NULL)
  {
    Encoder::instances[1]->countUp();
  }
}

void Encoder::countUp()
{
  this->counter++;
}

void Encoder::interruptTimer()
{
  Timer1.detachInterrupt();
  this->distance = this->wheelCircumfrence * (float(this->counter) / float(this->encoderN));
  this->counter = 0;
}


