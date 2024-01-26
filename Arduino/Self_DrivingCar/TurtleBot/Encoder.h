#include <Arduino.h>
#include "TimerOne.h"

class Encoder{
  private:
    byte interruptPin;
    int encoderN;
    unsigned int counter;
    unsigned long timer = 0;
    unsigned long measureTimer = 100;
    float rpm;

  public:
    Encoder(byte interruptPin, int encoderN);
    void update(unsigned long time);
    void interruptTimer();
    void interruptEvent();
};