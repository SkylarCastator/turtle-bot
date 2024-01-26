#include <Arduino.h>
#include "TimerOne.h"

class Encoder{
  private:
    byte interruptPin;
    int encoderN;
    unsigned int counter;
    float rpm;

  public:
    Encoder(byte interruptPin, int encoderN);
    void init();
    void interruptTimer();
    void interruptEvent();
};