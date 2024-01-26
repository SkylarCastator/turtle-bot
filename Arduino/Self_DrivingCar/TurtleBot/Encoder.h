#include <Arduino.h>
#include "TimerOne.h"

class Encoder{
  private:
    byte interruptPin;
    int encoderN;
    unsigned long timer = 0;
    unsigned long measureTimer = 100;
    volatile unsigned int counter;
    static Encoder * instances [2];
    float wheelRadius = 1.5f;
    float wheelCircumfrence = 2 * M_PI *wheelRadius;
    float distance = 0;

  public:
    Encoder(byte interruptPin, int encoderN);
    void update(unsigned long time);
    void interruptTimer();
    static void interruptEvent0();
    static void interruptEvent1();
    void countUp();
};