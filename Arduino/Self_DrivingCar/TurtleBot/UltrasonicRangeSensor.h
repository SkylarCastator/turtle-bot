#include <Arduino.h>

class UltrasonicRangeSensor {
  private:
    byte triggerPin;
    byte echoPin;

  public:
    UltrasonicRangeSensor(byte triggerPin, byte echoPin);
    void init();
    int getDistance();
};