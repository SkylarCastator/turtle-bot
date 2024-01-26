#include <Arduino.h>

class Motor{
  private:
    byte enablePin;
    byte motorPin1;
    byte motorPin2;
  public:
    Motor(byte enablePin, byte motorPin1, byte motorPin2);
    void enableMotor();
    void disableMotor();
    void driveForward();
    void driveReverse();
};