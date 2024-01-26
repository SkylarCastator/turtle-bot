#include <Arduino.h>
#include "Motor.h"

#define ENABLE_A 5
#define MOTOR_A1 6
#define MOTOR_A2 7
#define ENABLE_B 8
#define MOTOR_B1 9
#define MOTOR_B2 10

class Controller{
  private:
    const unsigned long reverseTimer = 500;
    const unsigned long turningTimer = 200;
    unsigned long stateStartTime = 0;
    Motor motorLeft = Motor(ENABLE_A, MOTOR_A1, MOTOR_A2);
    Motor motorRight = Motor(ENABLE_B, MOTOR_B1, MOTOR_B2);

  public:
    enum motion {
      FORWARD, REVERSE, TURNING, STOPPED  };
    motion motorState;
    Controller();
    void updateMotors(unsigned long time);
    void hitObject(unsigned long time);
    void driveForward();
    void driveReverse();
    void enableMotors();
    void disableMotors();
    void turnRight();
    void turnLeft();
};