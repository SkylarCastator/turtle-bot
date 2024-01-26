#include "Controller.h"

Controller::Controller()
{
  driveForward();
}

void Controller::updateMotors(unsigned long time)
{
  if (this->motorState == FORWARD)
  {
    this->stateStartTime = time;
  }
  else if (this->motorState == REVERSE)
  {
    if ((time - this->stateStartTime) > this->reverseTimer)
    {
      this->stateStartTime = time;
      if (random(2) == 0) {  // Generates 0 or 1, randomly        
        turnRight(); 
      }
      else {
        turnLeft();  
      }
    }
  }
  else if (this->motorState == TURNING)
  {
    if ((time - this->stateStartTime) > this->turningTimer)
    {
      this->stateStartTime = time;
      driveForward();
    }
  }
}

void Controller::hitObject (unsigned long time)
{
    this->stateStartTime = time;
    driveReverse();
}

void Controller::driveForward() {
  motorState = FORWARD;
  this->motorLeft.driveForward();
  this->motorRight.driveForward();
}

void Controller::driveReverse() {
  motorState = REVERSE;
  this->motorLeft.driveReverse();
  this->motorRight.driveReverse();
}

void Controller::turnRight() {
  motorState = TURNING;
  this->motorLeft.driveForward();
  this->motorRight.driveReverse();
}

void Controller::turnLeft() {
  motorState = TURNING;
  this->motorLeft.driveReverse();
  this->motorRight.driveForward();
}

void Controller::disableMotors() {
  this->motorLeft.disableMotor();
  this->motorRight.disableMotor();
  motorState = STOPPED;
}
