/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Code to manage motor inputs
**/

#define ENABLE_A 5
#define MOTOR_A1 6
#define MOTOR_A2 7
#define ENABLE_B 8
#define MOTOR_B1 9
#define MOTOR_B2 10

enum motion {
    FORWARD, REVERSE, TURNING, STOPPED  };

const unsigned long reverseTimer = 500;
const unsigned long turningTimer = 200;

motion motorState;
unsigned long stateStartTime = 0;

void setupMotors()
{
  pinMode(ENABLE_A, OUTPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(ENABLE_B, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  driveForward();
}

void updateMotors(unsigned long time)
{
  if (motorState == FORWARD)
  {
    stateStartTime = time;
  }
  else if (motorState == REVERSE)
  {
    if ((time - stateStartTime) > reverseTimer)
    {
      stateStartTime = time;
      if (random(2) == 0) {  // Generates 0 or 1, randomly        
        turnRight();  // Turn right for one second
      }
      else {
        turnLeft();  // Turn left for one second
      }
    }
  }
  else if (motorState == TURNING)
  {
    if ((time - stateStartTime) > turningTimer)
    {
      stateStartTime = time;
      driveForward();
    }
  }
}

void hitObject (unsigned long time)
{
    stateStartTime = time;
    driveReverse();
}

void enableMotors()
{
  digitalWrite (ENABLE_A, HIGH);
  digitalWrite (ENABLE_B, HIGH);
}

void driveForward() {
  enableMotors();
  motorState = FORWARD;
  digitalWrite (MOTOR_A1, LOW);
  digitalWrite (MOTOR_A2, HIGH);
  digitalWrite (MOTOR_B1, LOW);
  digitalWrite (MOTOR_B2, HIGH);
}

void driveReverse() {
  enableMotors();
  motorState = REVERSE;
  digitalWrite (MOTOR_A1,HIGH);
  digitalWrite (MOTOR_A2,LOW);  
  digitalWrite (MOTOR_B1,HIGH);
  digitalWrite (MOTOR_B2,LOW);  
}

void turnRight() {
  enableMotors();
  motorState = TURNING;
  digitalWrite (MOTOR_A1, LOW);
  digitalWrite (MOTOR_A2, HIGH);
  digitalWrite (MOTOR_B1,HIGH);
  digitalWrite (MOTOR_B2,LOW); 
}

void turnLeft() {
  enableMotors();
  motorState = TURNING;
  digitalWrite (MOTOR_A1,HIGH);
  digitalWrite (MOTOR_A2,LOW);  
  digitalWrite (MOTOR_B1, LOW);
  digitalWrite (MOTOR_B2, HIGH);
}

void disableMotors() {
  digitalWrite (ENABLE_A, LOW);
  digitalWrite (ENABLE_B, LOW);
  motorState = STOPPED;
}

