#include <PID_v1.h>

double kp_v, kd_v, ki_v;
float kp_w, kd_w, ki_w;

float vCurr = 0;
float vDest = 0;
double vErrSum, vErrLast;

float wCurr = 0;
float wDest = 0;
double wErrSum, wErrLast;

unsigned long lastTime;

float wheel_base = 100; // Space between Wheels mm
float wheel_dia = 65; //Wheel diminsion mm

double vVal, wVal;


void updatePID()
{
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  double vErr = vDest-vCurr;
  vErrSum += (vErr * timeChange);
  double vErrDot = (vErr*vErrLast) / timeChange;
  vVal = kp_v*(vErr) + kd_v*(vErrDot) + ki_v*(vErrSum);
  vErrLast = vErr;

  double wErr = wDest-wCurr;
  wErrSum += (wErr * timeChange);
  double wErrDot = (wErr*wErrLast) / timeChange;
  wVal = kp_w*(wErr) + kd_w*(wErrDot) + ki_w*(wErrSum);
  vErrLast = vErr;

  lastTime = now;
}




#define FORWARD  0
#define REVERSE 1
#define MOTOR_A 0
#define MOTOR_B 1
#define DIRA 12 // Direction control for motor A
#define PWMA 3  // PWM control (speed) for motor A
#define DIRB 13 // Direction control for motor B
#define PWMB 11 // PWM control (speed) for motor B


void setup () 
{
  Serial.begin(9600);
  setupArdumoto();
}

void loop () 
{
  updatePID();
  int vr_dest = (2*vVal+(wVal*wheel_base))/wheel_dia;
  int vl_dest = (2*vVal-(wVal*wheel_base))/wheel_dia;
  if (vr_dest < 0)
  {
    driveArdumoto(MOTOR_A, REVERSE, vr_dest * -1);
  }
  else
  {
    driveArdumoto(MOTOR_A, FORWARD, vr_dest);
  }

   if (vl_dest < 0)
  {
    driveArdumoto(MOTOR_B, REVERSE, vl_dest * -1);
  }
  else
  {
    driveArdumoto(MOTOR_B, FORWARD, vl_dest);
  }
  delay(30);                                
}

void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}