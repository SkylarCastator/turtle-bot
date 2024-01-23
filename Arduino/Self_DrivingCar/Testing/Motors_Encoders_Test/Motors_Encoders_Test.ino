/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Manages the data coming in from the encoders for both motors
**/
#include "TimerOne.h"
#define ENCODER_LEFT_PIN 2
#define ENCODER_RIGHT_PIN 3
#define ENCODER_N 20

#define ENABLE_A 5
#define MOTOR_A1 6
#define MOTOR_A2 7
#define ENABLE_B 8
#define MOTOR_B1 9
#define MOTOR_B2 10


unsigned int counter1 = 0;
unsigned int counter2 = 0;
float leftRPM = 0;
float rightRPM = 0;

void setupEncoders()
{
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  
  Timer1.initialize(1000000); // set timer for 1 sec
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), inturruptEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), inturruptEncoder2, RISING);
  Timer1.attachInterrupt(interuptTimerOne);
}

void inturruptEncoder1()
{
  counter1++;
}

void inturruptEncoder2()
{
  counter2++;
}

void interuptTimerOne()
{
  Timer1.detachInterrupt();
  Serial.print("Counter  1 : ");
  Serial.println(counter1);
  Serial.print("Counter  2 : ");
  Serial.println(counter2);

  leftRPM = (60.00) * (float(counter1) / float(ENCODER_N));
  rightRPM = (60.00) * (float(counter2) / float(ENCODER_N));
  Serial.print("Motor Left RPM : ");
  Serial.println(leftRPM);

  Serial.print("Motor Right RPM : ");
  Serial.println(rightRPM);
  counter1 = 0;
  counter2 = 0;
  Timer1.attachInterrupt(interuptTimerOne);
}

void driveForward() 
{
  Serial.println("Motion Forward");
  digitalWrite (MOTOR_A1, LOW);
  digitalWrite (MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void driveReverse()
{
  Serial.println("Motion Reverse");
  digitalWrite (MOTOR_A1, HIGH);
  digitalWrite (MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void setup() 
{
  Serial.begin(9600);
  setupEncoders();
  pinMode (ENABLE_A, OUTPUT);
  pinMode (MOTOR_A1, OUTPUT);
  pinMode (MOTOR_A2, OUTPUT);

  pinMode (ENABLE_B, OUTPUT);
  pinMode (MOTOR_B1, OUTPUT);
  pinMode (MOTOR_B2, OUTPUT);
}

void loop() 
{
  Serial.println("EnableMotors");
  digitalWrite(ENABLE_A, HIGH);
  digitalWrite(ENABLE_B, HIGH);

  driveForward();
  delay(3000);

  driveReverse();
  delay(3000);

  Serial.println("Stop Motors");
  digitalWrite(ENABLE_A, LOW);
  digitalWrite(ENABLE_B, LOW);
  delay(3000);

}