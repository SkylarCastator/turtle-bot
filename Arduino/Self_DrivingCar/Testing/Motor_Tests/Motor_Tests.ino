/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Drive the motors forward and backwards with a delay
**/
//Motor A
#define ENABLE_A = 5;
#define MOTOR_A1 = 6;
#define MOTOR_A2 = 7;

//Motor B
#define ENABLE_B = 8;
#define MOTOR_B1 = 9;
#define MOTOR_B2 = 10;

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
