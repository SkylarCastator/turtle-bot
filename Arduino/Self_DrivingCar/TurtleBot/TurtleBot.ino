#include "TimerOne.h"
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

#include <SoftwareSerial.h>
SoftwareSerial EEBlue(4,11);

#define ENCODER_LEFT_PIN 2
#define ENCODER_RIGHT_PIN 3
#define ENCODER_N 20

unsigned int counter1 = 0;
unsigned int counter2 = 0;
float Motor_Left_RPM = 0;
float Motor_Right_RPM = 0;

void ISR_count1()
{
  counter1++;
}

void ISR_count2()
{
  counter2++;
}

#define TRIGGER_PIN 12
#define ECHO_PIN 13 

void Setup_Ultrasonic()
{
  pinMode(ECHO_PIN,INPUT);
  pinMode(TRIGGER_PIN,OUTPUT);
}

#define enableA 5
#define MotorA1 6
#define MotorA2 7

#define enableB 8
#define MotorB1 9
#define MotorB2 10

void Setup_Motors()
{
  pinMode(enableA, OUTPUT);
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);

  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  
  Timer1.initialize(1000000); // set timer for 1 sec
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), ISR_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), ISR_count2, RISING);
  Timer1.attachInterrupt(ISR_timerone);

  delay(200);
  go_forward();
}

void Setup_Serial(int baud_rate)
{
  Serial.begin(baud_rate);
  EEBlue.begin(baud_rate);
}

int Update_Ultrasonic()
{
  int distance = 0;
  int average = 0;

  for (int i = 0; i < 4; i++)
  {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    distance = pulseIn(ECHO_PIN,HIGH);
    distance  = distance /74 /2;
    average += distance;
    delay(10);
  }
  distance = average /4;
  Serial.print("u ");
  Serial.print(distance);
  Serial.print("\n");

  int distance_copy = distance;
  char str[] = "u ";
  char str_dist[10];

  sprintf(str_dist, "%d", distance_copy);\
  char add_new_line[] = "\n";
  strcat(str_dist, add_new_line);
  strcat(str, str_dist);

  EEBlue.write(str);
  return distance;
}

void Move_Robot(int distance)
{
  if (distance >= 0 && distance <= 2)
  {
    go_backwards();
    delay(500);
    /* Go left or right to avoid the obstacle*/
    if (random(2) == 0) {  // Generates 0 or 1, randomly        
      go_right();  // Turn right for one second
    }
    else {
      go_left();  // Turn left for one second
    }
    delay(400);
    go_forward();  // Move forward
  }
  delay(50);
}

void go_forward() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Move forward
  digitalWrite (MotorA1, LOW);
  digitalWrite (MotorA2, HIGH);
  digitalWrite (MotorB1, LOW);
  digitalWrite (MotorB2, HIGH);
 
}
void go_backwards() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Go backwards
  digitalWrite (MotorA1,HIGH);
  digitalWrite (MotorA2,LOW);  
  digitalWrite (MotorB1,HIGH);
  digitalWrite (MotorB2,LOW);  
   
}
void go_right() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Turn right
  digitalWrite (MotorA1, LOW);
  digitalWrite (MotorA2, HIGH);
  digitalWrite (MotorB1,HIGH);
  digitalWrite (MotorB2,LOW); 
}
void go_left() {
  //enabling motor A and B
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
   
  // Turn left
  digitalWrite (MotorA1,HIGH);
  digitalWrite (MotorA2,LOW);  
  digitalWrite (MotorB1, LOW);
  digitalWrite (MotorB2, HIGH);
}
void stop_all() {
  digitalWrite (enableA, LOW);
  digitalWrite (enableB, LOW);
}

void Read_From_Serial(){
  while(Serial.available() > 0)
  {
    EEBlue.write(Serial.read());
  }
}

void ISR_timerone()
{
  Timer1.detachInterrupt();
  Serial.print("Counter  1 : ");
  Serial.println(counter1);
  Serial.print("Counter  2 : ");
  Serial.println(counter2);

  Motor_Left_RPM = (60.00) * (float(counter1) / float(ENCODER_N));
  Motor_Right_RPM = (60.00) * (float(counter2) / float(ENCODER_N));
  Serial.print("Motor Left RPM : ");
  Serial.println(Motor_Left_RPM);

  Serial.print("Motor Right RPM : ");
  Serial.println(Motor_Right_RPM);
  counter1 = 0;
  counter2 = 0;
  Timer1.attachInterrupt(ISR_timerone);
}

void Read_Compass()
{
  int x, y, z, a, b;
	char myArray[3];
	
	compass.read();
  
	x = compass.getX();
	y = compass.getY();
	z = compass.getZ();
	
	a = compass.getAzimuth();
	
	b = compass.getBearing(a);

	compass.getDirection(myArray, a);
  
  
	Serial.print("X: ");
	Serial.print(x);

	Serial.print(" Y: ");
	Serial.print(y);

	Serial.print(" Z: ");
	Serial.print(z);

	Serial.print(" Azimuth: ");
	Serial.print(a);

	Serial.print(" Bearing: ");
	Serial.print(b);

	Serial.print(" Direction: ");
	Serial.print(myArray[0]);
	Serial.print(myArray[1]);
	Serial.print(myArray[2]);

	Serial.println();

}

void setup() {
  Wire.begin();
  randomSeed(analogRead(3));
  compass.init();
  Setup_Ultrasonic();
  Setup_Serial(9600);
  Setup_Motors();
}

void loop() {
  int distance = Update_Ultrasonic();
  Read_From_Serial();
  Move_Robot(distance);
  Read_Compass();
}
