#include <SoftwareSerial.h>
SoftwareSerial EEBlue(2,3);

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

void setup() {
  randomSeed(analogRead(3));
  Setup_Ultrasonic();
  Setup_Serial(9600);
  Setup_Motors();
}

void loop() {
  int distance = Update_Ultrasonic();
  Read_From_Serial();
  Move_Robot(distance);
}
