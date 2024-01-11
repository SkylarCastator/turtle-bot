#import <Wire.h>

void Setup_Compass();
void Setup_Encoders();
void Setup_Serial(int baud_rate);
void Setup_Ultrasonic();
void Setup_Motors();

int Update_Ultrasonic();
void Read_Compass();
void Read_From_Serial();
void Move_Robot(int distance);

void setup() {
  Wire.begin();
  //randomSeed(analogRead(3));
  Setup_Compass();
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
