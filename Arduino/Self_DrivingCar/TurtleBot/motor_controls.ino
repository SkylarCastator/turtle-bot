
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

