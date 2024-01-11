//Motor A
int enableA = 5;
int MotorA1 = 6;
int MotorA2 = 7;

//Motor B
int enableB = 8;
int MotorB1 = 9;
int MotorB2 = 10;

void setup() {
  Serial.begin(9600);

  pinMode (enableA, OUTPUT);
  pinMode (MotorA1, OUTPUT);
  pinMode (MotorA2, OUTPUT);

  pinMode (enableB, OUTPUT);
  pinMode (MotorB1, OUTPUT);
  pinMode (MotorB2, OUTPUT);
}

void loop() {
  Serial.println("EnableMotors");
  digitalWrite(enableA, HIGH);
  digitalWrite(enableB, HIGH);

  Serial.println("Motion Forward");
  digitalWrite (MotorA1, LOW);
  digitalWrite (MotorA2, HIGH);
  digitalWrite(MotorB1, LOW);
  digitalWrite(MotorB2, HIGH);
  delay(3000);

  Serial.println("Motion Reverse");
  digitalWrite (MotorA1, HIGH);
  digitalWrite (MotorA2, LOW);
  digitalWrite(MotorB1, HIGH);
  digitalWrite(MotorB2, LOW);
  delay(3000);

  Serial.println("Stop Motors");
  digitalWrite(enableA, LOW);
  digitalWrite(enableB, LOW);
  delay(3000);

}
