#define ENCODER_LEFT_PIN 2
#define ENCODER_RIGHT_PIN 3
#define ENCODER_N 20

unsigned int counter1 = 0;
unsigned int counter2 = 0;
float Motor_Left_RPM = 0;
float Motor_Right_RPM = 0;

void Setup_Encoders()
{
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  
  Timer1.initialize(1000000); // set timer for 1 sec
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), ISR_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), ISR_count2, RISING);
  Timer1.attachInterrupt(ISR_timerone);
}

void ISR_count1()
{
  counter1++;
}

void ISR_count2()
{
  counter2++;
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
