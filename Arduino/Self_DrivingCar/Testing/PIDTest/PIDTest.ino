#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "TimerOne.h"

#define ENCODER_LEFT_PIN 7
#define ENCODER_RIGHT_PIN 5
#define ENCODER_N 20
#define MAX_RPM_9V 180

JsonDocument doc; //Manages Control Messages from the Phone
JsonDocument sendDoc;

SoftwareSerial EEBlue(2,4); //RX and TX
QMC5883LCompass compass;

class RobotData
{
  public :
   int azimuth = 0; //Magntometer Direction 180 -> -180
   char directionArray[3]; // Gives the direction like _N_
   int calibrated = 0; //Used for creating the offset
  
   int kp_v = 20; //velocity setting
   int kd_v = 10; //Derivitive Velocity Setting
   int ki_v = 0; //Integral Velocity Setting
   int kp_w = 50; // Angular Velocity Setting
   int kd_w = 10; //Derivitive Angular Velocity Setting
   int ki_w = 0; //Integral Angular Velocity Setting
};

RobotData robot;

float vCurr = 0; //Current Velocity
float vDest = 0; //Destinatoin Velocity
double vErrSum, vErrLast;

float angleCurr = 0; //Current Angular Velocity
float angleDest = 0; //Destination Angular Velocity
double angleErrSum, angleErrLast;
int angleErrHistory[20];
int angleErrAvr = 0;
int angleErrCounter = 0;

unsigned long lastTime; //Last update for PID Controller

float wheel_base = 0.1; // Space between Wheels mm
float wheel_dia = 0.065; //Wheel diminsion mm

double vVal, angleVal;

boolean newData = false;
const byte numChars = 128; //Length of Bluetooth Message
char receivedChars[numChars];

#define FORWARD  0
#define REVERSE 1
#define MOTOR_A 0
#define MOTOR_B 1
#define DIRA 12 // Direction control for motor A
#define PWMA 3  // PWM control (speed) for motor A
#define DIRB 13 // Direction control for motor B
#define PWMB 11 // PWM control (speed) for motor B

bool rightStateHigh = false;
bool leftStateHigh = false;
int counterRight = 0;
int counterLeft = 0;
int rightMotorDirection = 1; // Forward
int leftMotorDirection = 1; // Forward
float leftRPM = 0;
float rightRPM = 0;

void setup () 
{
  Serial.begin(9600);
  EEBlue.begin(9600);
  robot = RobotData();
  for(uint8_t i = 0; i < sizeof(angleErrHistory); ++i)
    angleErrHistory[i] = 0;
  Serial.println("The Bluetooth gates are open.");
  Serial.println("Connect to HC-05 with 1234 as key");
  compass.init();
  setupArdumoto();
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  Timer1.initialize(100000); // set timer for 1 sec
  Timer1.attachInterrupt(interuptTimerOne);
}

void loop () 
{
	char directionArray[3];
	compass.read();
  int a = compass.getAzimuth();
  robot.azimuth = a;
	compass.getDirection(robot.directionArray, a);
  angleCurr = robot.azimuth;

  int rightPinState = digitalRead(ENCODER_RIGHT_PIN);
  if (rightPinState == 1)
  {
    if (!rightStateHigh)
    {
      rightStateHigh = true;
      counterRight += rightMotorDirection;
    }
  }
  else
  {
    rightStateHigh = false;
  }

  int leftPinState = digitalRead(ENCODER_LEFT_PIN);
  if (leftPinState == 1)
  {
    if (!leftStateHigh)
    {
      leftStateHigh = true;
      counterLeft += leftMotorDirection;
    }
  }
  else
  {
    leftStateHigh = false;
  }

  bluetoothSerialization();
  updatePID();
  motorControls();
 // delay(20);                                
}

void interuptTimerOne()
{
  //Max RPM 9V 180
  Timer1.detachInterrupt();
  //Serial.print("Counter  Left : ");
  //Serial.println(counterLeft);
  //Serial.print("Counter  Right : ");
  //Serial.println(counterRight);
  
  leftRPM = (600.00) * (float(counterLeft) / float(ENCODER_N));
  rightRPM = (600.00) * (float(counterRight) / float(ENCODER_N));
  vCurr = (leftRPM+rightRPM)/2;
  //Serial.print("Motor Left RPM : ");
  //Serial.println(leftRPM);

  //Serial.print("Motor Right RPM : ");
  //Serial.println(rightRPM);
  counterLeft = 0;
  counterRight = 0;
  Timer1.attachInterrupt(interuptTimerOne);
}

void sendBluetoothMessage(){
  //sendDoc["azimuth"] = robot.azimuth;
  //sendDoc["calibrated"] = robot.calibrated;
  //sendDoc["directionArray"][0] = robot.directionArray[0];
  ///sendDoc["directionArray"][1] = robot.directionArray[1];
  //sendDoc["directionArray"][2] = robot.directionArray[2];
  //sendDoc["kp_v"] = robot.kp_v;
  ///sendDoc["kd_v"] = robot.kd_v;
  //sendDoc["ki_v"] = robot.ki_v;
  //sendDoc["kp_w"] = robot.kp_w;
  ///sendDoc["kd_w"] = robot.kd_w;
  //sendDoc["ki_w"] = robot.ki_w;
  //serializeJson(sendDoc, Serial);
  //EEBlue.write(Serial); 
}

void bluetoothSerialization(){
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '{';
    char endMarker = '}';
    char rc;
    while (EEBlue.available() > 0 && newData == false){
      rc = EEBlue.read();

      if (recvInProgress == true) {
       if (rc != endMarker) {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
        }
        else {
          receivedChars[ndx] = rc;
          ndx++;
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }
      else if (rc == startMarker) {
        recvInProgress = true;
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
    }

   if (newData == true) {
        Serial.println(receivedChars);
        DeserializationError error = deserializeJson(doc, receivedChars);
        if(error) {
          Serial.print("deserializeJson() returned ");
+         Serial.println(error.c_str());
          return;
        }
        double x = doc["x"];
        double y = doc["y"];
        double phi = doc["phi"];
        vDest = sqrt(pow(x, 2) + pow(y, 2)) * MAX_RPM_9V;
        angleDest = phi * 180 / PI;
        robot.kp_v = doc["v_kp"];
        robot.kd_v = doc["v_kv"];
        robot.ki_v = doc["v_ki"];
        robot.kp_w = doc["w_kp"];
        robot.kd_w = doc["w_kv"];
        robot.ki_w = doc["w_ki"];
        newData = false;
    } 
}

int findShortestPath(int currentAngle, int targetAngle) {
    // Normalize angles to be from 0 to 360
    int normalizedCurrent = (currentAngle + 360) % 360;
    int normalizedTarget = (targetAngle + 360) % 360;

    // Calculate clockwise and counter-clockwise distances
    int cwDistance = (normalizedTarget - normalizedCurrent + 360) % 360;
    int ccwDistance = (normalizedCurrent - normalizedTarget + 360) % 360;

    // Determine shortest path and adjust for original range (-180 to 180)
    if (cwDistance <= ccwDistance) {
        // Clockwise is shorter or equal
        return cwDistance > 180 ? cwDistance - 360 : cwDistance;
    } else {
        // Counter-clockwise is shorter
        return ccwDistance > 180 ? 360 - ccwDistance : -ccwDistance;
    }
}


void updatePID()
{
  //Controls for Velocity
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  double vErr = vDest-vCurr;
  vErrSum += (vErr * timeChange);
  double vErrDot = (vErr-vErrLast) / timeChange;
  vVal = float(robot.kp_v/10)*(vErr) + float(robot.kd_v/10)*(vErrDot) + float(robot.ki_v/10)*(vErrSum);
  vErrLast = vErr;

  double angleErr = findShortestPath(angleCurr, angleDest);//angleDest-angleCurr; // Manage for around the curve -180 -> 180
  angleErrSum += (angleErr * timeChange);
  double wErrDot = (angleErr-angleErrLast) / timeChange; //Filter by average
  angleErrHistory[angleErrCounter] = wErrDot;
  int arraySum = 0;
  for(int index = 0; index < sizeof(angleErrHistory); index++)
  { 
    arraySum += angleErrHistory[index]; 
  }
  angleErrAvr = arraySum / sizeof(angleErrHistory);
  angleVal = float(robot.kp_w/10)*(angleErr) + float(robot.kd_w/10)*(wErrDot) + float(robot.ki_w/10)*(angleErrSum);
  angleErrLast = angleErr;
  angleErrCounter += 1;
  if (angleErrCounter >= sizeof(angleErrHistory))
  {
    angleErrCounter = 0;
  }
  lastTime = now;
}



void savePIDs()
{
  //for(int i = 0; i < strlen(str); i++)
  //{
	//byte byteAtCurrentStringPosition = (byte) str[i];
	//EEPROM.write(ADDRESS_OFFSET + i, byteAtCurrentStringPosition);
  //}
  //EEPROM.write(ADDRESS_OFFSET + strlen(str) + 1, ‘\0’);
}

void motorControls()
{
  //Serial.println(vVal);
  //Serial.println(angleVal);
  int vr_dest = (2*vVal+(angleVal*wheel_base))/wheel_dia;
  int vl_dest = (2*vVal-(angleVal*wheel_base))/wheel_dia;
  driveArdumoto(MOTOR_A, vr_dest);
  driveArdumoto(MOTOR_B, vl_dest);
}

void driveArdumoto(byte motor, int spd)
{
  int dir = 1;
  if (motor == 0)
  {
    rightMotorDirection = 1;
  }
  else{
    leftMotorDirection = 1;
  }
  if (spd < 0){
    dir = 0;
    spd *= -1;
    if (motor == 0)
    {
      rightMotorDirection = -1;
    }
    else{
      leftMotorDirection = -1;
    }
  }
  spd = min(spd, 255);  
  if (spd < 30){
    spd = 0;
  }
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

void setupArdumoto()
{
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}