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

SoftwareSerial EEBlue(2,4); //RX and TX
QMC5883LCompass compass;

class RobotData
{
  public :
   String name = "Turtle";
   int battery = 100;
   int azimuth = 0; //Magntometer Direction 180 -> -180
   char directionArray[3]; // Gives the direction like _N_
  
   int kp_v = -100; //velocity setting
   int kd_v = -5; //Derivitive Velocity Setting
   int ki_v = 0; //Integral Velocity Setting
   int kp_w = 4; // Angular Velocity Setting
   int kd_w = 4; //Derivitive Angular Velocity Setting
   int ki_w = 0; //Integral Angular Velocity Setting
};

RobotData robot;

float feedForwardScaler = 10000;

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

float wheel_base = 100; // Space between Wheels mm
float wheel_dia = 65; //Wheel diminsion mm

double vVal, angleVal;

boolean newData = false;
const byte numChars = 32; //Length of Bluetooth Message
char receivedChars[numChars];
char tempChars[numChars];

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

const int numReadings = 10;
int readingsRight[numReadings];
int readingsLeft[numReadings];
int readingCounter = 0;

void setup () 
{
  for (int i = 0; i< numReadings; i++)
  {
    readingsLeft[i] = 0;
    readingsRight[i] = 0;
  }
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
  Timer1.initialize(10000); // set timer for 0.01 sec
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

  tickEncoder(ENCODER_RIGHT_PIN, rightStateHigh, counterRight, rightMotorDirection);
  tickEncoder(ENCODER_LEFT_PIN, leftStateHigh, counterLeft, leftMotorDirection);

  bluetoothSerialization();
  parseBluetoothMessage();
  updatePID();
  motorControls();  
  delay(10);                          
}

void tickEncoder(int pinNum, boolean state, int counter, int direction){
  int pinVal = digitalRead(ENCODER_LEFT_PIN);
  if (pinVal == 1)
  {
    if (!state)
    {
      state = true;
      counter += direction;
    }
  }
  else
  {
    state = false;
  }
}

void interuptTimerOne()
{
  //Max RPM 9V 180
  Timer1.detachInterrupt();
  //Serial.print("Counter  Left : ");
  //Serial.println(counterLeft);
  //Serial.print("Counter  Right : ");
  //Serial.println(counterRight);
  
  //leftRPM = (6000.00) * (float(counterLeft) / float(ENCODER_N));
  //rightRPM = (6000.00) * (float(counterRight) / float(ENCODER_N));
  //vCurr = (leftRPM+rightRPM)/2;
  //Serial.print("Motor Left RPM : ");
  //Serial.println(leftRPM);

  //Serial.print("Motor Right RPM : ");
  //Serial.println(rightRPM);
  counterLeft = 0;
  counterRight = 0;
  Timer1.attachInterrupt(interuptTimerOne);
}

void sendDeviceValues()
{
  String message = "< 1, ";
  message.concat(robot.name);
  message.concat(", ");
  message.concat(robot.battery);
  message.concat(", ");
  message.concat(robot.azimuth);
  message.concat(", ");
  message.concat(robot.directionArray[0]);
  message.concat(", ");
  message.concat(robot.directionArray[1]);
  message.concat(", ");
  message.concat(robot.directionArray[2]);
  message.concat(">");
  //EEBlue.write(message);
}

void sendPIDValues()
{
  String message = "< 2, ";
  message.concat(robot.kp_v);
  message.concat(", ");
  message.concat(robot.kd_v);
  message.concat(", ");
  message.concat(robot.ki_v);
  message.concat(", ");
  message.concat(robot.kp_w);
  message.concat(", ");
  message.concat(robot.kd_w);
  message.concat(", ");
  message.concat(robot.ki_w);
  message.concat(">");
  //EEBlue.write(message);
}

void bluetoothSerialization(){
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
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
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

  void parseBluetoothMessage()
  {
   if (newData == true) {
        strcpy(tempChars, receivedChars);
        //Serial.println(receivedChars);
        char * strtokIndx;
        strtokIndx = strtok(tempChars,",");
        int messageType = atoi(strtokIndx);
        if (messageType == 1){
            strtokIndx = strtok(NULL, ",");
            double x = atof(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            double y = atof(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            double phi = atof(strtokIndx);
            vDest = sqrt(pow(x, 2) + pow(y, 2));
            if (vDest != 0){
              angleDest = phi * 180 / PI;
              angleDest -= 90;
              if (angleDest < -180)
              {
                angleDest += 360;
              }
              angleDest *= -1;
            }
            else
            {
              angleDest = angleDest;
            }
        }
        else if(messageType == 2)
        {
            strtokIndx = strtok(NULL, ",");
            int saveFlag = atoi(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            robot.kp_v = -1 * atoi(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            robot.kd_v = -1 * atoi(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            robot.ki_v = -1 * atoi(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            robot.kp_w = atoi(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            robot.kd_w = atoi(strtokIndx);
            strtokIndx = strtok(NULL, ",");
            robot.ki_w = atoi(strtokIndx);
        }
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
  vVal = float(robot.kp_v)*(vErr) + float(robot.kd_v)*(vErrDot) + float(robot.ki_v)*(vErrSum);
  vErrLast = vErr;

  double angleErr = findShortestPath(angleCurr, angleDest);
  angleErrSum += (angleErr * timeChange);
  double wErrDot = (angleErr-angleErrLast) / timeChange;
  angleVal = float(float(robot.kp_w))*(angleErr) + float(float(robot.kd_w))*(wErrDot) + float(float(robot.ki_w))*(angleErrSum);
  angleErrLast = angleErr;

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
  int velocity = (vDest * -feedForwardScaler);
  //Serial.println("velocity");
  //Serial.println(velocity);
  //Serial.println("AngleVal");
  //Serial.println(angleVal);

  int vr_dest = (2*velocity+(angleVal*wheel_base))/wheel_dia;
  int vl_dest = (2*velocity-(angleVal*wheel_base))/wheel_dia;

  //Serial.println("vr_dest");
  //Serial.println(vr_dest);
  //Serial.println("vl_dest");
  //Serial.println(vl_dest);

  int leftTotal = 0;
  int rightTotal = 0;
  int averageRight = 0;
  int averageLeft = 0;
  readingsRight[readingCounter] = vr_dest;
  readingsLeft[readingCounter] = vl_dest;
  for (int i = 0; i < numReadings; i++)
  {
    rightTotal += readingsRight[i];
    leftTotal += readingsLeft[i];
  }
  averageLeft = leftTotal / readingCounter;
  averageRight = rightTotal / readingCounter;
  readingCounter = readingCounter + 1;
  if (readingCounter >= numReadings)
  {
    readingCounter = 0;
  }

  driveArdumoto(MOTOR_A, averageRight);
  driveArdumoto(MOTOR_B, averageLeft);
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
  Serial.println(spd);
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