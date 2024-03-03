#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

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
  
   int kp_v = 6; //elocity setting
   int kd_v = 4; //Derivitive Velocity Setting
   int ki_v = 0; //Integral Velocity Setting
   int kp_w = 3; // Angular Velocity Setting
   int kd_w = 4; //Derivitive Angular Velocity Setting
   int ki_w = 0; //Integral Angular Velocity Setting
};

RobotData robot;

float vCurr = 0; //Current Velocity
float vDest = 0; //Destinatoin Velocity
double vErrSum, vErrLast;

float wCurr = 0; //Current Angular Velocity
float wDest = 0; //Destination Angular Velocity
double wErrSum, wErrLast;

unsigned long lastTime; //Last update for PID Controller

float wheel_base = 100; // Space between Wheels mm
float wheel_dia = 65; //Wheel diminsion mm

double vVal, wVal;

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

void setup () 
{
  Serial.begin(9600);
  EEBlue.begin(9600);
  robot = RobotData();
  Serial.println("The Bluetooth gates are open.");
  Serial.println("Connect to HC-05 with 1234 as key");
  compass.init();
  setupArdumoto();
}

void loop () 
{
	char directionArray[3];
	compass.read();
	robot.azimuth = compass.getAzimuth();
	compass.getDirection(robot.directionArray, robot.azimuth);
  wCurr = robot.azimuth;

  bluetoothSerialization();
  updatePID();
  motorControls(robot.azimuth);
  delay(20);                                
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
          return false;
        }
        double x = doc["x"];
        double y = doc["y"];
        double phi = doc["phi"];
        vDest = sqrt(pow(x, 2) + pow(y, 2));
        wDest = phi * 180 / PI;
        robot.kp_v = doc["v_kp"];
        robot.kd_v = doc["v_kv"];
        robot.ki_v = doc["v_ki"];
        robot.kp_w = doc["w_kp"];
        robot.kd_w = doc["w_kv"];
        robot.ki_w = doc["w_ki"];
        newData = false;
    } 
}

void updatePID()
{
  //Controls for Velocity
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  double vErr = vDest-vCurr;
  vErrSum += (vErr * timeChange);
  double vErrDot = (vErr*vErrLast) / timeChange;
  vVal = robot.kp_v*(vErr) + robot.kd_v*(vErrDot) + robot.ki_v*(vErrSum);
  vErrLast = vErr;
  //Controls for Angular Momentum
  double wErr = wDest-wCurr;
  wErrSum += (wErr * timeChange);
  double wErrDot = (wErr*wErrLast) / timeChange;
  wVal = robot.kp_w*(wErr) + robot.kd_w*(wErrDot) + robot.ki_w*(wErrSum);
  wErrLast = wErr;
  lastTime = now;
}

void motorControls(int a)
{
  int vr_dest, vl_dest;

  if (a > 0) {
    vr_dest = (2*vVal+(wVal*wheel_base))/wheel_dia;
    vl_dest = (2*vVal-(wVal*wheel_base))/wheel_dia;
  }
  else
  {
    vr_dest = (2*vVal-(wVal*wheel_base))/wheel_dia;
    vl_dest = (2*vVal+(wVal*wheel_base))/wheel_dia;
  }
  
  if (vr_dest < 0)
  {
    driveArdumoto(MOTOR_A, REVERSE, vr_dest * -1);
  }
  else
  {
    driveArdumoto(MOTOR_A, FORWARD, vr_dest);
  }

   if (vl_dest < 0)
  {
    driveArdumoto(MOTOR_B, REVERSE, vl_dest * -1);
  }
  else
  {
    driveArdumoto(MOTOR_B, FORWARD, vl_dest);
  }
}

void driveArdumoto(byte motor, byte dir, byte spd)
{
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

void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
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