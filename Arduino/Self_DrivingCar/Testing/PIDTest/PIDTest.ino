#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>
#include <ArduinoJson.h>
JsonDocument doc;

SoftwareSerial EEBlue(2,3); //RX and TX
QMC5883LCompass compass;

int kp_v, kd_v, ki_v;
int kp_w, kd_w, ki_w;

float vCurr = 0;
float vDest = 0;
double vErrSum, vErrLast;

float wCurr = 0;
float wDest = 0;
double wErrSum, wErrLast;

unsigned long lastTime;

float wheel_base = 100; // Space between Wheels mm
float wheel_dia = 65; //Wheel diminsion mm

double vVal, wVal;


void updatePID()
{
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  double vErr = vDest-vCurr;
  vErrSum += (vErr * timeChange);
  double vErrDot = (vErr*vErrLast) / timeChange;
  vVal = kp_v*(vErr) + kd_v*(vErrDot) + ki_v*(vErrSum);
  vErrLast = vErr;

  double wErr = wDest-wCurr;
  wErrSum += (wErr * timeChange);
  double wErrDot = (wErr*wErrLast) / timeChange;
  wVal = kp_w*(wErr) + kd_w*(wErrDot) + ki_w*(wErrSum);
  vErrLast = vErr;

  lastTime = now;
}




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
  Serial.println("The Bluetooth gates are open.");
  Serial.println("Connect to HC-05 with 1234 as key");
  compass.init();
  setupArdumoto();
}

boolean newData = false;
const byte numChars = 128;
char receivedChars[numChars];

void loop () 
{
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
        //Serial.println(receivedChars);
        DeserializationError error = deserializeJson(doc, receivedChars);
        if(error) {
          Serial.print("deserializeJson() returned ");
+         Serial.println(error.c_str());
          return false;
        }
        double x = doc["x"];
        double y = doc["y"];
        double phi = doc["phi"];
        vDest = 0;
        wDest = 0;
        kp_v = doc["v_kp"];
        kd_v = doc["v_kv"];
        ki_v = doc["v_ki"];
        kp_w = doc["w_kp"];
        kd_w = doc["w_kv"];
        ki_w = doc["w_ki"];
        newData = false;
    } 
  updatePID();
  int vr_dest = (2*vVal+(wVal*wheel_base))/wheel_dia;
  int vl_dest = (2*vVal-(wVal*wheel_base))/wheel_dia;
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

  int x, y, z, a, b;
	char directionArray[3];
	
	compass.read();
  
	x = compass.getX();
	y = compass.getY();
	z = compass.getZ();
	
	a = compass.getAzimuth();
	
	b = compass.getBearing(a);

	compass.getDirection(directionArray, a);
  
	/*Serial.print("X: ");
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
	Serial.print(directionArray[0]);
	Serial.print(directionArray[1]);
	Serial.print(directionArray[2]);

	Serial.println();*/
  delay(30);                                
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

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}