/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Turtle bot is a simple autonomous vehicle that drives around until it reaches an obsticle and then rotates and drives a new direction.
**/

#import <Wire.h>

const int BAUD_RATE = 9600;

unsigned long currentMillis;

void setupCompass();
void setupEncoders();
void setupBluetooth(int baudRate);
void setupDistanceSensor();
void setupMotors();

int updateDistanceSensor();
void readCompass();
void readBluetoothSerial();
void sendBluetoothDistanceMessage(int distance);
void updateMotors(unsigned long time);
void hitObject (unsigned long time);

void setup() {
  Wire.begin();
  //randomSeed(analogRead(3));
  Serial.begin(BAUD_RATE);
  setupBluetooth(BAUD_RATE);
  setupEncoders();
  setupCompass();
  setupDistanceSensor();
  setupMotors();
}

void loop() {
  currentMillis = millis();

  int distance = updateDistanceSensor();
  readBluetoothSerial();
  readCompass();
  sendBluetoothDistanceMessage(distance);

  if (distance >= 0 && distance <= 2)
  {
    hitObject(currentMillis);
  }
  updateMotors(currentMillis);
}
