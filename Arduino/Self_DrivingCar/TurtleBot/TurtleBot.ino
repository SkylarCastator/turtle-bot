/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Turtle bot is a simple autonomous vehicle that drives around until it reaches an obsticle and then rotates and drives a new direction.
**/

#import <Wire.h>
#include "BotInstance.h"
#include "UltrasonicRangeSensor.h"

#define TRIGGER_PIN 12
#define ECHO_PIN 13 

const int BAUD_RATE = 9600;

unsigned long currentMillis;

BotInstance botInstance("001", "Turtleturtle");
UltrasonicRangeSensor distSensor(TRIGGER_PIN, ECHO_PIN);

void setupCompass();
void setupEncoders();
void setupBluetooth(int baudRate);
void setupMotors();

void readCompass();
void readBluetoothSerial();
void updateMotors(unsigned long time);
void hitObject (unsigned long time);

void setup() {
  Wire.begin();
  //randomSeed(analogRead(3));
  Serial.begin(BAUD_RATE);
  setupBluetooth(BAUD_RATE);
  setupEncoders();
  setupCompass();
  setupMotors();
}

void loop() {
  currentMillis = millis();
  Serial.print("t ");
  Serial.print(currentMillis);
  Serial.print("\n");

  int distance = distSensor.getDistance();
  readBluetoothSerial();
  readCompass();
  
  sendDiagnosticData(botInstance.serializeData());

  if (distance >= 0 && distance <= 2)
  {
    hitObject(currentMillis);
  }
  updateMotors(currentMillis);
}
