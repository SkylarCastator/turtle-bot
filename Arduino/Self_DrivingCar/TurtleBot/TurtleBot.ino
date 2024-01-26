/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Turtle bot is a simple autonomous vehicle that drives around until it reaches an obsticle and then rotates and drives a new direction.
**/

#import <Wire.h>
#include "BotInstance.h"
#include "Bluetooth.h"
#include "UltrasonicRangeSensor.h"

#define ENCODER_LEFT_PIN 2
#define ENCODER_RIGHT_PIN 3
#define BLUETOOTH_RX 4
#define BLUETOOTH_TX 11
#define TRIGGER_PIN 12
#define ECHO_PIN 13 

const int BAUD_RATE = 9600;

unsigned long currentMillis;

BotInstance botInstance("001", "Turtleturtle");
UltrasonicRangeSensor distSensor(TRIGGER_PIN, ECHO_PIN);
Bluetooth bluetoothDevice(BLUETOOTH_RX, BLUETOOTH_TX);

void setupMotors();
void updateMotors(unsigned long time);
void hitObject (unsigned long time);

void setup() {
  Wire.begin();
  Serial.begin(BAUD_RATE);
  bluetoothDevice.init(BAUD_RATE);
  setupMotors();
}

void loop() {
  currentMillis = millis();

  int distance = distSensor.getDistance();

  bluetoothDevice.readBluetoothSerial();
  botInstance.update(currentMillis);
  bluetoothDevice.sendDiagnosticData(botInstance.serializeData());

  if (distance >= 0 && distance <= 2)
  {
    hitObject(currentMillis);
  }
  updateMotors(currentMillis);
}
