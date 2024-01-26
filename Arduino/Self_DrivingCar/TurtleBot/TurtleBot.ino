/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Turtle bot is a simple autonomous vehicle that drives around until it reaches an obsticle and then rotates and drives a new direction.
**/
#import <Wire.h>
#include "BotInstance.h"
#include "Bluetooth.h"

#define BLUETOOTH_RX 4
#define BLUETOOTH_TX 11

const int BAUD_RATE = 9600;
unsigned long currentMillis;
Encoder * Encoder::instances[2] = {NULL, NULL};

BotInstance botInstance("001", "Turtleturtle");
Bluetooth bluetoothDevice(BLUETOOTH_RX, BLUETOOTH_TX);

void setup() {
  Wire.begin();
  Serial.begin(BAUD_RATE);
  bluetoothDevice.init(BAUD_RATE);
}

void loop() {
  currentMillis = millis();
  bluetoothDevice.readBluetoothSerial();
  botInstance.update(currentMillis);
  bluetoothDevice.sendDiagnosticData(botInstance.serializeData());
}
