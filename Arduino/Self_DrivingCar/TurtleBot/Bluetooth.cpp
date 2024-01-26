#include "Bluetooth.h"

Bluetooth::Bluetooth(byte rx, byte tx)
: EEBlue(rx, tx)
{
  this->rx = rx;
  this->tx = tx;
}

void Bluetooth::init(int baud_rate)
{
  this->EEBlue.begin(baud_rate);
}

void Bluetooth::readBluetoothSerial()
{
  while(Serial.available() > 0)
  {
    this->EEBlue.write(Serial.read());
  }
}

void Bluetooth::sendDiagnosticData(String botData)
{
  this->EEBlue.write(botData.c_str());
}
