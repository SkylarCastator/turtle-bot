/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Code to manage the bluetooth communication from the robot
**/
#include <SoftwareSerial.h>
SoftwareSerial EEBlue(4,11);

void setupBluetooth(int baud_rate)
{
  EEBlue.begin(baud_rate);
}

void readBluetoothSerial()
{
  while(Serial.available() > 0)
  {
    EEBlue.write(Serial.read());
  }
}

void sendDiagnosticData(String botData)
{
  EEBlue.write(botData.c_str());
}
