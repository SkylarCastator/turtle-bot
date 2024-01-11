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

void sendBluetoothDistanceMessage(int distance)
{
  char str[] = "u ";
  char distVal[10];

  sprintf(distVal, "%d", distance);
  char add_new_line[] = "\n";
  strcat(distVal, add_new_line);
  strcat(str, distVal);

  EEBlue.write(str);
}