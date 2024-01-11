#include <SoftwareSerial.h>

SoftwareSerial EEBlue(2,3); //RX and TX
void setup()
{
  Serial.begin(9600);
  EEBlue.begin(9600);
  Serial.println("The Bluetooth gates are open.");
  Serial.println("Connect to HC-05 with 1234 as key");
}

void loop()
{
  if (EEBlue.available()){
    Serial.write(EEBlue.read());
  }

  if (Serial.available())
  {
    EEBlue.write(Serial.read());
  }
}