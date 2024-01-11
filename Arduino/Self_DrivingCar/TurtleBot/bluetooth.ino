#include <SoftwareSerial.h>
SoftwareSerial EEBlue(4,11);

void Setup_Serial(int baud_rate)
{
  Serial.begin(baud_rate);
  EEBlue.begin(baud_rate);
}

void Read_From_Serial(){
  while(Serial.available() > 0)
  {
    EEBlue.write(Serial.read());
  }
}