#include <Arduino.h>
#include <SoftwareSerial.h>

class Bluetooth
{
  private:
    byte rx;
    byte tx;
    SoftwareSerial EEBlue;

  public:
    Bluetooth(byte rx, byte tx);
    void init(int baud_rate);
    void readBluetoothSerial();
    void sendDiagnosticData(String message);
};