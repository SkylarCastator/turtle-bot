#include <Arduino.h>
#include <ArduinoJson.h>
#include <QMC5883LCompass.h>
#include "Encoder.h"

class BotInstance{
  private:
    int id;
    String name;
    int time = 0;
    int distance = 0;
    int azimuth = 0;
    QMC5883LCompass compass;
    Encoder encoderLeft;
    Encoder encoderRight;

  public:
    BotInstance(int id, String name);
    void update(int time);
    String serializeData();
};