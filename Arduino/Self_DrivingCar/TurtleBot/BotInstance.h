#include <Arduino.h>
#include <ArduinoJson.h>
#include <QMC5883LCompass.h>

class BotInstance{
  private:
    int id;
    String name;
    int time = 0;
    int distance = 0;
    int azimuth = 0;
    QMC5883LCompass compass;

  public:
    BotInstance(int id, String name);
    void init();
    void update(int time);
    String serializeData();
};