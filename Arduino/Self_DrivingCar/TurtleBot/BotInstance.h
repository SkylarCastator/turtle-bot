#include <Arduino.h>
#include <ArduinoJson.h>
#include <QMC5883LCompass.h>
#include "UltrasonicRangeSensor.h"
#include "Controller.h"
#include "Encoder.h"

#define ENCODER_LEFT_PIN 2
#define ENCODER_RIGHT_PIN 3
#define TRIGGER_PIN 12
#define ECHO_PIN 13 

class BotInstance{
  private:
    int id;
    String name;
    int time = 0;
    int distance = 0;
    int azimuth = 0;
    QMC5883LCompass compass;
    UltrasonicRangeSensor distSensor = UltrasonicRangeSensor(TRIGGER_PIN, ECHO_PIN);
    Controller controller = Controller();

  public:
    Encoder encoderLeft = Encoder(ENCODER_LEFT_PIN,20);
    Encoder encoderRight = Encoder(ENCODER_RIGHT_PIN,20);
    BotInstance(int id, String name);
    void update(int time);
    String serializeData();
    void updateMotors(unsigned long time);
    void hitObject (unsigned long time);
};