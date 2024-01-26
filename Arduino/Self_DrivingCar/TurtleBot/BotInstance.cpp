#include "BotInstance.h"

    BotInstance::BotInstance(int id, String name)
    {
      this->id = id;
      this->name = name;
    }

    void BotInstance::init()
    {
      this->compass.init();
    }

    void BotInstance::update(int time)
    {
      this->time = time;
      this->compass.read();
      this->azimuth = this->compass.getAzimuth();
    }

    String BotInstance::serializeData()
    {
      StaticJsonDocument<200> doc;
      doc["id"] = this->id;
      doc["name"] = this->name;
      doc["time"] = this->time;
      doc["u_sensor"] = this->distance;
      doc["azimuth"] = this->azimuth;
      String output;
      serializeJson(doc, output);
      return output;
    }