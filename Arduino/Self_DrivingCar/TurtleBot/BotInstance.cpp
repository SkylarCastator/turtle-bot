#include "BotInstance.h"
    BotInstance::BotInstance(int id, String name)
    {
      this->id = id;
      this->name = name;
      this->compass.init();
    }

    void BotInstance::update(int time)
    {
      this->time = time;
      this->compass.read();
      this->distance = this->distSensor.getDistance();
      this->azimuth = this->compass.getAzimuth();

      if (this->distance >= 0 && this->distance <= 2)
      {
        this->controller.hitObject(this->time);
      }
      this->controller.updateMotors(this->time);
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