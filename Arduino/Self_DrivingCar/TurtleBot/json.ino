#include <ArduinoJson.h>

void serializeBotData(int time)
{
  StaticJsonDocument<200> doc;
  doc["id"] = id;
  doc["time"] = time;
  doc["u_sensor"] = u;
  doc["azimuth"] = a;
  JsonArray mag_raw = doc.createNestedArray("mag_raw");
  mag_raw.add(x);
  mag_raw.add(y);
  mag_raw.add(z);

  serializeJson(doc, Serial);
  Serial.println();
  serializeJsonPretty(doc, Serial);
}