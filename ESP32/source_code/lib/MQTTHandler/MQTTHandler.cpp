#include "MQTTHandler.h"

MQTTHandler::MQTTHandler(const char *mqttServer, int mqttPort)
    : mqttServer(mqttServer), mqttPort(mqttPort), client(espClient) {}

void MQTTHandler::handleReconnect()
{
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client"))
    {
      Serial.println("Connected to MQTT broker");
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Trying again in 5 seconds");
      delay(5000);
    }
  }
}

void MQTTHandler::connect()
{
  client.setServer(mqttServer, mqttPort);
}

void MQTTHandler::publishMessage(const char *topic,  const String &message)
{
  if (!client.connected())
  {
    handleReconnect();
  }
  client.publish(topic, message.c_str());
}

void MQTTHandler::loop()
{
  if (!client.connected())
  {
    handleReconnect();
  }
  client.loop();
}
