#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

class MQTTHandler
{
public:
  MQTTHandler(const char *mqttServer, int mqttPort);
  void connect();
  void publishMessage(const char *topic, const String &message);
  void loop();

private:
  const char *mqttServer;
  int mqttPort;

  WiFiClient espClient;
  PubSubClient client;

  void handleReconnect();
};

#endif
