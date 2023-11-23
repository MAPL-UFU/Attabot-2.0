#include <Arduino.h>
#include "DataHandler.h"
#include "WiFiHandler.h"
#include "MQTTHandler.h"

const char *ssid = "your_wifi_ssid";         // example: "jose-wifi"
const char *password = "your_wifi_password"; // example: "123456"
const char *mqttServer = "192.168.0.11";     // example: "183.776.0.12"
const int mqttPort = 1883;

const char *leftMotorTopic = "attabot/left_motor";
const char *rightMotorTopic = "attabot/right_motor";
const char *collisionTopic = "attabot/collision";
const char *accelGyroTopic = "attabot/accel_gyro";
const char *distanceTopic = "attabot/distance";
const char *ldrTopic = "attabot/ldr";
const char *messagesTopic = "attabot/messages";

WiFiHandler wifiHandler(ssid, password);
MQTTHandler mqttHandler(mqttServer, mqttPort);

int robotId;
bool collisionStates[6];
String leftMotor;
String rightMotor;
bool lightSensor;
float distanceReadings[25];
AccelerometerData accelerometer;

void setup()
{
  Serial.begin(57600);
  wifiHandler.connect();
  mqttHandler.connect();
}

void loop()
{
  if (Serial.available() > 0)
  {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    // Parse the message using the DataHandler class
    DataHandler::parseMessage(message, robotId, collisionStates, leftMotor, rightMotor, lightSensor, distanceReadings, accelerometer);

    mqttHandler.publishMessage(messagesTopic, String(robotId));
    mqttHandler.publishMessage(leftMotorTopic, leftMotor);
    mqttHandler.publishMessage(rightMotorTopic, rightMotor);
    mqttHandler.publishMessage(collisionTopic, DataHandler::boolArrayToString(collisionStates, 6));
    mqttHandler.publishMessage(accelGyroTopic, accelerometer.getAccelerationInfoString());
    mqttHandler.publishMessage(distanceTopic, DataHandler::floatArrayToString(distanceReadings, 25, 2));
    mqttHandler.publishMessage(ldrTopic, String(lightSensor));

    mqttHandler.loop();
  }
  delay(2000); // Adjust the delay as needed
}