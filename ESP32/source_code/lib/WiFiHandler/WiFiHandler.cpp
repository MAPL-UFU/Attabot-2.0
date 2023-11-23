#include "WiFiHandler.h"

WiFiHandler::WiFiHandler(const char *ssid, const char *password)
    : ssid(ssid), password(password) {}

void WiFiHandler::connect()
{
  delay(10);
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to WiFi");
}
