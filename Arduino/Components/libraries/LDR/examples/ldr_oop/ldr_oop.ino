/* 
PINOUT LDR MODULE
GND: GND
VCC: 5V
DO: Analog Port A0
*/

// Include Attabot libraries
#include <LDR.h>

// Define LDR analog port
const int sensorPin = A0;

// Define light intensity
int lightIntensity = 200;

// Instantiate LDR object
LDR ldr(sensorPin);

void setup() {
  // Configure baudrate
  Serial.begin(57600);
  
  // Set the light intensity
  ldr.setLightIntensity(lightIntensity);
}

void loop() {
  // Check if light is detected by the LDR sensor
  bool ldrSensorState = ldr.checkLight();
  
  if (ldrSensorState == true) {
    Serial.println("Event: light detected");
  }

  // Wait for a second
  delay(1000);

}
