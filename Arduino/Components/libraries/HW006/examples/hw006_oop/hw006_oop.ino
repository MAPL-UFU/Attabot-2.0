// Include Attabot libraries
#include <HW006.h>

// Define HW006 digital port
const int sensorPin = 3;

// Instantiate HW006 object
HW006 hw006(sensorPin);

void setup() {
  // Configure Baudrate
  Serial.begin(57600);

  // Print a message to the serial monitor
  Serial.println("Program started");
}

void loop() {
  // Read the proximity sensor (HW006) and set the event message
  bool collision = hw006.checkCollision();

  // If a collision is detected, print a message to the serial monitor
  if(collision == true){
    Serial.println("Event: almost collision detected");
  }

  // Wait for one second before checking the sensor again
  delay(1000);
}
