/*
  This code uses a proximity sensor (HW006) to detect the presence of an object. 
  If the sensor detects a presence, it turns on an LED
  and sends an event message via the serial monitor. 
 */

// Declare the pin for the LED and HW006 module
const int ledPin = 13;
const int sensorPin = 3;

// Declare the variable to store the HW006 state
int sensorState = LOW;

void setup() {
  Serial.begin(57600); // Initiate the serial communication
  
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output
  pinMode(sensorPin, INPUT); // Set the HW006 pin as an input
}

void loop() {
  sensorState = digitalRead(sensorPin);

  // If a presence is detected, turn on the LED and send an event message
  if (sensorState == HIGH){
    Serial.println("Event: presence detected"); // Send an event message
    digitalWrite(ledPin, HIGH); // Turn the LED on
  } else {
    digitalWrite(ledPin, LOW); // Turn the LED off
  }

  // Wait for one second before checking the sensor again
  delay(1000);
}
