#include <Arduino.h>
#include "LDR.h"

// Constructor - initializes the LDR object 
LDR::LDR(int _pinSensor){
  pinSensor = _pinSensor;
  pinMode(pinSensor, INPUT);
}

// Sets the light intensity threshold for the LDR sensor
void LDR::setLightIntensity(int _lightIntensity){
  lightIntensity = _lightIntensity;
}

// Returns the light intensity threshold for the LDR sensor
int LDR::getLightIntensity(){
  return lightIntensity;
}

// Checks if the light intensity detected by the LDR sensor is greater than or equal to the set lightIntensity and returns true if detected
bool LDR::checkLight(){
  bool lightDetected;

  analogRead(pinSensor) <= lightIntensity ? lightDetected = true : lightDetected = false;
  
  return lightDetected;
}

