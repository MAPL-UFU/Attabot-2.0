// LDR.h

#ifndef LDR_H_INCLUDED
#define LDR_H_INCLUDED

#include <Arduino.h>

class LDR {
  private:
    int pinSensor; // Pin number for the LDR sensor
    int lightIntensity; // Threshold for the light intensity
  public:
    LDR(int _pinSensor); // Constructor 
    bool checkLight(); // Checks if theres a light event
    void setLightIntensity(int _lightIntensity); // Sets the light intensity threshold for the LDR sensor change it's value
    int getLightIntensity(); // Returns the light intensity of the LDR sensor
};

#endif // LDR_H_INCLUDED

