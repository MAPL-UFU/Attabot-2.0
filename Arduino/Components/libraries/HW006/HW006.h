#ifndef HW006_H_INCLUDED
#define HW006_H_INCLUDED

#include <Arduino.h>

class HW006 {
  private:
    int pinSensor;
    int preState;
    int currentState;
  public:
    // Constructor
    HW006(int _pinSensor);

    // Method to check for almost collision
    bool checkCollision();
};

#endif // HW006_H_INCLUDED
