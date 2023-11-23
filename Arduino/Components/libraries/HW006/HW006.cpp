#include "HW006.h"
#include <Arduino.h>

// Constructor
HW006::HW006(int _pinSensor){
  pinSensor = _pinSensor;
  preState = 0;
  currentState = 0;
  pinMode(pinSensor, INPUT);
}

// Method to check for almost collision
bool HW006::checkCollision(){
  /* 
  * Returns True if an almost collision is detected
  * Checks the state of the proximity sensor
  * If the current state is HIGH and different from previous state, then collision detected
  */
  bool collisionDetected = false;
  
  currentState = digitalRead(pinSensor);
  
  if(currentState == HIGH && currentState != preState){
    collisionDetected = true;
    preState = currentState;
  } else if (currentState == LOW){
    collisionDetected = false;
    preState = currentState;
  }
  
  return collisionDetected;
}
