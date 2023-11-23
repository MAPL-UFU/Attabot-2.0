// Include necessary libraries and header files
#include "CollisionModule.h"  // Include the header file for the CollisionModule class
#include <Arduino.h>  // Include the Arduino standard library

// Constructor for the CollisionModule class, taking an array of sensor pins as input
CollisionModule::CollisionModule(int *sensorPins)
{
  _sensorPins = sensorPins;  // Store the sensor pins array in the class variable

  // Loop through each sensor pin
  for (int i = 0; i < 6; i++)
  {
    _sensorPins[i] = sensorPins[i];  // Store the individual sensor pin
    pinMode(_sensorPins[i], INPUT);   // Set the sensor pin mode to INPUT
  }
}

// Method to check for almost collision
void CollisionModule::checkCollision()
{
  // Loop through each sensor
  for (int i = 0; i < 6; i++)
  {
    _sensorStates[i] = digitalRead(_sensorPins[i]);  // Read the state of the sensor pins
  }

}

// Method to return movement instructions
int CollisionModule::getMovementInstructions()
{
  bool obstacleLeft = _sensorStates[0] == HIGH || _sensorStates[1] == HIGH;
  bool obstacleRight = _sensorStates[4] == HIGH || _sensorStates[5] == HIGH;
  bool obstacleCenter = _sensorStates[2] == HIGH || _sensorStates[3] == HIGH;

  if (obstacleLeft && !obstacleRight)
  {
    // Turn right
    return 4;
  }
  else if (obstacleRight && !obstacleLeft)
  {
    // Turn left
    return 3;
  }
  else if (obstacleCenter)
  {
    // Move backward
    return 2;
  }
  else
  {
    // Move forward
    return 1;
  }
}
