#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include <Arduino.h>

// Define the struct for accelerometer data
struct AccelerometerData
{
  float eulerX;
  float eulerY;
  float eulerZ;
  float linearAccX;
  float linearAccY;
  float linearAccZ;
  float gyroX;
  float gyroY;
  float gyroZ;

  String getAccelerationInfoString();
};

class DataHandler
{
public:
  // Function prototypes
  static void parseMessage(String message, int &robotId, bool collisionStates[], String &leftMotor, String &rightMotor, bool &lightSensor, float distanceReadings[], AccelerometerData &accelerometer);
  static String floatArrayToString(float values[], int size, int precision = 2);
  static String boolArrayToString(bool values[], int size);

private:
  // Private helper functions
  static void extractCollisionStates(String data, bool collisionStates[]);
  static void extractDistanceReadings(String data, float distanceReadings[]);
  static void extractAccelerometerData(String data, AccelerometerData &accelerometer);
};

#endif // DATA_HANDLER_H
