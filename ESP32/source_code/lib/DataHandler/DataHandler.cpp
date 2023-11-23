#include "DataHandler.h"

void DataHandler::parseMessage(String message, int &robotId, bool collisionStates[], String &leftMotor, String &rightMotor, bool &lightSensor, float distanceReadings[], AccelerometerData &accelerometer)
{
  // Find and extract Robot ID
  int index = message.indexOf("Robot ID: ");
  if (index != -1)
  {
    robotId = message.substring(index + 10, message.indexOf(",", index)).toInt();
  }

  // Find and extract Collision States
  index = message.indexOf("Collision States: ");
  if (index != -1)
  {
    String collisionStatesStr = message.substring(index + 18, message.indexOf(", Motor Values:"));
    extractCollisionStates(collisionStatesStr, collisionStates);
  }

  // Find and extract Motor Values
  int index = message.indexOf("Motor Values: ");
  if (index != -1)
  {
    String motorValues = message.substring(index + 14, message.indexOf(", Light Sensor:"));

    // Assuming motor values are comma-separated
    int commaIndex = motorValues.indexOf(',');
    if (commaIndex != -1)
    {
      leftMotor = motorValues.substring(0, commaIndex).toFloat();
      rightMotor = motorValues.substring(commaIndex + 1).toFloat();
    }
    else
    {
      // If there's only one motor value, assign it to both left and right motors
      leftMotor = rightMotor = motorValues.toFloat();
    }
  }

  // Find and extract Light Sensor
  index = message.indexOf("Light Sensor: ");
  if (index != -1)
  {
    lightSensor = message.substring(index + 14, message.indexOf(", Distance Readings:")) == "true";
  }

  // Find and extract Distance Readings
  index = message.indexOf("Distance Readings: ");
  if (index != -1)
  {
    String distanceReadingsStr = message.substring(index + 20, message.indexOf(", Accelerometer:"));
    extractDistanceReadings(distanceReadingsStr, distanceReadings);
  }

  // Find and extract Accelerometer data
  index = message.indexOf("Accelerometer: {");
  if (index != -1)
  {
    String accelDataStr = message.substring(index + 16, message.length() - 3);
    extractAccelerometerData(accelDataStr, accelerometer);
  }
}

void DataHandler::extractCollisionStates(String data, bool collisionStates[])
{
  for (int i = 0; i < 6; i++)
  {
    collisionStates[i] = data.charAt(2 * i) == '1';
  }
}

void DataHandler::extractDistanceReadings(String data, float distanceReadings[])
{
  for (int i = 0; i < 25; i++)
  {
    distanceReadings[i] = data.substring(i * 6, i * 6 + 5).toFloat();
  }
}

void DataHandler::extractAccelerometerData(String data, AccelerometerData &accelerometer)
{
  // Extract Euler angles
  int index = data.indexOf("Euler: {");
  if (index != -1)
  {
    data = data.substring(index + 8, data.indexOf("},"));
    accelerometer.eulerX = data.substring(4, data.indexOf(", Y:")).toFloat();
    accelerometer.eulerY = data.substring(8 + data.indexOf(", Y:"), data.indexOf(", Z:")).toFloat();
    accelerometer.eulerZ = data.substring(8 + data.indexOf(", Z:"), data.length()).toFloat();
  }

  // Extract Linear Acceleration
  index = data.indexOf("LinearAcc: {");
  if (index != -1)
  {
    data = data.substring(index + 12, data.indexOf("},"));
    accelerometer.linearAccX = data.substring(6, data.indexOf(", Y:")).toFloat();
    accelerometer.linearAccY = data.substring(6 + data.indexOf(", Y:"), data.indexOf(", Z:")).toFloat();
    accelerometer.linearAccZ = data.substring(6 + data.indexOf(", Z:"), data.length()).toFloat();
  }

  // Extract Gyro Values
  index = data.indexOf("Gyro: {");
  if (index != -1)
  {
    data = data.substring(index + 7, data.length() - 1);
    accelerometer.gyroX = data.substring(4, data.indexOf(", Y:")).toFloat();
    accelerometer.gyroY = data.substring(4 + data.indexOf(", Y:"), data.indexOf(", Z:")).toFloat();
    accelerometer.gyroZ = data.substring(4 + data.indexOf(", Z:"), data.length()).toFloat();
  }
}

String DataHandler::floatArrayToString(float values[], int size, int precision)
{
  String result = "";
  for (int i = 0; i < size; i++)
  {
    result += String(values[i], precision);
    if (i < size - 1)
    {
      result += ", ";
    }
  }
  return result;
}

String DataHandler::boolArrayToString(bool values[], int size)
{
  String result = "";
  for (int i = 0; i < size; i++)
  {
    result += values[i] ? "true" : "false";
    if (i < size - 1)
    {
      result += ", ";
    }
  }
  return result;
}

String AccelerometerData::getAccelerationInfoString()
{
  String result = "Euler X: " + String(eulerX) +
                  " Euler Y: " + String(eulerY) +
                  " Euler Z: " + String(eulerZ) +
                  " Linear Acc X: " + String(linearAccX) +
                  " Linear Acc Y: " + String(linearAccY) +
                  " Linear Acc Z: " + String(linearAccZ) +
                  " Gyro X: " + String(gyroX) +
                  " Gyro Y: " + String(gyroY) +
                  " Gyro Z: " + String(gyroZ);

  return result;
}
