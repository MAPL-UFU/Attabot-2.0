// Include Attabot libraries
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <CollisionModule.h>
#include "N20.h"
#include <LDR.h>
#include "MPU9250.h"

int robotId = 0;
// Define constants for array sizes
const int COLLISION_STATES_SIZE = 6;
const int DISTANCE_READINGS_SIZE = 25;

enum CurrentDirection
{
  BACKWARD, // Move backward: 0
  FORWARD,  // Move forward: 1
  LEFT,  // Turn left: 2
  RIGHT, // Turn right: 3
};


MPU9250 mpu;

// Create a structure to store the sensor data
struct MPUSensorData
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
};

// Define Motors digital ports (PWM)
int _inPin1 = 10;
int _inPin2 = 9;
int _pinEncoderC1 = 3;
int _pinEncoderC2 = 4;
int _deviceID = 1;

int _inPin21 = 12;
int _inPin22 = 11;
int _pinEncoderC21 = 5;
int _pinEncoderC22 = 6;
int _deviceID2 = 2;

N20 rightMotor(_inPin1, _inPin2, _pinEncoderC1, _pinEncoderC2, _deviceID);
N20 leftMotor(_inPin21, _inPin22, _pinEncoderC21, _pinEncoderC22, _deviceID2);

int leftMotorDirection;
int rightMotorDirection;

void rightISR();
void leftISR();

CurrentDirection currentDirection = FORWARD;

// Define CollisionModule digital ports
int sensorPins[COLLISION_STATES_SIZE] = {2, 3, 4, 5, 6, 7};

// Define LDR analog port
const int ldrPin = A0;

// Define light intensity
int lightIntensity = 200;

bool ldrSensorState;

// Instantiate CollisionModule object
CollisionModule collisionModule(sensorPins);

// Instantiate LDR object
LDR ldr(ldrPin);

// Instantiate MPU9250 object
MPUSensorData mpuSensorData;

#define EULER_X_OFFSET 187;
#define EULER_Z_OFFSET 43;

void getMPUData();

// Create an instance of the VL53L0X sensor
Adafruit_VL53L0X vl53l0x = Adafruit_VL53L0X();

// Define the number of readings desired
int numReadings = DISTANCE_READINGS_SIZE;

// Array to store distance readings
float distanceReadings[DISTANCE_READINGS_SIZE];

// Variable to store the start time
unsigned long startTime;

// Define minimum and maximum angles and Pi value
float angleMin;
float angleMax;
const float Pi = 3.14159;

void getDistanceData();

// Sets the instructions for robots direction
void handleDirection(CurrentDirection currentDirection)
{
  Serial.print("Current Direction: ");

  switch (currentDirection) {
    case FORWARD:
      leftMotor.setDirection(currentDirection, 200);
      rightMotor.setDirection(currentDirection, 200);
      leftMotorDirection = currentDirection;
      rightMotorDirection = currentDirection;
      break;

    case BACKWARD:
      leftMotor.setDirection(currentDirection, 200);
      rightMotor.setDirection(currentDirection, 200);
      leftMotorDirection = currentDirection;
      rightMotorDirection = currentDirection;
      break;

    case LEFT:
      leftMotor.setDirection(1, 200);
      rightMotor.setDirection(0, 200);
      leftMotorDirection = currentDirection;
      rightMotorDirection = currentDirection;
      break;

    case RIGHT:
      leftMotor.setDirection(0, 200);
      rightMotor.setDirection(1, 200);
      leftMotorDirection = currentDirection;
      rightMotorDirection = currentDirection;
      break;
  }
}

// Sets the appropriate Enum for directions
CurrentDirection intToEnum(int currentDirection) {
  switch (currentDirection){
    case 1:
      return FORWARD;
    case 2:
      return BACKWARD;
    case 3:
      return LEFT;
    case 4:
      return RIGHT;
    default:
      return FORWARD;
  }
}

void setup()
{
  // Configure Baudrate
  Serial.begin(57600);

  // Motors Initial Setup
  attachInterrupt(digitalPinToInterrupt(_pinEncoderC1), rightISR, RISING);
  attachInterrupt(digitalPinToInterrupt(_pinEncoderC21), leftISR, RISING);

  rightMotor.setPID(1.0, 0.025, 0);
  rightMotor.updateTargetPosition(900);

  rightMotor.setPID(1.0, 0.025, 0);
  rightMotor.updateTargetPosition(600);

  // Set the light intensity
  ldr.setLightIntensity(lightIntensity);

  // MPU config
  if (!mpu.setup(0x68))
  { 
    Serial.println("MPU connection failed.");
}

// Initialize the I2C communication
Wire.begin();

// Initialize the VL53L0X sensor
if (!vl53l0x.begin())
{
  Serial.println("Error initializing the VL53L0X sensor.");
}
}

void loop() {
  // Update motors info
  rightMotor.updateInfo();
  leftMotor.updateInfo();

  // Check collisions and set motor directions
  collisionModule.checkCollision();
  int intDirection = collisionModule.getMovementInstructions();
  currentDirection = intToEnum(intDirection);
  handleDirection(currentDirection);

  // Check if light is detected by the LDR sensor
  bool ldrSensorState = ldr.checkLight();
  getMPUData();
  getDistanceData();
}

void rightISR()
{
  rightMotor.readEncoder();
}

void leftISR()
{
  leftMotor.readEncoder();
}

void getMPUData(){
  // Store sensor data in the structure
  mpuSensorData.eulerX = mpu.getEulerX() - EULER_X_OFFSET;
  mpuSensorData.eulerY = mpu.getEulerY();
  mpuSensorData.eulerZ = mpu.getEulerZ() - EULER_Z_OFFSET;
  mpuSensorData.quaternionW = mpu.getQuaternionW();
  mpuSensorData.linearAccX = mpu.getLinearAccX();
  mpuSensorData.linearAccY = mpu.getLinearAccY();
  mpuSensorData.linearAccZ = mpu.getLinearAccZ();
  mpuSensorData.gyroX = mpu.getGyroX();
  mpuSensorData.gyroY = mpu.getGyroY();
  mpuSensorData.gyroZ = mpu.getGyroZ();
}

void getDistanceData(){
  unsigned long scanStart = millis();

  // Record the start time
  startTime = millis();
  int i = 0;

  while (i < numReadings)
  {
    VL53L0X_RangingMeasurementData_t measure;
    vl53l0x.rangingTest(&measure, false);

    // Check if the measurement is valid
    if (measure.RangeStatus != 4)
    {
      // Store distance in meters
      distanceReadings[i] = static_cast<float>(measure.RangeMilliMeter) / 1000.0;
      i++;
    }
  }
}

// Function to generate collision states string
String generateCollisionStatesMessage()
{
  String message = collisionModule.getMessage();
  return message;
}

// Function to generate motor values string
String generateMotorValuesMessage()
{
  String leftMotorStr = handleMotorValues(leftMotorDirection);
  String rightMotorDirection = handleMotorValues(rightMotorDirection);

  String message = "Motor Values: " + leftMotorStr + ", " + rightMotorDirection;
  return message;
}

String handleMotorValues(int direction){
  if (direction == 0){
    return "backward";
  } else if (direction == 1){
    return "forward";
  } else if (direction == 2){
    return "left";
  } else {
    return "right";
  }
}

// Function to generate distance readings string
String generateDistanceReadingsMessage()
{
  String distanceReadingsStr = "Distance Readings: ";

  // Loop through distance readings array and append to the string
  for (int i = 0; i < DISTANCE_READINGS_SIZE; i++)
  {
    distanceReadingsStr += String(distanceReadings[i]);
    if (i < DISTANCE_READINGS_SIZE - 1)
    {
      distanceReadingsStr += ", ";
    }
  }

  return distanceReadingsStr;
}

String generateAccelerometerMessage(){
  return "Accelerometer: { Euler: { X: " + String(MPUSensorData.eulerX) +
         ", Y: " + String(MPUSensorData.eulerY) +
         ", Z: " + String(MPUSensorData.eulerZ) +
         " }, LinearAcc: { X: " + String(MPUSensorData.linearAccX) +
         ", Y: " + String(MPUSensorData.linearAccY) +
         ", Z: " + String(MPUSensorData.linearAccZ) +
         " }, Gyro: { X: " + String(MPUSensorData.gyroX) +
         ", Y: " + String(MPUSensorData.gyroY) +
         ", Z: " + String(MPUSensorData.gyroZ) +
         " } }";

}

void generateMessage(){
  // Function to update data and generate a formatted message
    String data = "< Robot ID: " + String(robotId);

    // data += ", " + generateCollisionStatesMessage();
    data += ", " + generateMotorValuesMessage();
    data += ", Light Sensor: " + ldrSensorState;
    data += ", " + generateDistanceReadings();
    data += ", " + generateAccelerometerData();
    data += ">";

    return data;
}
