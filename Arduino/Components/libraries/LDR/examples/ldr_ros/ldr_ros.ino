/* 
PINOUT LDR MODULE
GND: GND
VCC: 5V
DO: Analog Port A0
*/

// Include ROS libraries
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>

// Include Attabot libraries
#include <LDR.h>

// Define the ROS node handler
ros::NodeHandle nh;

// Define the ROS publisher
std_msgs::String ldr_msg;
ros::Publisher ldr_pub("ldr_publisher", &ldr_msg);

// Define LDR message variable
char lightDetectedEvent[22] = "Event: light detected";

// Define LDR analog port
const int sensorPin = A0;

// Define light intensity
int lightIntensity = 200;

// Instantiate LDR object
LDR ldr(sensorPin);

void setup() {
  // Configure baudrate
  nh.getHardware()->setBaud(57600);
  
  // Initialize the ROS node handler
  nh.initNode();

  // Advertise the publisher
  nh.advertise(ldr_pub);
  nh.spinOnce();

  // Set the light intensity
  ldr.setLightIntensity(lightIntensity);
}

void loop() {
  // Check if light is detected by the LDR sensor
  bool ldrSensorState = ldr.checkLight();

  // If light is detected, publish an event message to the ROS network
  if (ldrSensorState == true) {
    ldr_msg.data = lightDetectedEvent;
    ldr_pub.publish(&ldr_msg);
  }

  // Wait for a second
  delay(1000);

  // Process any incoming ROS messages
  nh.spinOnce();
}
