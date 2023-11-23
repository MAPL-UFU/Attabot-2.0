// Include ROS libraries
#define USE_USBCON // Required for Arduino Due
#include <ros.h> // ROS library for Arduino
#include <std_msgs/String.h> // Standard message types for ROS

// Include Attabot libraries
#include <HW006.h> // Library for the Attabot HW006 proximity sensor

// Define HW006 digital port
const int sensorPin = 3; // Digital pin to which the HW006 sensor is connected

//Instantiate HW006 object
HW006 hw006(sensorPin); // Create an instance of the HW006 sensor object

// Define the ROS node handler
ros::NodeHandle nh; // Create a ROS node handler object

// Define the ROS publishers
std_msgs::String hw006_msg; // Create a string message for the HW006 data
ros::Publisher hw006_pub("hw006_publisher", &hw006_msg); // Create a publisher for the HW006 data

// Define HW006 message variable
char almostCollisionEvent[24] = "Event: almost collision"; // Define the message to be published

void setup() {
  // Configure Baudrate
  nh.getHardware()->setBaud(57600); // Set the baud rate for the ROS node handler
  
  // Initialize the ROS node handler
  nh.initNode(); // Initialize the ROS node handler

  // Advertise the publishers
  nh.advertise(hw006_pub); // Advertise the HW006 data publisher
  nh.spinOnce(); // Process any incoming messages
}

void loop() {
  // Read the proximity sensor (HW006) and set the event message
  bool collision = hw006.checkCollision(); // Read the sensor and store the result in a boolean variable

  if(collision == true){
    /* If the sensor detects a collision the hw006 publisher will send an event message to Attabot node  */
    hw006_msg.data = almostCollisionEvent;
    hw006_pub.publish(&hw006_msg);
  }

  nh.spinOnce(); 
  delay(1000); // Wait for 1 second
}
