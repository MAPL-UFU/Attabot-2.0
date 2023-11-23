#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/LaserScan.h>

// Create an instance of the VL53L0X sensor
Adafruit_VL53L0X vl53l0x = Adafruit_VL53L0X();

// Create a ROS node handle
ros::NodeHandle nh;

// Create a LaserScan message for VL53L0X data
sensor_msgs::LaserScan vl53l0xMsg;
ros::Publisher vl53l0xPub("/vl53l0x_scan", &vl53l0xMsg);

// Define the number of readings desired
int numReadings = 25;

// Array to store distance readings
float distanceReadings[25];

// Variable to store the start time
unsigned long startTime;

// Define minimum and maximum angles and Pi value
float angleMin;
float angleMax;
const float Pi = 3.14159;

void setup()
{
  // Initialize the I2C communication
  Wire.begin();

  // Initialize the VL53L0X sensor
  if (!vl53l0x.begin()) {
    Serial.println(F("Error initializing the VL53L0X sensor."));
    while (1);
  }

  // Set the baud rate for ROS communication
  nh.getHardware()->setBaud(57600);

  // Initialize the ROS node
  nh.initNode();

  // Advertise the LaserScan topic
  nh.advertise(vl53l0xPub);
}

void loop()
{
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

  // Fill in LaserScan message fields
  vl53l0xMsg.header.frame_id = "vl53l0x_frame";
  vl53l0xMsg.angle_min = -(12.5 * Pi) / 180;
  vl53l0xMsg.angle_max = (12.5 * Pi) / 180;
  vl53l0xMsg.angle_increment = (1 * Pi) / 180;
  vl53l0xMsg.range_min = 0.05; // Minimum range value [m]
  vl53l0xMsg.range_max = 1.2;  // Maximum range value [m]

  vl53l0xMsg.scan_time = (millis() - startTime) / 1000.0;
  vl53l0xMsg.ranges_length = numReadings;
  vl53l0xMsg.ranges = distanceReadings;
  vl53l0xMsg.header.stamp = nh.now();

  // Publish the LaserScan message
  vl53l0xPub.publish(&vl53l0xMsg);

  // Advertise the LaserScan topic
  nh.advertise(vl53l0xPub);

  // Process ROS callbacks
  nh.spinOnce();
}

/* INSTRUCTIONS FOR USE */
/* 1. Run the ROS node: rosrun
 * 2. Start the serial communication: rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
 * 3. List available topics: rostopic list
 * 4. Echo the laser scan topic: rostopic echo laser_scan
 * 5. Use RViz to visualize data:
 *    - Set the global frame to 'laser'
 *    - Create a laser element
 *    - Use the topic name for visualization
 */
