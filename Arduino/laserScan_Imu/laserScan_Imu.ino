#include "MPU9250.h"
#include <VL53L0X.h>

#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

int intPin=4;
int intPin2=5;   // interrupt pin
MPU9250 mpu;
VL53L0X sensor;

ros::NodeHandle nh;

sensor_msgs::LaserScan lidar_msg;
ros::Publisher lidar_pub("/laser_scan", &lidar_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);


float ranges[25] = {0};
volatile int p = 0;
float angle_min;
float angle_max;
const float Pi = 3.14159;
bool take_measure = true;


void setup() {
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }
  attachInterrupt(digitalPinToInterrupt(intPin), getImu, RISING);
  //attachInterrupt(digitalPinToInterrupt(intPin2), getScanSensor, RISING);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(lidar_pub);

  sensor.init();
  sensor.setTimeout(500);
}

void loop() {

  getScanSensor();
  //getImu();
  
  lidar_pub.publish(&lidar_msg);
  nh.advertise(lidar_pub);
  imu_pub.publish(&imu_msg);
  nh.advertise(imu_pub);
  nh.spinOnce(); 
}

void getImu(){   

  if (mpu.update()) {
      volatile static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
          prev_ms = millis();
      }
  }
  
  imu_msg.header.frame_id = "imu";
  
  
  imu_msg.orientation.x = mpu.getQuaternionX();
  imu_msg.orientation.y = mpu.getQuaternionY();
  imu_msg.orientation.z = mpu.getQuaternionZ();
  imu_msg.orientation.w = mpu.getQuaternionW();  
  
  imu_msg.linear_acceleration.x = mpu.getLinearAccX();
  imu_msg.linear_acceleration.y = mpu.getLinearAccY();
  imu_msg.linear_acceleration.z = mpu.getLinearAccZ();
  //imu_msg.linear_acceleration_covariance[0] = -1;
  
  imu_msg.angular_velocity.x = mpu.getRoll();
  imu_msg.angular_velocity.y = mpu.getPitch();
  imu_msg.angular_velocity.z = mpu.getYaw();
  
  imu_msg.header.stamp = nh.now();
    
}

void getScanSensor(){
 
  int p = 0;
    
  lidar_msg.header.frame_id = "laser";
  lidar_msg.angle_min = -(12.5*Pi)/180;
  lidar_msg.angle_max = (12.5*Pi)/180;
  lidar_msg.angle_increment = (1*Pi)/180;
  lidar_msg.range_min = 0.05;          // minimum range value [m]
  lidar_msg.range_max = 1.2;           // maximum range value [m]

  volatile unsigned long scan_start = millis();
    
  while(p <= 24) {
    if(take_measure == true) {
    ranges[p] = (volatile float)sensor.readRangeSingleMillimeters()/1000.0;
    ++p;
    }
  }
  
  lidar_msg.scan_time = (millis()-scan_start)/1000.0;
//  lidar_msg.time_increment = 0.5;
  lidar_msg.ranges_length = p;
  lidar_msg.ranges = ranges;
  lidar_msg.header.stamp = nh.now();
  

}
