/*
biblioteca: https://github.com/hideakitai/MPU9250
doc: http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html
datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
example: https://github.com/dheera/ros-imu-bno055/blob/master/src/bno055_i2c_node.cpp
requer: sudo apt-get install ros-noetic-imu-tools
*/

#include "MPU9250.h"

#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/Imu.h>

MPU9250 mpu;

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);


void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(imu_pub);
}

void loop() {

    nh.spinOnce();  
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = nh.now();
    
    imu_msg.orientation.x = mpu.getEulerX()-178;
    imu_msg.orientation.y = mpu.getEulerY();
    imu_msg.orientation.z = mpu.getEulerZ()-43;
    imu_msg.orientation.w = mpu.getQuaternionW();
    
    imu_msg.linear_acceleration.x = mpu.getLinearAccX();
    imu_msg.linear_acceleration.y = mpu.getLinearAccY();
    imu_msg.linear_acceleration.z = mpu.getLinearAccZ();

    imu_msg.angular_velocity.x = mpu.getGyroX();
    imu_msg.angular_velocity.y = mpu.getGyroY();
    imu_msg.angular_velocity.z = mpu.getGyroZ();
    
    imu_pub.publish(&imu_msg);
    nh.advertise(imu_pub);
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 0);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 0);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 0);
}
