#include "MPU9250.h"
#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/Imu.h>

MPU9250 mpu;

// Create a structure to store the sensor data
struct MPUSensorData {
    float eulerX;
    float eulerY;
    float eulerZ;
    float quaternionW;
    float linearAccX;
    float linearAccY;
    float linearAccZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

MPUSensorData mpuSensorData; // Create an instance of the MPUSensorData structure to store sensor values

void setup() {
    // ... (rest of the setup code remains the same)
}

void loop() {
    nh.spinOnce();

    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_sensor_data();
            prev_ms = millis();
        }
    }

    // Store sensor data in the structure
    mpuSensorData.eulerX = mpu.getEulerX() - 178; // Adjust values as needed
    mpuSensorData.eulerY = mpu.getEulerY();
    mpuSensorData.eulerZ = mpu.getEulerZ() - 43; // Adjust values as needed
    mpuSensorData.quaternionW = mpu.getQuaternionW();
    mpuSensorData.linearAccX = mpu.getLinearAccX();
    mpuSensorData.linearAccY = mpu.getLinearAccY();
    mpuSensorData.linearAccZ = mpu.getLinearAccZ();
    mpuSensorData.gyroX = mpu.getGyroX();
    mpuSensorData.gyroY = mpu.getGyroY();
    mpuSensorData.gyroZ = mpu.getGyroZ();

    // Populate imu_msg fields from the structure
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = nh.now();
    imu_msg.orientation.x = mpuSensorData.eulerX;
    imu_msg.orientation.y = mpuSensorData.eulerY;
    imu_msg.orientation.z = mpuSensorData.eulerZ;
    imu_msg.orientation.w = mpuSensorData.quaternionW;
    imu_msg.linear_acceleration.x = mpuSensorData.linearAccX;
    imu_msg.linear_acceleration.y = mpuSensorData.linearAccY;
    imu_msg.linear_acceleration.z = mpuSensorData.linearAccZ;
    imu_msg.angular_velocity.x = mpuSensorData.gyroX;
    imu_msg.angular_velocity.y = mpuSensorData.gyroY;
    imu_msg.angular_velocity.z = mpuSensorData.gyroZ;

    imu_pub.publish(&imu_msg);
    nh.advertise(imu_pub);
}

void print_sensor_data() {
    Serial.println("Sensor Data:");
    Serial.print("Euler X: ");
    Serial.println(mpuSensorData.eulerX);
    Serial.print("Euler Y: ");
    Serial.println(mpuSensorData.eulerY);
    Serial.print("Euler Z: ");
    Serial.println(mpuSensorData.eulerZ);
    Serial.print("Quaternion W: ");
    Serial.println(mpuSensorData.quaternionW);
    Serial.print("Linear Acc X: ");
    Serial.println(mpuSensorData.linearAccX);
    Serial.print("Linear Acc Y: ");
    Serial.println(mpuSensorData.linearAccY);
    Serial.print("Linear Acc Z: ");
    Serial.println(mpuSensorData.linearAccZ);
    Serial.print("Gyro X: ");
    Serial.println(mpuSensorData.gyroX);
    Serial.print("Gyro Y: ");
    Serial.println(mpuSensorData.gyroY);
    Serial.print("Gyro Z: ");
    Serial.println(mpuSensorData.gyroZ);
    Serial.println("---------------");
}
