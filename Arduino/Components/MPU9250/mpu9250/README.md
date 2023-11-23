# MPU9250 Arduino ROS
This tutorial guides you through the process of using an Arduino board with an MPU9250 sensor to publish Inertial Measurement Unit (IMU) data to a ROS (Robot Operating System) topic.

## Prerequisites
Before starting, ensure you have the following:

An Arduino board with the required hardware components (MPU9250 sensor, wires, etc.).
ROS (Robot Operating System) installed on your Linux machine.
Setup
Arduino Code: Upload the provided Arduino code to your Arduino board. This code interfaces with the MPU9250 sensor and publishes IMU data to a ROS topic. Ensure the following connections:

MPU9250 Sensor PINOUT:

VCC: 3.3V
GND: GND
SDA: A4
SCL: A5
ROS Core: Start the ROS core by running the following command in your terminal:

```bash
roscore
```
ROS Serial Node: Run the ROS serial node to establish communication between your Arduino and ROS. Replace /dev/ttyACM0 with the correct port if needed, and set the baud rate to match your Arduino code (default is 57600):

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```

List ROS Topics: To verify that the imu_publisher topic is being published, run:

```
rostopic list
```
You should see imu_publisher in the list of topics.

View IMU Data: To view the IMU data published by the Arduino, use the following command:

```bash
rostopic echo imu_publisher
```
This will display messages published to the imu_publisher topic, containing orientation, linear acceleration, and angular velocity data.

## Troubleshooting
If you encounter issues, make sure that your Arduino board is properly connected to your computer, and the Arduino code is successfully uploaded.
Double-check the serial port (/dev/ttyACM0 in the example) and baud rate settings.
Ensure that ROS is correctly installed and the core (roscore) is running.

## Conclusion
This explains how to use an Arduino with an MPU9250 sensor to publish IMU data to a ROS topic. To visualize and work with the IMU data, it is recommended to install the ros-noetic-imu-tools package. You can install it using the following command:

```bash
sudo apt-get install ros-noetic-imu-tools
After installation, you can use tools like rqt_plot from imu_tools to visualize and analyze the IMU data.
```

Now, you can integrate this functionality into your robotics projects or explore further ROS capabilities.
