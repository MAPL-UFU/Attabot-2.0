# Arduino ROS HW006 Module (TCR5000) Tutorial

This tutorial explains how to use an Arduino board with an HW006 Module (TCR5000 - Infrared Sensor) to publish close objects detection events to a ROS (Robot Operating System) topic.

## Prerequisites

Before getting started, make sure you have the following:

- An Arduino board with the required hardware components (TCR5000 module, wires, etc.).
- ROS (Robot Operating System) installed on your Linux machine.

## Setup

1. **Arduino Code**: Upload the provided Arduino code to your Arduino board. This code interfaces with the HW006 module and publishes almost collision events to a ROS topic. Ensure the following connections:

    **HW006 Module PINOUT**:

    - GND: GND
    - VCC: 5V
    - OUT: Digital Port D3

2. **ROS Core**: Start the ROS core by running the following command in your terminal:

    ```bash
    roscore
    ```

3. **ROS Serial Node**: Run the ROS serial node to establish communication between your Arduino and ROS. Replace `/dev/ttyACM0` with the correct port if needed, and set the baud rate to match your Arduino code (default is 57600):

    ```bash
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
    ```

4. **List ROS Topics**: To verify that the `hw006_publisher` topic is being published, run:

    ```bash
    rostopic list
    ```

    You should see `hw006_publisher` in the list of topics.

5. **View Collision Detection Events**: To view the Collision detection events published by the Arduino, use the following command:

    ```bash
    rostopic echo hw006_publisher
    ```

    This will display messages published to the `hw006_publisher` topic. You should see "Event: collision detected" when the HW006 module detects a close object.

## Troubleshooting

- If you encounter issues, make sure that your Arduino board is properly connected to your computer, and the Arduino code is successfully uploaded.
- Double-check the serial port (`/dev/ttyACM0` in the example) and baud rate settings.
- Ensure that ROS is correctly installed and the core (`roscore`) is running.

## Conclusion

This tutorial explains how to use an Arduino with an HW006 Module (TCR5000 sensor) to publish close objects (almost collision events) to a ROS topic. You can now integrate this functionality into your robotics projects or explore further ROS capabilities.
