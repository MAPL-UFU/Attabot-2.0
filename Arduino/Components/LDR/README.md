# Arduino ROS LDR Sensor Tutorial

This tutorial explains how to use an Arduino board with an LDR (Light Dependent Resistor) sensor to publish light detection events to a ROS (Robot Operating System) topic.

## Prerequisites

Before getting started, make sure you have the following:

- An Arduino board with the required hardware components (LDR module, wires, etc.).
- ROS (Robot Operating System) installed on your Linux machine.

## Setup

1. **Arduino Code**: Upload the provided Arduino code to your Arduino board. This code interfaces with the LDR sensor and publishes light detection events to a ROS topic. Ensure the following connections:

    **LDR Module PINOUT**:

    - GND: GND
    - VCC: 5V
    - DO: Analog Port A0

2. **ROS Core**: Start the ROS core by running the following command in your terminal:

    ```bash
    roscore
    ```

3. **ROS Serial Node**: Run the ROS serial node to establish communication between your Arduino and ROS. Replace `/dev/ttyACM0` with the correct port if needed, and set the baud rate to match your Arduino code (default is 57600):

    ```bash
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
    ```

4. **List ROS Topics**: To verify that the `ldr_publisher` topic is being published, run:

    ```bash
    rostopic list
    ```

    You should see `ldr_publisher` in the list of topics.

5. **View Light Detection Events**: To view the light detection events published by the Arduino, use the following command:

    ```bash
    rostopic echo ldr_publisher
    ```

    This will display messages published to the `ldr_publisher` topic. You should see "Event: light detected" when the LDR sensor detects light above the configured threshold.

3. **Customization**: You can customize the behavior of the Arduino code by modifying the `lightIntensity` threshold in the code to adjust the sensitivity to light changes.

## Troubleshooting

- If you encounter issues, make sure that your Arduino board is properly connected to your computer, and the Arduino code is successfully uploaded.
- Double-check the serial port (`/dev/ttyACM0` in the example) and baud rate settings.
- Ensure that ROS is correctly installed and the core (`roscore`) is running.

## Conclusion

This tutorial explains how to use an Arduino with an LDR sensor to publish light detection events to a ROS topic. You can now integrate this functionality into your robotics projects or explore further ROS capabilities.
