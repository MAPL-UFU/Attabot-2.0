# ESP32 Source Code - Communicating the Arduino Due with MQTT Broker

## **Part 1: Setting up Mosquitto MQTT Broker**

### **Step 1: Update and Upgrade**

Before you begin, make sure your Ubuntu machine is up-to-date:

```bash
sudo apt update
sudo apt upgrade
```

### **Step 2: Install Mosquitto MQTT Broker**

You can install the Mosquitto MQTT broker by running the following command:

```bash
sudo apt install mosquitto mosquitto-clients
```

### **Step 3: Start and Enable Mosquitto**

Start and enable the Mosquitto service to run on boot:

```bash
sudo systemctl start mosquitto
sudo systemctl enable mosquitto
```

### **Step 4: Verify Mosquitto Status**

Check the status to make sure Mosquitto is running without any issues:

```bash
sudo systemctl status mosquitto
```

Mosquitto should be up and running now.

To configure MQTT topics with username and password authentication in Mosquitto, you'll need to set up authentication in the Mosquitto broker and then configure your ESP32 client to provide the correct username and password when connecting. Here are the steps:

### **Step 5: Configure Authentication in Mosquitto**

1. Open the Mosquitto configuration file for editing. The file is typically located at **`/etc/mosquitto/mosquitto.conf`**:

    ```bash
    sudo nano /etc/mosquitto/mosquitto.conf
    ```

2. Add the following lines to set up username and password authentication. Replace **`your_username`** and **`your_password`** with your desired credentials:

    ```bash
    allow_anonymous false
    password_file /etc/mosquitto/passwd
    ```

3. Save the changes and exit the text editor.
4. Create a password file using the **`mosquitto_passwd`** command. Run the following commands to create the file and add a user:

    ```bash
    sudo mosquitto_passwd -c /etc/mosquitto/passwd your_username
    ```

    You will be prompted to enter the password for your_username.

    /**n this case, I used the username: divinojr and password: attabot**

5. Restart Mosquitto to apply the changes:

    ```bash
    sudo systemctl restart mosquitto
    ```

    Now, Mosquitto is configured to require username and password authentication for clients.

### **Step 6: Sending a message to Attabot Topic via terminal**

To acomplish this step we’re going to use **`mosquitto_sub`** and **`mosquitto_pub`** command-line utilities.

**Subscribing to the 'attabot_logger' Topic:**

1. Open a terminal window.
2. Use the **`mosquitto_sub`** command to subscribe to the 'attabot_logger' topic. Replace **`your_username`** and **`your_password`** with your MQTT username and password if you have configured authentication:

    ```bash
    mosquitto_sub -h YourMqttBrokerIPAddress -t attabot_logger -u your_username -P your_password
    ```

    As we have configured authentication, I used the command:

    ```bash
    mosquitto_sub -h localhost -t attabot_logger -u divinojr -P attabot
    ```

    If you haven't configured authentication in Mosquitto, you can omit the **`-u`** and **`-P`** flags.

3. You should now be subscribed to the 'attabot_logger' topic, and you will see any messages that are published to it.

    **Observation**: A great way of subscribing to all topics and subtopics is to use the following command:

      ```bash
      mosquitto_sub -h localhost -t '#'
      ```

**Publishing to the 'attabot_logger' Topic:**

1. Open a different terminal window.
2. Use the **`mosquitto_pub`** command to publish a message to the 'attabot_logger' topic. Again, replace **`your_username`** and **`your_password`** if you have configured authentication:

    ```bash
    mosquitto_pub -h YourMqttBrokerIPAddress -t attabot_logger -m "Hello, MQTT!" -u your_username -P your_password
    ```

    ```bash
    mosquitto_pub -h localhost -t attabot_logger -m "Hello, MQTT!" -u divinojr -P attabot
    ```

    If you haven't configured authentication, omit the **`-u`** and **`-P`** flags.

3. The message "Hello, MQTT!" will be published to the 'attabot_logger' topic, and you should see it in the terminal where you are subscribed.

By using the **`mosquitto_sub`** and **`mosquitto_pub`** commands in the terminal, you can easily test the MQTT broker and the 'attabot_logger' topic to ensure they are working as expected. This is a useful way to debug and verify your MQTT configuration before connecting your ESP32 device.

## **Part 2: Connect ESP32 using PlatformIO**

Now, let's set up an ESP32 device to connect to the MQTT broker using PlatformIO.

### **Step 1: Install PlatformIO**

If you haven't already, you'll need to install PlatformIO on your development machine. You can do this as a VS Code extension or through the command line.

### **Step 2: Create a New Project**

In your preferred development environment (VS Code with PlatformIO extension, if you've chosen it), create a new PlatformIO project.

### **Step 3: Configure ESP32**

Open the **`platformio.ini`** file in your project folder and add the necessary configuration for your ESP32. Here's a sample **`platformio.ini`**:

```plain-text
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
upload_speed = 115200
lib_deps = 
    knolleary/PubSubClient@^2.8
    ArduinoJson@^6.18
```

This configuration sets up the ESP32 board, Arduino framework, and necessary libraries.

### **Step 4: Write MQTT Code**

In your PlatformIO project, create a new source file (e.g., **`main.cpp`**) and write the code to connect to the MQTT broker. Here's a simple example using the PubSubClient library:

```c++

/* This code just publish a simple message on a topic */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char *ssid = "jose-wifi";         // YourWiFiSSID
const char *password = "jose-password";     // YourWiFiPassword
const char *mqttServer = "182.412.0.13"; // YourMqttBrokerIPAddress
const int mqttPort = 1883;
const char *mqttUser = "divinojr";    // YourMqttUsername
const char *mqttPassword = "attabot"; // YourMqttPassword

WiFiClient espClient;
PubSubClient client(espClient);

int x = 0;

void callback(char *topic, byte *payload, unsigned int length)
{
  // Handle MQTT messages here
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("connected");
      client.subscribe("attabot_logger");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200); // Set the baud rate for serial communication

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize MQTT client
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  // Connect to MQTT broker
  if (client.connect("ESP32Client", mqttUser, mqttPassword))
  {
    Serial.println("Connected to MQTT Broker");
    client.subscribe("attabot_logger");
  }
  else
  {
    Serial.println("Connection to MQTT Broker failed");
  }
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // Your code to collect data
  String data = "Hello, from Attabot 2.0: " + String(x);

  if (client.connected())
  {
    // Publish to the 'attabot_logger' topic
    client.publish("attabot_logger", data.c_str());
  }
  delay(1000); // Adjust the delay as needed

  x++;
}
```

## Part 3: ESP32 Workflow for WiFi and MQTT Connection

This project demonstrates the workflow for an ESP32-based device to connect to a WiFi network, handle data with a `DataHandler` class, and communicate with an MQTT broker. The example code uses three classes: `WiFiHandler`, `MQTTHandler`, and `DataHandler`.

### Prerequisites

Before getting started, make sure you have the following:

- Arduino IDE installed on your development machine.
- ESP32 board support installed in the Arduino IDE.
- Required libraries: `WiFi.h`, `PubSubClient.h`.

### Project Structure

- `WiFiHandler.h` and `WiFiHandler.cpp`: Classes for handling WiFi connection.
- `MQTTHandler.h` and `MQTTHandler.cpp`: Classes for handling MQTT connection.
- `DataHandler.h` and `DataHandler.cpp`: Class for handling data operations.

### Workflow

1. **Include Necessary Libraries:**

   Include the required libraries in your Arduino sketch:

   ```cpp
   #include <Arduino.h>
   #include "WiFiHandler.h"
   #include "MQTTHandler.h"
   #include "DataHandler.h"
   ```

2. Set Up WiFi, MQTT, and DataHandler Credentials:

Set your WiFi SSID, password, MQTT broker IP address, and port, as well as any data-related parameters:

```cpp
const char *ssid = "YourWiFiSSID";
const char*password = "YourWiFiPassword";
const char *mqttServer = "YourMqttBrokerIPAddress";
const int mqttPort = 1883;

3. Create Instances of WiFiHandler, MQTTHandler, and DataHandler:

4. **Messages Structure:**

  The message structure follow this pattern:

  ```plaintext
  < id: [id_value], [collision_states], [left_motor_value], [right_motor_value], [light_sensor_value], [distance_readings], [accelerometer_values] >;
  ```

  | Value                           | Type                      | Example |
  | ------------------------------- | ------------------------- | ------- |
  | **Robot ID Value**              | Integer                   | `1` |
  | **Collision States**            | Array of Booleans (6)     | `true, false, true, false, true, true` |
  | **Left and Right Motor Values** | String                    | `forward, stopped` |
  | **Light Sensor Value**          | Boolean                   | `true`  |
  | **Distance Readings**           | Array of Floats (25)      | `1.23, 2.45, 3.67, 4.89, 5.01, 6.12, 7.34, 8.56, 9.78, 10.90, 11.21, 12.43, 13.65, 14.87, 15.09, 16.30, 17.52, 18.74, 19.96, 20.18, 21.40, 22.61, 23.83, 24.05, 25.27` |
  | **Accelerometer Values**        | Struct (SensorData)       |  |
  | Accelerometer                   | - **Euler Angles**        | `{ X: 1.23, Y: 2.45, Z: 3.67 }` |
  | Accelerometer                   | - **Linear Acceleration** | `{ X: 4.01, Y: 5.12, Z: 6.34 }` |
  | Accelerometer                   | - **Gyro Values**         | `{ X: 7.56, Y: 8.78, Z: 9.01 }` |
  | **Accelerometer Struct**        | **Example:**              | `{ Euler: { X: 1.23, Y: 2.45, Z: 3.67 }, , LinearAcc: { X: 4.01, Y: 5.12, Z: 6.34 }, Gyro: { X: 7.56, Y: 8.78, Z: 9.01 } }` |
