from datetime import datetime

import paho.mqtt.client as mqtt

from config.config import *

from ..logger.csv_logger import CSVLogger
from ..logger.mongodb_logger import MongoDBLogger


class AttabotDataReceiver:
    def __init__(self, broker_address, broker_port):
        """
        Initializes the AttabotDataReceiver.

        The AttabotDataReceiver connects to an MQTT broker and receives data from different topics.
        It logs the data into CSV files and MongoDB.

        Parameters:
        - broker_address (str): The IP address or hostname of the MQTT broker.
        - broker_port (int): The port number of the MQTT broker.
        """
        self.initialize_robot_variables()
        self.initialize_loggers()

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(broker_address, broker_port, 60)
        self.client.loop_forever()

    def initialize_robot_variables(self):
        """
        Initializes sensor-related variables.
        """
        self.formatted_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        self.robot_id = 0
        self.collision_states = [False] * 6
        self.left_motor = ""
        self.right_motor = ""
        self.light_sensor = False
        self.distance_readings = [0.0] * 25
        self.accelerometer = {
            "eulerX": 0.0,
            "eulerY": 0.0,
            "eulerZ": 0.0,
            "linearAccX": 0.0,
            "linearAccY": 0.0,
            "linearAccZ": 0.0,
            "gyroX": 0.0,
            "gyroY": 0.0,
            "gyroZ": 0.0
        }

    def initialize_loggers(self):
        """
        Initializes CSV and MongoDB loggers.
        """
        # Initialize CSV Logger
        self.csv_logger = CSVLogger()
        columns = CsvConfig.DEFAULT_COLUMNS
        self.csv_logger.initialize_csv(columns)

        # Initialize MongoDB Logger
        connection_string = MongodbConfig.CONNECTION_STRING
        database_name = MongodbConfig.DATABASE_NAME 
        collection_name = MongodbConfig.COLLECTION_NAME  
        self.mongodb_connector = MongoDBLogger(connection_string, database_name, collection_name)

    def update_timestamp(self):
        """
        Updates the timestamp to the current date and time.
        """
        self.formatted_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

    def build_document(self, sensor_id, sensor_data):
        """
        Builds a document for MongoDB insertion.

        Parameters:
        - sensor_id (str): The ID of the sensor.
        - sensor_data (any): The data from the sensor.

        Returns:
        - dict: The document for insertion into MongoDB.
        """
        return {
            "robot_id": self.robot_id,
            "sensor_id": sensor_id,
            "sensor_data": sensor_data,
            "timestamp": self.formatted_timestamp
        }

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback when the MQTT client connects to the broker.

        Parameters:
        - client (mqtt.Client): The MQTT client.
        - userdata: The user data.
        - flags: The flags returned by the broker.
        - rc (int): The result code.
        """
        print("Connected to MQTT broker with result code " + str(rc))
        self.subscribe_to_topics()

    def subscribe_to_topics(self):
        """
        Subscribes to MQTT topics.

        Topics to subscribe to are defined in the `topics` list.
        """
        topics = [
            TopicConfig.ROBOT_ID,
            TopicConfig.COLLISION_STATES,
            TopicConfig.LEFT_MOTOR,
            TopicConfig.RIGHT_MOTOR,
            TopicConfig.LIGHT_SENSOR,
            TopicConfig.DISTANCE_READINGS,
            TopicConfig.ACCELEROMETER
        ]

        for topic in topics:
            self.client.subscribe(topic)

    def on_message(self, client, userdata, msg):
        """
        Callback when a message is received from the MQTT broker.

        Parameters:
        - client (mqtt.Client): The MQTT client.
        - userdata: The user data.
        - msg (mqtt.MQTTMessage): The received message.
        """
        payload = msg.payload.decode("utf-8")
        self.update_timestamp()
        self.handle_message(msg.topic, payload)
        self.log_sensor_data()

    def handle_message(self, topic, payload):
        """
        Handles the incoming MQTT message and updates sensor variables.

        Parameters:
        - topic (str): The MQTT topic.
        - payload (str): The payload of the MQTT message.
        """
        if topic == TopicConfig.ROBOT_ID:
            self.robot_id = int(payload)
        elif topic == TopicConfig.COLLISION_STATES:
            self.collision_states = [bool(int(state)) for state in payload.split(',')]
        elif topic == TopicConfig.LEFT_MOTOR:
            self.left_motor = payload
        elif topic == TopicConfig.RIGHT_MOTOR:
            self.right_motor = payload
        elif topic == TopicConfig.LIGHT_SENSOR:
            self.light_sensor = payload.lower() == "true"
        elif topic == TopicConfig.DISTANCE_READINGS:
            self.distance_readings = [float(value) for value in payload.split(',')]
        elif topic == TopicConfig.ACCELEROMETER:
            accel_data = payload.split(',')
            self.accelerometer = {
                "eulerX": float(accel_data[0]),
                "eulerY": float(accel_data[1]),
                "eulerZ": float(accel_data[2]),
                "linearAccX": float(accel_data[3]),
                "linearAccY": float(accel_data[4]),
                "linearAccZ": float(accel_data[5]),
                "gyroX": float(accel_data[6]),
                "gyroY": float(accel_data[7]),
                "gyroZ": float(accel_data[8])
            }

    def print_sensor_variables(self):
        """
        Prints the sensor variables for debugging purposes.
        """
        print(f"Robot ID: {self.robot_id}")
        print(f"Collision States: {self.collision_states}")
        print(f"Left Motor: {self.left_motor}")
        print(f"Right Motor: {self.right_motor}")
        print(f"Light Sensor: {self.light_sensor}")
        print(f"Distance Readings: {self.distance_readings}")
        print("Accelerometer Data:")
        for key, value in self.accelerometer.items():
            print(f"{key}: {value}")

    def log_sensor_data(self):
        """
        Logs sensor data to CSV and MongoDB.
        """
        sensor_data_list = [
            ('CollisionModule', self.collision_states),
            ('LeftMotor', self.left_motor),
            ('RightMotor', self.right_motor),
            ('LDRLuminosity', self.light_sensor),
            ('DistanceSensor', self.distance_readings),
            ('AccelerometerGyroscope', self.accelerometer),
        ]
        
        print(f"\n >>> Loggin the data: {sensor_data_list}")
        # Log data to CSV
        for sensor_id, sensor_data in sensor_data_list:
            self.csv_logger.insert_row([self.robot_id, sensor_id, sensor_data, self.formatted_timestamp])
        print("Logged onto CSV.")

        # Log data to MongoDB
        for sensor_id, sensor_data in sensor_data_list:
            self.mongodb_connector.insert_document(self.build_document(sensor_id, sensor_data))
        print("Logged onto MongoDB.")
