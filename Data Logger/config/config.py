# config.py

class MqttConfig:
    BROKER_ADDRESS = "192.168.0.11"
    BROKER_PORT = 1883

class CsvConfig:
    CSV_BASE_PATH = "data/csv/"
    DEFAULT_COLUMNS = ['robot_id', 'sensor_id', 'sensor_data', 'timestamp']

class MongodbConfig:
    CONNECTION_STRING = "...mongodb.net/" #replace with your MongoDB connection string
    DATABASE_NAME = "database_attabot"
    COLLECTION_NAME = "collection_attabot"

class TopicConfig:
    ROBOT_ID = "attabot/robot_id"
    COLLISION_STATES = "attabot/collision_states"
    LEFT_MOTOR = "attabot/left_motor"
    RIGHT_MOTOR = "attabot/right_motor"
    LIGHT_SENSOR = "attabot/light_sensor"
    DISTANCE_READINGS = "attabot/distance_readings"
    ACCELEROMETER = "attabot/accelerometer"
