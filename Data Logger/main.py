import time
from datetime import datetime

from config.config import MqttConfig
from modules.data_handler.attabot_data_receiver import AttabotDataReceiver

if __name__ == "__main__":
    # Create AttabotDataReceiver instance
    attabot_receiver = AttabotDataReceiver(MqttConfig.BROKER_ADDRESS, MqttConfig.BROKER_PORT)

    # Infinite loop to keep the program running
    while True:
        # Adjust the delay as needed
        time.sleep(2)

