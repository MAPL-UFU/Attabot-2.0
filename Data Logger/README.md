# Attabot Data Receiver

The Attabot Data Receiver is a Python script designed to receive sensor data from an Attabot robot through MQTT, log the data into CSV files, and store it in a MongoDB database.

## Getting Started

### Prerequisites

- Python 3
- Paho MQTT (`pip install paho-mqtt`)
- pymongo (`pip install pymongo`)

### Installation

**1. Clone the repository:**

   ```bash
   git clone https://github.com/MAPL-UFU/Attabot-2.0.git
   ```

**2. Navigate into Logger Directory**  
**3. Install the required Python packages:**

   ```bash
   pip install -r requirements.txt
   ```

**Update the configuration in config.py with your specific details.**

### Usage

- **main script:**
   This script creates an instance of AttabotDataReceiver and keeps the program running indefinitely.

- **python attabot_data_receiver.py:**
   This script initializes the MQTT client, connects to the broker, and listens for incoming sensor data. It logs the data into CSV files and a MongoDB database.

- **Configuration:** Update the configuration in config.py with your specific details:
  - MqttConfig: MQTT broker address and port.
  - CsvConfig: Base path for CSV files.
  - MongodbConfig: MongoDB connection string, database name, and collection name.
  - TopicConfig: MQTT topics for various sensor data.

### Contributing

Feel free to contribute to this project. Fork the repository, make your changes, and submit a pull request.
