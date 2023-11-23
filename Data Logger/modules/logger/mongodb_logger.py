from pymongo import MongoClient


class MongoDBLogger:
    def __init__(self, connection_string, database_name, collection_name):
        """
        Initializes the MongoDBLogger.

        Parameters:
        - connection_string (str): The connection string to connect to the MongoDB cluster.
        - database_name (str): The name of the MongoDB database.
        - collection_name (str): The name of the MongoDB collection.
        """
        print("Connecting to MongoDB...")
        self.client = MongoClient(connection_string)
        self.db = self.client.get_database(database_name)
        self.collection = self.db.get_collection(collection_name)
        print("Connected to MongoDB!")

    def insert_document(self, sensor_data):
        """
        Inserts a document into the MongoDB collection.

        Parameters:
        - sensor_data (dict): The sensor data to be inserted as a document.
        """
        # Assuming sensor_data is a dictionary containing sensor information
        self.collection.insert_one(sensor_data)

    def close_connection(self):
        """
        Closes the MongoDB connection.
        """
        print("Closing MongoDB connection...")
        self.client.close()
        print("MongoDB connection closed.")
