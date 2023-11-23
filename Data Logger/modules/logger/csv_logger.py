import csv
import os
from datetime import datetime

from config.config import CsvConfig


class CSVLogger:
    def __init__(self):
        """
        Initializes the CSVLogger.

        The CSVLogger is used for logging data into CSV files.

        Attributes:
        - base_path (str): The base path where CSV files are stored.
        - file_suffix (str): The suffix added to the CSV file name, including a timestamp and file count.
        """
        self.base_path = os.path.join(os.getcwd(), CsvConfig.CSV_BASE_PATH)
        self.file_suffix = self._generate_file_suffix()

    def _generate_file_suffix(self):
        """
        Generates a unique file suffix based on the current file count and timestamp.

        Returns:
        - str: The generated file suffix.
        """
        os.makedirs(os.path.dirname(self.base_path), exist_ok=True)

        file_count = len([f for f in os.listdir(self.base_path) if os.path.isfile(os.path.join(self.base_path, f))])
        timestamp = datetime.now().strftime('%Y_%m_%d__%H_%M_%S')
        return f'_{file_count + 1}_{timestamp}'

    def _get_file_path(self):
        """
        Generates the full file path for the CSV file.

        Returns:
        - str: The full file path.
        """
        file_name = f'data{self.file_suffix}.csv'
        return os.path.join(self.base_path, file_name)

    def initialize_csv(self, columns):
        """
        Initializes a new CSV file with the specified columns.

        Parameters:
        - columns (list): The list of column names.

        Prints a log statement indicating the initialization of the CSV file.
        """
        file_path = self._get_file_path()

        with open(file_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=';')
            csv_writer.writerow(columns)

        print(f'CSV file "{file_path}" initialized with headers in path: {self.base_path}')

    def create_headers(self, headers):
        """
        Adds headers to an existing CSV file.

        Parameters:
        - headers (list): The list of headers to be added.

        Prints a log statement indicating the addition of headers to the CSV file.
        """
        file_path = self._get_file_path()

        if os.path.exists(file_path):
            with open(file_path, mode='a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(headers)
            print(f'Headers added to CSV file "{file_path}"')
        else:
            print(f'CSV file "{file_path}" does not exist. Please initialize the file first.')

    def insert_row(self, values):
        """
        Inserts a new row of values into an existing CSV file.

        Parameters:
        - values (list): The list of values to be inserted.

        Prints a log statement indicating the insertion of a row into the CSV file.
        """
        file_path = self._get_file_path()

        if os.path.exists(file_path):
            with open(file_path, mode='a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(values)
        else:
            print(f'CSV file "{file_path}" does not exist. Please initialize the file first.')
