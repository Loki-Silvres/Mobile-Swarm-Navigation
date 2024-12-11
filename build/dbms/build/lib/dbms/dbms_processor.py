import csv
import os
from ament_index_python.packages import get_package_share_directory

file_dump = open(os.path.join(get_package_share_directory('dbms'),'data_base','object_detected_database.csv'), mode='r', newline='')
reader = csv.reader(file_dump)

database_file = open(os.path.join(get_package_share_directory('dbms'),'data_base','primary_database.csv'), mode='a', newline='')


class CSVRowReader:
    def __init__(self, file_name):
        self.file = open(file_name, mode='r')  # Open the file
        self.csv_reader = csv.reader(self.file)  # Create a CSV reader object
        self.iterator = iter(self.csv_reader)  # Create an iterator from the reader

    def get_next_row(self):
        try:
            return next(self.iterator)  # Get the next row
        except StopIteration:
            return None  # No more rows

    def close(self):
        self.file.close()  # Close the file

reader = CSVRowReader(file_dump)

while True:
    row = reader.get_next_row()  
    if row is None:
        print("End of file reached.")
        continue

    print("Row:", row)  # Process the row
    user_input = input("Press Enter to read the next row, or type 'exit' to quit: ")
    if user_input.lower() == 'exit':
        break

reader.close()
