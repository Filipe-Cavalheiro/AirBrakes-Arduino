import serial
import csv

NUMBERLINES_CSV = 874

# Settings
arduino_port = 'COM7'  # Change this to your Arduino port
baud_rate = 9600
csv_file_path = "Your file path//example.csv"

# Open serial connection
ser = serial.Serial(arduino_port, baud_rate)

# Function to read and parse CSV file
def read_csv(file_path):
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        data = [row for row in reader if not row[0].startswith("#")]
    return data

csv_data = read_csv(csv_file_path) # Read CSV file
line_index = 0
while line_index < NUMBERLINES_CSV:
    arduino_input = ser.read_until(b'#').decode().strip()
    arduino_input = arduino_input[:-1]
    if(arduino_input == "A"):
        line_to_send = ','.join(csv_data[line_index])
        line_index += 1
        ser.write(line_to_send.encode('utf-8'))
        print("Sent: " + line_to_send)
        if(line_index >= NUMBERLINES_CSV):
            exit()
    else:
        print("Angle: " + arduino_input + "\n")
    arduino_input = None