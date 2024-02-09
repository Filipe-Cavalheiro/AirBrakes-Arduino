import serial
import csv

# Define the serial port and baud rate
arduino_port = 'COM7'  # Change this to your Arduino port
baud_rate = 9600

# Open serial connection
ser = serial.Serial(arduino_port, baud_rate)

# CSV file path
csv_file_path = "C:\\Users\\caval\\Documents\\Universidade\\PIIC\\test.csv"

# Function to read and parse CSV file
def read_csv(file_path):
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        data = [row for row in reader if not row[0].startswith("#")]
    return data

# Read CSV file
csv_data = read_csv(csv_file_path)
line_index = 0

while line_index < 874:
    arduino_input = ser.read_until(b'#').decode().strip()
    arduino_input = arduino_input[:-1]
    if(arduino_input == "A"):
        line_to_send = ','.join(csv_data[line_index])
        line_index += 1
        ser.write(line_to_send.encode('utf-8'))
        print("Sent: " + line_to_send)
        if(line_index >= 874):
            exit()
    else:
        print("Angle: " + arduino_input + "\n")
    arduino_input = None