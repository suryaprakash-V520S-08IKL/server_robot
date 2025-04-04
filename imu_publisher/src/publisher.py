#!/usr/bin/env python3
import serial
import time

# Set up the serial port (adjust 'COM3' to your ESP32's port)
# On Linux/Mac, it might be something like '/dev/ttyUSB0' or '/dev/ttyACM0'
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Serial port and baud rate
time.sleep(2)  # Wait for the connection to establish

# Function to read data from ESP32
def read_serial_data():
    while True:
        if ser.in_waiting > 0:  # Check if there is data in the serial buffer
            line = ser.readline().decode('utf-8').strip()  # Read a line and decode it
            print(line)  # Print the data (could be accelerometer/gyroscope values)

if __name__ == '__main__':
    print("Starting serial communication...")
    read_serial_data()

