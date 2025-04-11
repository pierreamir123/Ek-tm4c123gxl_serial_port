import serial
import time

# Define the serial port and baud rate
SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your actual port (e.g., /dev/ttyACM0)
BAUD_RATE = 115200

# Open the serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# Read and print data from the serial port
try:
    while True:
        if ser.in_waiting > 0:  # Check if data is available
            data = ser.readline().decode('utf-8').strip()  # Read and decode the data
            print(f"Received: {data}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()