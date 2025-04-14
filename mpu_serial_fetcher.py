import serial
import time

# Constants
SENSITIVITY_FACTOR = 16384.0  # For ±2g range
GRAVITY = 9.81  # Acceleration due to gravity in m/s²

SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your actual port (e.g., /dev/ttyACM0)
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

try:
    while True:
        if ser.in_waiting > 0:  # Check if data is available
            data = ser.readline().decode('utf-8').strip()  # Read and decode the data
            print(f"Received: {data}")

            if data.startswith("Accel:") and "Gyro:" in data and "Temp:" in data:
                parts = data.split()
                accel = parts[1].split(",")
                gyro = parts[3].split(",")
                temp = float(parts[5])
                pitch = parts[7]
                roll = parts[9]

                # Convert raw accelerometer values to integers
                ax_g = int(accel[0])
                ay_g = int(accel[1])
                az_g = int(accel[2])

                # Convert g-force to m/s²
                ax_ms2 = ax_g * GRAVITY
                ay_ms2 = ay_g * GRAVITY
                az_ms2 = az_g * GRAVITY

                # Print the parsed data
                print(f"Accelerometer: X={ax_raw}, Y={ay_raw}, Z={az_raw}")
                print(f"Gyroscope: X={gyro[0]}, Y={gyro[1]}, Z={gyro[2]}")
                print(f"Temperature: {temp:.2f}°C")
                print(f"Pitch: {pitch}°, Roll: {roll}°")
                print(f"Acceleration (m/s²): X={ax_ms2:.2f}, Y={ay_ms2:.2f}, Z={az_ms2:.2f}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()