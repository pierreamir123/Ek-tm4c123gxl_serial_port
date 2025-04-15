import serial
import matplotlib.pyplot as plt
import re

# Configure serial port (update with your USB Virtual COM Port)
SERIAL_PORT = '/dev/ttyACM0'  # Update this if needed
BAUD_RATE = 115200
TIMEOUT = 1

# Initialize serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    exit(1)

# Data storage
distances = []
speeds = []

# Regular expression for parsing the expected UART message format
data_pattern = re.compile(r"^Distance: ([\d.]+) m \| Speed: ([\d.]+) m/s \| Dir: (\w+)$")

def validate_data(line):
    """
    Validates the received line of data against the expected format.
    Returns parsed values if valid, otherwise None.
    """
    match = data_pattern.match(line)
    if not match:
        print(f"Invalid data format: {line}")
        return None

    try:
        distance = float(match.group(1))
        speed = float(match.group(2))
        direction = match.group(3)

        # Validate numerical ranges
        if distance < 0 or speed < 0:
            print(f"Invalid numerical values: Distance={distance}, Speed={speed}")
            return None

        if direction not in ["FWD", "REV"]:
            print(f"Invalid direction: {direction}")
            return None

        return distance, speed, direction

    except ValueError as e:
        print(f"Error parsing numerical values: {e}")
        return None

try:
    while True:
        try:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()

            # Print the raw received data as it is
            print(f"Raw Serial Data: {line}")

            # Skip empty lines
            if not line:
                continue

            # Validate and parse the data
            parsed_data = validate_data(line)
            if parsed_data is None:
                continue

            # Unpack validated data
            distance, speed, direction = parsed_data

            # Append data to lists for plotting
            distances.append(distance)
            speeds.append(speed)

            # Print the parsed data to the console
            print(f"Distance: {distance} m, Speed: {speed} m/s, Direction: {direction}")

            # Plotting
            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(distances, label="Distance (m)")
            plt.legend()

            plt.subplot(2, 1, 2)
            plt.plot(speeds, label="Speed (m/s)", color='orange')
            plt.legend()

            plt.pause(0.01)

        except UnicodeDecodeError:
            print("Error decoding serial data. Skipping...")
            continue

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.show()