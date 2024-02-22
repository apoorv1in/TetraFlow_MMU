#!/bin/bash

# Check if the correct number of arguments are provided
if [ $# -ne 2 ]; then
    echo "Usage: $0 <serial_port> <parameter>"
    exit 1
fi

# Assign arguments to variables
SERIAL_PORT=$1
PARAMETER=$2

# Check if the serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo "Error: Serial port $SERIAL_PORT not found."
    exit 1
fi

# Define a Python script within the shell script using a here document
python_script=$(cat <<EOF
import serial
import sys
import time

# Initialize ser to None
ser = None

try:
    # Get serial port and parameter from shell script arguments
    serial_port = sys.argv[1]
    parameter = sys.argv[2]

    # Open the serial port
    ser = serial.Serial(serial_port)

    # Write the parameter to the serial port
    ser.write(parameter.encode())

    print("Command: ", parameter)

    # Set timeout for response
    timeout = 90  # Timeout in seconds
    start_time = time.time()

    # Read response from the serial port
    responses = []
    while True:
        response = ser.readline().decode().strip()
        if response:
            responses.append(response)
            print("Received response:", response)
        if 'OK' in response:
            break
        if time.time() - start_time > timeout:
            print("Timeout while waiting for 'OK' response.")
            break

except Exception as e:
    print("An exception occurred:", e)

finally:
    # Close the serial port if it was opened
    if ser is not None:
        ser.close()
EOF
)

# Execute the embedded Python script
python3 -c "$python_script" "$SERIAL_PORT" "$PARAMETER"
