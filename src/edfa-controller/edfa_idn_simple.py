#!/usr/bin/env python3
"""
Minimal script to query FiberLabs EDFA identification.
Usage: python3 edfa_idn_simple.py [PORT] [BAUD]
"""

import serial
import time
import sys

# Default settings
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600

print(f"Connecting to {PORT} at {BAUD} baud...")

try:
    # Open serial connection
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=2,
    )

    time.sleep(0.5)  # Let port settle

    # Clear buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Send *IDN? command
    command = b"*IDN?\r"
    print(f"Sending: {repr(command)}")
    ser.write(command)
    ser.flush()

    # Wait and read response
    time.sleep(0.5)

    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting).decode("ascii", errors="ignore")
        print(f"\nReceived ({len(response)} bytes):")
        print(response)
    else:
        print("No response received.")

    ser.close()

except serial.SerialException as e:
    print(f"Serial port error: {e}")
    print(f"\nTip: On Linux, you might need:")
    print(f"  sudo chmod 666 {PORT}")
    print(f"  or add yourself to the dialout group:")
    print(f"  sudo usermod -a -G dialout $USER")
    sys.exit(1)
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
