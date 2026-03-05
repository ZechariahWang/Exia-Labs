#!/usr/bin/env python3
#
# Assign a new ID to an LSS servo.
# Used during initial setup to give each servo its expected ID
# (1=throttle, 2=brake, 3=gear).
#
# Usage:
#   python3 assign_servo_id.py <new_id> [port] [current_id]
#
# Arguments:
#   new_id      - ID to assign (required): 1=throttle, 2=brake, 3=gear
#   port        - serial port (default: /dev/lss_controller)
#   current_id  - servo's current ID (default: 0, factory default)
#
# Examples:
#   python3 assign_servo_id.py 1                              # assign ID 1 (throttle)
#   python3 assign_servo_id.py 2 /dev/ttyUSB0                 # assign ID 2 on ttyUSB0
#   python3 assign_servo_id.py 3 /dev/lss_controller 254      # reassign from ID 254 to 3
#
# Tip: connect only ONE servo at a time when assigning from the factory default ID (0).
#
# Requires: pyserial (pip install pyserial)
import serial
import time
import sys

PORT = '/dev/lss_controller'
BAUD = 115200

if len(sys.argv) < 2:
    print('Usage: python3 assign_servo_id.py <new_id> [port] [current_id]')
    print('  new_id     : ID to assign (1=throttle, 2=brake, 3=gear)')
    print('  port       : serial port (default: /dev/lss_controller)')
    print('  current_id : current servo ID (default: 0)')
    sys.exit(1)

new_id = int(sys.argv[1])
if len(sys.argv) > 2:
    PORT = sys.argv[2]
current_id = int(sys.argv[3]) if len(sys.argv) > 3 else 0

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.5)
ser.reset_input_buffer()

ser.write(f'#{current_id}CID{new_id}\r'.encode())
ser.flush()
time.sleep(0.2)

ser.write(f'#{new_id}QID\r'.encode())
ser.flush()
time.sleep(0.2)
resp = ser.read(ser.in_waiting).decode(errors='ignore').strip()

if f'*{new_id}' in resp:
    print(f'Servo ID set to {new_id}')
else:
    print(f'No response on ID {new_id} (got: {resp})')

ser.close()
