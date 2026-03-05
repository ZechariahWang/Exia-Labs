#!/usr/bin/env python3
#
# Sweep an LSS servo through its preset positions (throttle, brake, or gear).
# Steps through each position on Enter, useful for verifying range of motion.
#
# Usage:
#   python3 sweep_servo.py <throttle|brake|gear> [port]
#
# Arguments:
#   preset  - which servo to sweep (required):
#               throttle (ID 1): neutral(5) <-> max(550)
#               brake    (ID 2): released(900) <-> engaged(-100)
#               gear     (ID 3): reverse(-330) <-> neutral(-100) <-> high(200)
#   port    - serial port (default: /dev/lss_controller)
#
# Examples:
#   python3 sweep_servo.py throttle
#   python3 sweep_servo.py brake /dev/ttyUSB0
#
# Press Enter to cycle to next position, 'q' to quit.
# On exit the servo is sent the limp command (L).
#
# Requires: pyserial (pip install pyserial)
import serial
import time
import sys

PORT = '/dev/lss_controller'
BAUD = 115200

PRESETS = {
    'throttle': (1, [5, 550], 'THROTTLE: neutral(0.5) <-> max(55)'),
    'brake':    (2, [900, -100], 'BRAKE: released(900) <-> engaged(-100)'),
    'gear':     (3, [-330, -100, 200], 'GEAR: reverse(-330) <-> neutral(-100) <-> high(200)'),
}

if len(sys.argv) < 2 or sys.argv[1] not in PRESETS:
    print('Usage: python3 sweep_servo.py <throttle|brake|gear> [port]')
    sys.exit(1)

name = sys.argv[1]
if len(sys.argv) > 2:
    PORT = sys.argv[2]

servo_id, positions, label = PRESETS[name]

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.5)
ser.reset_input_buffer()

print(f'Verifying servo ID {servo_id} on {PORT}...')
ser.write(f'#{servo_id}QID\r'.encode())
ser.flush()
time.sleep(0.2)
resp = b''
while ser.in_waiting:
    resp += ser.read(ser.in_waiting)
    time.sleep(0.01)
resp_str = resp.decode(errors='ignore').strip()
if f'*{servo_id}' not in resp_str:
    print(f'No response from servo ID {servo_id} (got: "{resp_str}")')
    print('Check wiring and that the servo has the correct ID assigned.')
    ser.close()
    sys.exit(1)
print(f'Servo ID {servo_id} responded: {resp_str}')

ser.write(f'#{servo_id}SD600\r'.encode())
ser.flush()

print(f'\n{label}')
print('Press Enter to move to next position, q to quit\n')

i = 0
while True:
    pos = positions[i % len(positions)]
    ser.write(f'#{servo_id}D{pos}\r'.encode())
    ser.flush()
    print(f'  -> D{pos} ({pos/10:.1f} deg)')
    try:
        val = input('  [Enter=next, q=quit] ')
        if val.strip().lower() == 'q':
            break
    except KeyboardInterrupt:
        break
    i += 1

ser.write(f'#{servo_id}L\r'.encode())
ser.flush()
ser.close()
print('Done.')
