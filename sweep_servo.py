#!/usr/bin/env python3
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
ser.write(f'#{servo_id}SD600\r'.encode())
ser.flush()

print(f'{label}')
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
