#!/usr/bin/env python3
import serial
import time
import sys

NAMES = {1: 'throttle', 2: 'brake', 3: 'gear'}

s = serial.Serial('/dev/lss_controller', 115200, timeout=0.3)
time.sleep(0.5)
s.reset_input_buffer()

if len(sys.argv) > 1 and sys.argv[1] == 'assign':
    target_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    name = NAMES.get(target_id, '?')
    print(f'Assigning ID 0 -> {target_id} ({name})...')
    s.write(f'#0CID{target_id}\r'.encode())
    s.flush()
    time.sleep(0.5)
    print('Power cycle the servo, then press Enter...')
    input()
    time.sleep(1)
    s.reset_input_buffer()
    s.write(f'#{target_id}QS\r'.encode())
    s.flush()
    time.sleep(0.3)
    resp = s.read(s.in_waiting)
    if f'*{target_id}'.encode() in resp:
        print(f'ID {target_id} ({name}) confirmed!')
    else:
        print(f'No response at ID {target_id}')

elif len(sys.argv) > 1 and sys.argv[1] == 'move':
    servo_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    print(f'Moving servo {servo_id} — center, +30, -30, center, limp')
    for pos, label in [(0, 'center'), (300, '+30 deg'), (-300, '-30 deg'), (0, 'center')]:
        print(f'  {label}...')
        s.write(f'#{servo_id}D{pos}\r'.encode())
        s.flush()
        time.sleep(2)
    s.write(f'#{servo_id}L\r'.encode())
    s.flush()
    print('  limp')

else:
    print('Scanning IDs 0-5...')
    for i in range(6):
        s.reset_input_buffer()
        s.write(f'#{i}QS\r'.encode())
        s.flush()
        time.sleep(0.2)
        resp = s.read(s.in_waiting)
        status = resp.decode().strip() if resp else '(no response)'
        name = NAMES.get(i, '')
        print(f'  ID {i} {name}: {status}')

s.close()
