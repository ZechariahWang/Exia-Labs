#!/usr/bin/env python3
import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/lss_controller'
s = serial.Serial(port, 115200, timeout=0.1)
time.sleep(0.5)
s.reset_input_buffer()

for sid in [0, 1, 2, 3, 254]:
    s.write(f'#{sid}QID\r'.encode())
    s.flush()
    time.sleep(0.2)
    resp = s.read(s.in_waiting).decode(errors='ignore').strip()
    if resp:
        print(f'ID {sid}: {resp}')

s.close()
