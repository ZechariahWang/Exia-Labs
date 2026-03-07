#!/usr/bin/env python3
import serial
import time

PORT = '/dev/lss_controller'

for baud in [9600, 19200, 38400, 57600, 115200]:
    ser = serial.Serial(PORT, baud, timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b'#254QID\r')
    ser.flush()
    time.sleep(0.5)
    resp = ser.read(ser.in_waiting)
    print(f'{baud}: {repr(resp)}')
    ser.close()
