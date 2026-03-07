#!/usr/bin/env python3
import serial
import time

PORT = '/dev/lss_controller'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1.0)
time.sleep(0.5)
ser.reset_input_buffer()

print(f"Port: {PORT} @ {BAUD}")
print()

print("Query ID 0 (factory default)...")
ser.write(b'#0QID\r')
ser.flush()
time.sleep(0.3)
resp = ser.read(ser.in_waiting)
print(f"  Response: {repr(resp)}")
print()

print("Broadcast query ID 254 (all servos)...")
ser.reset_input_buffer()
ser.write(b'#254QID\r')
ser.flush()
time.sleep(0.5)
resp = ser.read(ser.in_waiting)
print(f"  Response: {repr(resp)}")
print()

for sid in [1, 2, 3]:
    ser.reset_input_buffer()
    ser.write(f'#{sid}QID\r'.encode())
    ser.flush()
    time.sleep(0.3)
    resp = ser.read(ser.in_waiting)
    label = {1: 'throttle', 2: 'brake', 3: 'gear'}.get(sid, '')
    status = resp.decode(errors='ignore').strip() if resp else 'no response'
    print(f"  ID {sid} ({label}): {status}")

ser.close()
print()
print("Done. If no responses, check servo power and wiring.")
