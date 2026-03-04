#!/usr/bin/env python3
import serial
import time
import sys

PORT = '/dev/ttyUSB0'
BAUD = 115200
SERVO_ID = 1

if len(sys.argv) > 1:
    PORT = sys.argv[1]
if len(sys.argv) > 2:
    SERVO_ID = int(sys.argv[2])

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def lss_send(cmd):
    full = f"#{SERVO_ID}{cmd}\r"
    ser.write(full.encode())
    ser.flush()
    time.sleep(0.05)
    resp = b''
    while ser.in_waiting:
        resp += ser.read(ser.in_waiting)
        time.sleep(0.01)
    if resp:
        print(f"  -> {resp.decode(errors='ignore').strip()}")

def lss_query(cmd):
    full = f"#{SERVO_ID}{cmd}\r"
    ser.write(full.encode())
    ser.flush()
    time.sleep(0.1)
    resp = b''
    while ser.in_waiting:
        resp += ser.read(ser.in_waiting)
        time.sleep(0.01)
    decoded = resp.decode(errors='ignore').strip()
    print(f"  {cmd} -> {decoded}")
    return decoded

print(f"=== LSS Servo Test (ID={SERVO_ID}, port={PORT}) ===\n")

print("[1] Querying servo info...")
lss_query("QID")
lss_query("QV")
lss_query("QD")
lss_query("QS")

print(f"\n[2] Moving to center (0 tenths-of-degree)...")
lss_send("D0")
time.sleep(1)

print(f"\n[3] Moving to +300 (30 degrees right)...")
lss_send("D300")
time.sleep(1)

print(f"\n[4] Moving to -300 (30 degrees left)...")
lss_send("D-300")
time.sleep(1)

print(f"\n[5] Back to center...")
lss_send("D0")
time.sleep(1)

print(f"\n[6] Current position:")
lss_query("QD")

print("\n[7] Interactive mode (q to quit):")
print("     Type degrees (e.g. 45) or tenths with 't' suffix (e.g. 900t)")
while True:
    try:
        val = input("  Position: ").strip()
        if val.lower() == 'q':
            break
        if val.lower().endswith('t'):
            tenths = int(float(val[:-1]))
        else:
            tenths = int(float(val) * 10)
        print(f"  Sending D{tenths} ({tenths/10:.1f} deg)...")
        lss_send(f"D{tenths}")
    except ValueError:
        print("  Invalid input")
    except KeyboardInterrupt:
        break

print("\nReturning to center...")
lss_send("D0")
ser.close()
print("Done.")
