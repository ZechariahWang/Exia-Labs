#!/usr/bin/env python3
#
# Test and tune the gear shift servo via Jetson GPIO PWM.
# Uses servo degrees (0-180) like Arduino Servo.write(), converted to duty cycle internally.
#
# Usage:
#   python3 test_gearshift.py [pin]
#
# Arguments:
#   pin  - Jetson BOARD pin number (default: 32)
#
# Interactive commands:
#   <number>   - set servo angle in degrees (e.g. 80)
#   r          - go to reverse (default 57 deg)
#   n          - go to neutral (default 80 deg)
#   h          - go to high gear (default 110 deg)
#   r=<val>    - update reverse preset (e.g. r=55)
#   n=<val>    - update neutral preset
#   h=<val>    - update high preset
#   sweep      - sweep from 0 to 180 in 5 deg steps (1s each)
#   q          - quit (returns to neutral before exiting)
#
# Requires: Jetson.GPIO (run on Jetson only)

import sys
import time

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print('Jetson.GPIO not available')
    sys.exit(1)

PIN = int(sys.argv[1]) if len(sys.argv) > 1 else 32
FREQ = 50
MIN_PULSE_MS = 1.0
MAX_PULSE_MS = 2.0
PERIOD_MS = 1000.0 / FREQ

def angle_to_duty(degrees):
    degrees = max(0.0, min(180.0, degrees))
    pulse_ms = MIN_PULSE_MS + (degrees / 180.0) * (MAX_PULSE_MS - MIN_PULSE_MS)
    return (pulse_ms / PERIOD_MS) * 100.0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT)
pwm = GPIO.PWM(PIN, FREQ)
pwm.start(angle_to_duty(80))

reverse = 57.0
neutral = 80.0
high = 110.0
current = 80.0

print(f'Gear shift test — pin {PIN}, {FREQ}Hz')
print(f'Units: servo degrees (same as Arduino Servo.write)')
print(f'Defaults: R={reverse:.0f}  N={neutral:.0f}  H={high:.0f}')
print()
print('Commands:')
print('  <number>  : set angle in degrees (0-180)')
print('  r         : go to reverse')
print('  n         : go to neutral')
print('  h         : go to high')
print('  sweep     : sweep 0 -> 180 in 5 deg steps')
print('  q         : quit')
print()

try:
    while True:
        cmd = input(f'[{current:.0f} deg] > ').strip().lower()
        if not cmd:
            continue
        if cmd == 'q':
            break
        elif cmd == 'r':
            current = reverse
            pwm.ChangeDutyCycle(angle_to_duty(current))
            print(f'  -> reverse ({current:.0f} deg)')
        elif cmd == 'n':
            current = neutral
            pwm.ChangeDutyCycle(angle_to_duty(current))
            print(f'  -> neutral ({current:.0f} deg)')
        elif cmd == 'h':
            current = high
            pwm.ChangeDutyCycle(angle_to_duty(current))
            print(f'  -> high ({current:.0f} deg)')
        elif cmd == 'sweep':
            print('  sweeping 0 -> 180 (5 deg steps, 1s each)')
            d = 0.0
            while d <= 180.0:
                pwm.ChangeDutyCycle(angle_to_duty(d))
                current = d
                print(f'  {d:.0f} deg ({angle_to_duty(d):.2f}%)')
                time.sleep(1.0)
                d += 5.0
            print('  sweep done')
        elif cmd.startswith('r='):
            reverse = float(cmd[2:])
            print(f'  reverse set to {reverse:.0f} deg')
        elif cmd.startswith('n='):
            neutral = float(cmd[2:])
            print(f'  neutral set to {neutral:.0f} deg')
        elif cmd.startswith('h='):
            high = float(cmd[2:])
            print(f'  high set to {high:.0f} deg')
        else:
            try:
                current = float(cmd)
                pwm.ChangeDutyCycle(angle_to_duty(current))
                print(f'  -> {current:.0f} deg ({angle_to_duty(current):.2f}%)')
            except ValueError:
                print('  unknown command')
except (KeyboardInterrupt, EOFError):
    pass

pwm.ChangeDutyCycle(angle_to_duty(neutral))
time.sleep(0.3)
pwm.stop()
GPIO.cleanup()
print('\ndone')
