#!/usr/bin/env python3
#
# Test and tune the gear shift servo via Jetson GPIO PWM.
# Interactive tool for finding the correct duty cycle for each gear position.
#
# Usage:
#   python3 test_gearshift.py [pin]
#
# Arguments:
#   pin  - Jetson BOARD pin number (default: 32)
#
# Interactive commands:
#   <number>   - set duty cycle directly (e.g. 6.94)
#   r          - go to reverse duty (default 5.67%)
#   n          - go to neutral duty (default 6.94%)
#   h          - go to high gear duty (default 8.61%)
#   r=<val>    - update reverse preset (e.g. r=5.5)
#   n=<val>    - update neutral preset
#   h=<val>    - update high preset
#   sweep      - sweep from 5.0% to 10.0% in 0.25% steps (1s each)
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

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT)
pwm = GPIO.PWM(PIN, FREQ)
pwm.start(7.0)

print(f'Gear shift test — pin {PIN}, {FREQ}Hz')
print(f'Current defaults: R=5.67  N=6.94  H=8.61')
print()
print('Commands:')
print('  <number>  : set duty cycle (e.g. 6.94)')
print('  r         : go to reverse duty')
print('  n         : go to neutral duty')
print('  h         : go to high duty')
print('  sweep     : sweep from 5.0 to 10.0 in 0.25 steps')
print('  q         : quit')
print()

reverse = 5.67
neutral = 6.94
high = 8.61
current = 7.0

try:
    while True:
        cmd = input(f'[{current:.2f}%] > ').strip().lower()
        if not cmd:
            continue
        if cmd == 'q':
            break
        elif cmd == 'r':
            current = reverse
            pwm.ChangeDutyCycle(current)
            print(f'  -> reverse ({current:.2f}%)')
        elif cmd == 'n':
            current = neutral
            pwm.ChangeDutyCycle(current)
            print(f'  -> neutral ({current:.2f}%)')
        elif cmd == 'h':
            current = high
            pwm.ChangeDutyCycle(current)
            print(f'  -> high ({current:.2f}%)')
        elif cmd == 'sweep':
            print('  sweeping 5.0 -> 10.0 (0.25 steps, 1s each)')
            d = 5.0
            while d <= 10.0:
                pwm.ChangeDutyCycle(d)
                current = d
                print(f'  {d:.2f}%')
                time.sleep(1.0)
                d += 0.25
            print('  sweep done')
        elif cmd.startswith('r='):
            reverse = float(cmd[2:])
            print(f'  reverse set to {reverse:.2f}%')
        elif cmd.startswith('n='):
            neutral = float(cmd[2:])
            print(f'  neutral set to {neutral:.2f}%')
        elif cmd.startswith('h='):
            high = float(cmd[2:])
            print(f'  high set to {high:.2f}%')
        else:
            try:
                current = float(cmd)
                pwm.ChangeDutyCycle(current)
                print(f'  -> {current:.2f}%')
            except ValueError:
                print('  unknown command')
except (KeyboardInterrupt, EOFError):
    pass

pwm.ChangeDutyCycle(neutral)
time.sleep(0.3)
pwm.stop()
GPIO.cleanup()
print('\ndone')
