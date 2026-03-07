#!/usr/bin/env python3
#
# Interactively sweep the gear shift servo via Jetson GPIO PWM.
# Lets you enter arbitrary duty cycle percentages or use preset shortcuts.
#
# Usage:
#   python3 sweep_gear_servo.py [pin]
#
# Arguments:
#   pin  - Jetson BOARD pin number (default: 32)
#
# Interactive commands:
#   <number>  - set duty cycle directly (e.g. 7.5)
#   r         - reverse preset (5.67%)
#   n         - neutral preset (6.94%)
#   h         - high gear preset (8.61%)
#   q         - quit
#
# Safe range is 2-13%. Values outside this range are rejected.
#
# Requires: Jetson.GPIO (run on Jetson only)
import sys
import time

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print('Jetson.GPIO not available — run this on the Jetson')
    sys.exit(1)

PIN = int(sys.argv[1]) if len(sys.argv) > 1 else 32
FREQ = 50

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT)
pwm = GPIO.PWM(PIN, FREQ)
pwm.start(7.5)

print(f'Gear servo sweep — pin {PIN}, {FREQ}Hz')
print(f'Duty cycle range: 2.5% (500us) to 12.5% (2500us)')
print(f'Enter a duty cycle %, or one of: r=5.67 n=6.94 h=8.61')
print(f'Type q to quit\n')

shortcuts = {'r': 5.67, 'n': 6.94, 'h': 8.61}

try:
    while True:
        val = input('duty% > ').strip().lower()
        if val == 'q':
            break
        if val in shortcuts:
            duty = shortcuts[val]
        else:
            try:
                duty = float(val)
            except ValueError:
                print('  invalid — enter a number or r/n/h')
                continue
        if duty < 2.0 or duty > 13.0:
            print(f'  {duty}% is outside safe range (2-13), skipping')
            continue
        pwm.ChangeDutyCycle(duty)
        us = duty / 100.0 * (1000000.0 / FREQ)
        print(f'  -> {duty:.1f}% ({us:.0f}us)')
except KeyboardInterrupt:
    pass

pwm.stop()
GPIO.cleanup()
print('Done.')
