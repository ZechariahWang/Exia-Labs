#!/usr/bin/env python3
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
