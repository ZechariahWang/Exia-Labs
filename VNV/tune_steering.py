#!/usr/bin/env python3

import sys
import time
import threading
from Phidget22.Devices.MotorPositionController import MotorPositionController
from Phidget22.PhidgetException import PhidgetException

hub_port = int(sys.argv[1]) if len(sys.argv) > 1 else 0

m = MotorPositionController()
m.setHubPort(hub_port)
m.setIsHubPortDevice(False)

try:
    print('Waiting for attachment...')
    m.openWaitForAttachment(5000)
except PhidgetException as e:
    print(f'Failed to attach: {e}')
    print(f'Check: is the VINT Hub plugged in? Is the controller on port {hub_port}?')
    sys.exit(1)

rescale = 360.0 / (300 * 4 * 4.25)
m.setRescaleFactor(rescale)
m.setCurrentLimit(15)
m.setVelocityLimit(10000.0)
m.setAcceleration(50000.0)
m.setDeadBand(2.0)
m.setKp(400.0)
m.setKi(0.0)
m.setKd(150.0)
m.addPositionOffset(-m.getPosition())
m.setEngaged(True)

kp = 400.0
ki = 0.0
kd = 150.0
vel_limit = 10000.0
accel = 50000.0
current_limit = 15.0
dead_band = 2.0
target_pos = 0.0
running = True
logging = False
log_interval = 0.05

print(f'Attached on port {hub_port}')
print(f'Rescale factor: {rescale:.6f}')
print(f'Initial position: {m.getPosition():.1f} deg')
print()


def log_loop():
    while running:
        if logging:
            try:
                pos = m.getPosition()
                duty = m.getDutyCycle()
                vel = m.getVelocity()
                err = target_pos - pos
                print(f'  pos={pos:+8.1f}  target={target_pos:+8.1f}  err={err:+8.1f}  duty={duty:+.3f}  vel={vel:+8.1f}')
            except Exception:
                pass
        time.sleep(log_interval)


log_thread = threading.Thread(target=log_loop, daemon=True)
log_thread.start()


def print_status():
    try:
        pos = m.getPosition()
        duty = m.getDutyCycle()
        vel = m.getVelocity()
        err = target_pos - pos
        print(f'\n  pos={pos:+8.1f}  target={target_pos:+8.1f}  err={err:+8.1f}  duty={duty:+.3f}  vel={vel:+8.1f}')
        print(f'  kp={kp:.1f}  ki={ki:.2f}  kd={kd:.1f}  vel_limit={vel_limit:.0f}  accel={accel:.0f}  current={current_limit:.1f}A  deadband={dead_band:.1f}')
    except Exception as e:
        print(f'  Error reading status: {e}')


def print_help():
    print('''
Commands:
  p <value>     Set kp (proportional gain)
  i <value>     Set ki (integral gain)
  d <value>     Set kd (derivative gain)
  v <value>     Set velocity limit (deg/s)
  a <value>     Set acceleration (deg/s^2)
  c <value>     Set current limit (amps)
  b <value>     Set dead band (degrees)
  g <degrees>   Go to position (degrees)
  sweep <deg>   Sweep +deg, -deg, center with logging
  step <deg>    Step to +deg with logging, then back to 0
  zero          Re-zero encoder at current position
  log           Toggle continuous logging
  rate <sec>    Set log interval (default 0.05)
  s             Print current status
  r <factor>    Set rescale factor (360 / (CPR * 4 * gear))
  h             Show this help
  q             Quit
''')


print_help()

while True:
    try:
        raw = input('> ').strip()
    except (EOFError, KeyboardInterrupt):
        break

    if not raw:
        continue

    parts = raw.split()
    cmd = parts[0].lower()

    try:
        if cmd == 'q':
            break

        elif cmd == 'h':
            print_help()

        elif cmd == 's':
            print_status()

        elif cmd == 'p' and len(parts) > 1:
            kp = float(parts[1])
            m.setKp(kp)
            print(f'  kp = {kp}')

        elif cmd == 'i' and len(parts) > 1:
            ki = float(parts[1])
            m.setKi(ki)
            print(f'  ki = {ki}')

        elif cmd == 'd' and len(parts) > 1:
            kd = float(parts[1])
            m.setKd(kd)
            print(f'  kd = {kd}')

        elif cmd == 'v' and len(parts) > 1:
            vel_limit = float(parts[1])
            m.setVelocityLimit(vel_limit)
            print(f'  vel_limit = {vel_limit}')

        elif cmd == 'a' and len(parts) > 1:
            accel = float(parts[1])
            m.setAcceleration(accel)
            print(f'  accel = {accel}')

        elif cmd == 'c' and len(parts) > 1:
            current_limit = float(parts[1])
            m.setCurrentLimit(current_limit)
            print(f'  current_limit = {current_limit}A')

        elif cmd == 'b' and len(parts) > 1:
            dead_band = float(parts[1])
            m.setDeadBand(dead_band)
            print(f'  dead_band = {dead_band}')

        elif cmd == 'g' and len(parts) > 1:
            target_pos = float(parts[1])
            m.setTargetPosition(target_pos)
            print(f'  -> target = {target_pos}')
            time.sleep(0.5)
            print_status()

        elif cmd == 'zero':
            m.addPositionOffset(-m.getPosition())
            target_pos = 0.0
            m.setTargetPosition(0.0)
            print('  Encoder zeroed at current position')

        elif cmd == 'log':
            logging = not logging
            print(f'  Logging {"ON" if logging else "OFF"} (interval={log_interval:.3f}s)')

        elif cmd == 'rate' and len(parts) > 1:
            log_interval = float(parts[1])
            print(f'  Log interval = {log_interval:.3f}s')

        elif cmd == 'r' and len(parts) > 1:
            rescale = float(parts[1])
            m.setEngaged(False)
            m.setRescaleFactor(rescale)
            m.addPositionOffset(-m.getPosition())
            m.setEngaged(True)
            target_pos = 0.0
            print(f'  Rescale factor = {rescale:.6f} (re-zeroed)')

        elif cmd == 'sweep' and len(parts) > 1:
            deg = float(parts[1])
            logging = True
            print(f'  Sweep +/-{deg} deg')

            print(f'  -> +{deg}')
            target_pos = deg
            m.setTargetPosition(deg)
            time.sleep(2)

            print(f'  -> -{deg}')
            target_pos = -deg
            m.setTargetPosition(-deg)
            time.sleep(2)

            print(f'  -> 0')
            target_pos = 0.0
            m.setTargetPosition(0.0)
            time.sleep(1.5)

            logging = False
            print_status()

        elif cmd == 'step' and len(parts) > 1:
            deg = float(parts[1])
            logging = True
            print(f'  Step to {deg} deg')

            target_pos = deg
            m.setTargetPosition(deg)
            time.sleep(2)

            print(f'  Step back to 0')
            target_pos = 0.0
            m.setTargetPosition(0.0)
            time.sleep(2)

            logging = False
            print_status()

        else:
            print(f'  Unknown command: {raw} (type h for help)')

    except PhidgetException as e:
        print(f'  Phidget error: {e}')
    except ValueError as e:
        print(f'  Invalid value: {e}')

running = False
print('\nShutting down...')
m.setTargetPosition(0.0)
time.sleep(0.5)
m.setEngaged(False)
m.close()
print('Done')
