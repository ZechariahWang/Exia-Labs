#!/usr/bin/env python3
#
# Test Phidgets steering motor (MotorPositionController via DCC1000 + HKT22 encoder).
# Moves the motor to +degrees, -degrees, then back to center.
#
# Usage:
#   python3 test_phidgets.py [hub_port] [degrees]
#
# Arguments:
#   hub_port  - VINT Hub port number (default: 0)
#   degrees   - target position in degrees for left/right sweep (default: 500.0)
#
# Examples:
#   python3 test_phidgets.py           # port 0, sweep +/-500 deg
#   python3 test_phidgets.py 1 2160    # port 1, sweep +/-2160 deg (full lock)
#
# Requires: Phidget22 library (pip install Phidget22)

import sys
import time
from Phidget22.Devices.MotorPositionController import MotorPositionController
from Phidget22.PhidgetException import PhidgetException

hub_port = int(sys.argv[1]) if len(sys.argv) > 1 else 0
degrees = float(sys.argv[2]) if len(sys.argv) > 2 else 500.0

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

m.setRescaleFactor(360.0 / (300 * 4 * 4.25))
m.setCurrentLimit(3.0)
m.setVelocityLimit(10000.0)
m.setAcceleration(50000.0)
m.setDeadBand(2.0)
m.setKp(400.0)
m.setKi(0.0)
m.setKd(150.0)
m.addPositionOffset(-m.getPosition())
m.setEngaged(True)

print(f'Attached! Testing position control at +/-{degrees} degrees')

print(f'  Left (+{degrees} deg)...')
m.setTargetPosition(degrees)
time.sleep(2)
print(f'  Position: {m.getPosition():.1f} deg')

print(f'  Right (-{degrees} deg)...')
m.setTargetPosition(-degrees)
time.sleep(2)
print(f'  Position: {m.getPosition():.1f} deg')

print('  Center (0 deg)...')
m.setTargetPosition(0.0)
time.sleep(1.5)
print(f'  Position: {m.getPosition():.1f} deg')

m.setEngaged(False)
m.close()
print('Done')
