#!/bin/bash

LOGFILE="/home/exialabsargus/exia_startup.log"
ARDUINO_PORT="/dev/arduino_control"
ARDUINO_SKETCH_DIR="/home/exialabsargus/rc_control_pwm"
WORKSPACE="/home/exialabsargus/Exia-Labs"
MAX_WAIT=60

exec > >(tee -a "$LOGFILE") 2>&1
echo "=========================================="
echo "Exia Robot Startup - $(date)"
echo "=========================================="

wait_for_device() {
    local device=$1
    local waited=0
    echo "Waiting for $device..."
    while [ ! -e "$device" ] && [ $waited -lt $MAX_WAIT ]; do
        sleep 1
        waited=$((waited + 1))
    done
    if [ -e "$device" ]; then
        echo "$device found after ${waited}s"
        return 0
    else
        echo "ERROR: $device not found after ${MAX_WAIT}s"
        return 1
    fi
}

if ! wait_for_device "$ARDUINO_PORT"; then
    echo "Arduino not detected. Exiting."
    exit 1
fi

sleep 2

echo "Uploading Arduino firmware..."
cd "$ARDUINO_SKETCH_DIR"
if arduino-cli upload --fqbn arduino:avr:mega -p "$ARDUINO_PORT" .; then
    echo "Arduino upload successful"
else
    echo "ERROR: Arduino upload failed"
    exit 1
fi

sleep 3

echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID=0

echo "Starting rc_driver_control_node..."
ros2 run exia_ground_description rc_driver_control_node.py

echo "Node exited at $(date)"
