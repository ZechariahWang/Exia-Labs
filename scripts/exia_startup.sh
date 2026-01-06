#!/bin/bash

LOGFILE="/home/exialabsargus/exia_startup.log"
ARDUINO_PORT="/dev/arduino_control"
ARDUINO_CLI="/home/exialabsargus/bin/arduino-cli"
ARDUINO_SKETCH_DIR="/home/exialabsargus/rc_control_pwm"
WORKSPACE="/home/exialabsargus/Exia-Labs"
MAX_WAIT=120

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

wait_for_odrive() {
    local waited=0
    echo "Waiting for ODrive USB device..."
    while [ $waited -lt $MAX_WAIT ]; do
        if lsusb | grep -q "1209:0d32"; then
            echo "ODrive USB found after ${waited}s"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "WARNING: ODrive USB not found after ${MAX_WAIT}s (will retry in node)"
    return 1
}

reset_arduino() {
    echo "Resetting Arduino via DTR..."
    stty -F "$ARDUINO_PORT" 115200 hupcl -echo
    exec 3<>"$ARDUINO_PORT"
    sleep 0.25
    exec 3>&-
    sleep 3
    echo "Arduino reset complete"
}

wait_for_arduino_ready() {
    local waited=0
    local max_wait=30
    echo "Waiting for Arduino READY message..."
    stty -F "$ARDUINO_PORT" 115200 raw -echo
    while [ $waited -lt $max_wait ]; do
        if timeout 2 cat "$ARDUINO_PORT" 2>/dev/null | head -c 50 | grep -q "READY"; then
            echo "Arduino READY after ${waited}s"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "WARNING: No READY from Arduino after ${max_wait}s"
    return 1
}

if ! wait_for_device "$ARDUINO_PORT"; then
    echo "Arduino not detected. Exiting."
    exit 1
fi

sleep 2

reset_arduino
wait_for_arduino_ready

wait_for_odrive

sleep 2
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID=0

echo "Starting rc_driver_control_node..."
ros2 run exia_ground_description rc_driver_control_node.py

echo "Node exited at $(date)"
