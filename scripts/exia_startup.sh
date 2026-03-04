#!/bin/bash
# Exia Ground Robot — Startup Script
# Hardware: LSS servo bus (throttle/brake), Phidgets DC motor (steering),
#           Jetson GPIO PWM (gear), RFD900x radio, sensors via hw_teleop
# Paths hardcoded for user `exialabsargus` on Jetson Orin

LOGFILE="/home/exialabsargus/exia_startup.log"
LSS_PORT="/dev/lss_controller"
RADIO_PORT="/dev/exia_radio"
IMU_PORT="/dev/exia_imu"
GPS_PORT="/dev/exia_gps"
WORKSPACE="/home/exialabsargus/exia_ws"
KEY_DIR="/home/exialabsargus/.exia"

PIDS=()

cleanup() {
    echo "Shutting down all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    pkill -f "ros2.*exia" 2>/dev/null
    pkill -f "foxglove_bridge" 2>/dev/null
    pkill -f "velodyne" 2>/dev/null
    pkill -f "orbbec_camera" 2>/dev/null
    pkill -f "septentrio_gnss" 2>/dev/null
    wait 2>/dev/null
    echo "Shutdown complete at $(date)"
}

trap cleanup EXIT INT TERM

truncate -s 0 "$LOGFILE" 2>/dev/null
exec > >(tee -a "$LOGFILE") 2>&1
echo "=========================================="
echo "Exia Robot Startup - $(date)"
echo "=========================================="

check_device() {
    local device=$1
    if [ -e "$device" ]; then
        echo "$device : FOUND"
    else
        echo "$device : not present — node will connect when available"
    fi
}

echo "=== Device Check ==="
check_device "$LSS_PORT"
check_device "$RADIO_PORT"
check_device "$IMU_PORT"
check_device "$GPS_PORT"
echo "===================="

echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID=0

if [ ! -f "$KEY_DIR/radio_private.pem" ] || [ ! -f "$KEY_DIR/radio_public.pem" ]; then
    echo "WARNING: Radio encryption keys not found in $KEY_DIR"
    echo "Run: bash $WORKSPACE/scripts/generate_radio_keys.sh"
fi

echo "Starting hardware teleop..."
export JETSON_MODEL_NAME=JETSON_ORIN_NANO
ros2 run exia_control hw_teleop &
PIDS+=($!)

sleep 3

echo "Starting radio bridge (robot)..."
ros2 launch exia_bringup radio.launch.py role:=robot serial_port:="$RADIO_PORT" &
PIDS+=($!)

echo "All processes launched, waiting..."
wait -n 2>/dev/null || wait
echo "A process exited at $(date), shutting down..."
