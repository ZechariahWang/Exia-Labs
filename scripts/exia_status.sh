#!/bin/bash

echo "=== Exia Robot Service Status ==="
echo ""

if systemctl is-enabled exia-robot.service &>/dev/null; then
    echo "Autostart: ENABLED (will start on boot)"
else
    echo "Autostart: DISABLED (normal boot)"
fi

echo ""
echo "Current status:"
systemctl status exia-robot.service --no-pager 2>/dev/null || echo "Service not installed"

echo ""
echo "=== Devices ==="
for dev in /dev/lss_controller /dev/exia_radio /dev/exia_imu /dev/exia_gps; do
    if [ -e "$dev" ]; then
        target=$(readlink -f "$dev")
        echo "  $dev -> $target : OK"
    else
        echo "  $dev : NOT FOUND"
    fi
done

echo ""
echo "=== Phidgets (Steering) ==="
if lsusb 2>/dev/null | grep -qi phidget; then
    echo "  Phidgets VINT Hub: FOUND"
else
    echo "  Phidgets VINT Hub: NOT FOUND"
fi

echo ""
echo "=== Radio Keys ==="
if [ -f "$HOME/.exia/radio_private.pem" ] && [ -f "$HOME/.exia/radio_public.pem" ]; then
    echo "  Radio encryption keys: FOUND"
else
    echo "  Radio encryption keys: MISSING (run scripts/generate_radio_keys.sh)"
fi

echo ""
echo "=== ROS Nodes ==="
if command -v ros2 &>/dev/null; then
    ros2 node list 2>/dev/null || echo "  ROS2 not running or not sourced"
else
    echo "  ros2 command not found (source ROS environment first)"
fi

echo ""
echo "=== Recent Logs ==="
journalctl -u exia-robot -n 20 --no-pager 2>/dev/null || echo "No logs available"
