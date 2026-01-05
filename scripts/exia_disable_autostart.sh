#!/bin/bash

echo "Disabling Exia Robot autostart service..."

if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo. Re-running with sudo..."
    exec sudo "$0" "$@"
fi

systemctl stop exia-robot.service 2>/dev/null

systemctl disable exia-robot.service

echo ""
echo "Autostart DISABLED"
echo ""
echo "The Jetson will boot normally without starting the robot."
echo "To re-enable: ~/Exia-Labs/scripts/exia_enable_autostart.sh"
