#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="$SCRIPT_DIR/exia-robot.service"

echo "Installing Exia Robot autostart service..."

if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo. Re-running with sudo..."
    exec sudo "$0" "$@"
fi

chmod +x "$SCRIPT_DIR/exia_startup.sh"

cp "$SERVICE_FILE" /etc/systemd/system/exia-robot.service

systemctl daemon-reload

systemctl enable exia-robot.service

echo ""
echo "Autostart ENABLED"
echo ""
echo "Commands:"
echo "  Start now:     sudo systemctl start exia-robot"
echo "  Stop:          sudo systemctl stop exia-robot"
echo "  Status:        sudo systemctl status exia-robot"
echo "  View logs:     journalctl -u exia-robot -f"
echo "  Disable:       ~/Exia-Labs/scripts/exia_disable_autostart.sh"
echo ""
echo "The robot will start automatically on next boot."
