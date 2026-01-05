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
echo "=== Recent Logs ==="
journalctl -u exia-robot -n 20 --no-pager 2>/dev/null || echo "No logs available"
