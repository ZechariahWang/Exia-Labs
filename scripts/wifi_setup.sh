#!/usr/bin/env bash

ROLE="${1:-}"
PEER_IP="${2:-}"

if [ -z "$ROLE" ] || [ -z "$PEER_IP" ]; then
    echo "Usage: source wifi_setup.sh <role> <peer_ip>"
    echo "  role:    base | robot"
    echo "  peer_ip: IP address of the other machine"
    echo ""
    echo "Examples:"
    echo "  source wifi_setup.sh base 192.168.1.100   # on base laptop, robot is .100"
    echo "  source wifi_setup.sh robot 192.168.1.50   # on robot Jetson, base is .50"
    return 1 2>/dev/null || exit 1
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

CYCLONE_CFG=$(mktemp /tmp/cyclonedds_${ROLE}_XXXXXX.xml)
cat > "$CYCLONE_CFG" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="${PEER_IP}"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI="file://${CYCLONE_CFG}"

echo "WiFi DDS configured:"
echo "  Role:        ${ROLE}"
echo "  Peer IP:     ${PEER_IP}"
echo "  RMW:         ${RMW_IMPLEMENTATION}"
echo "  Domain ID:   ${ROS_DOMAIN_ID}"
echo "  CycloneDDS:  ${CYCLONE_CFG}"
echo ""
echo "Verify with: ros2 topic list  (should see topics from peer)"
