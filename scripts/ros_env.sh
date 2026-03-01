#!/usr/bin/env bash
# Source this file to standardize ROS environment across terminals:
#   source ~/exia_ws/scripts/ros_env.sh
# Optional domain override:
#   source ~/exia_ws/scripts/ros_env.sh 0

_exia_ws_default="$HOME/exia_ws"
_exia_ws="${EXIA_WS:-$_exia_ws_default}"
_ros_domain="${1:-0}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found"
  return 1 2>/dev/null || exit 1
fi

if [[ ! -f "$_exia_ws/install/setup.bash" ]]; then
  echo "ERROR: $_exia_ws/install/setup.bash not found"
  echo "Build first: cd $_exia_ws && colcon build"
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash
source "$_exia_ws/install/setup.bash"

# Keep discovery consistent across shells.
export ROS_DOMAIN_ID="$_ros_domain"
unset ROS_LOCALHOST_ONLY
unset RMW_IMPLEMENTATION
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset ROS_STATIC_PEERS

echo "ROS env ready:"
echo "  WS=$_exia_ws"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo
echo "If discovery is stale in this terminal:"
echo "  ros2 daemon stop && ros2 daemon start"
