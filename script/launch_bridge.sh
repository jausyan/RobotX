#!/bin/bash
# Launch the UAV <-> GCS bridge (team communication, rx_msgs).
# Every UAV node (MAVROS, control, vision, bridge) must run on the UAV domain.
# Usage:
#   ./launch_bridge.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG="$WORKSPACE/src/uav_bridge/config/uav_bridge.yaml"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"     # team rule: UAV = domain 30

echo "[launch_bridge] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  Config: $CONFIG"
ros2 run uav_bridge uav_bridge --ros-args --params-file "$CONFIG"
