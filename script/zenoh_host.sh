#!/bin/bash
# Zenoh bridge on the HOST: connects to the drone bridge and republishes its topics locally.
# Usage:
#   ./zenoh_host.sh                 (drone at 192.168.0.100)
#   ./zenoh_host.sh 100.88.205.115  (drone over tailscale)
# Start ~/zenoh/run_zenoh.sh on the drone first.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG="$WORKSPACE/config/zenoh_host.json5"
DRONE_IP="${1:-192.168.0.100}"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
echo "[zenoh_host.sh] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  drone=tcp/$DRONE_IP:7447"

exec zenoh-bridge-ros2dds -c "$CONFIG" -e "tcp/$DRONE_IP:7447"
