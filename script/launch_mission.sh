#!/bin/bash
# Launch the UAV mission (control node).
# Config is read from the workspace config/config.yaml — edit and rerun, no rebuild needed.
# Usage:
#   ./launch_mission.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG="$WORKSPACE/config/config.yaml"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "[launch_mission] Config: $CONFIG"
ros2 run control control --ros-args --params-file "$CONFIG"
