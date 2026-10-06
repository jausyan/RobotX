#!/bin/bash
# Launch vision_geo node + RViz obstacle map viewer
# Usage:
#   ./launch_vision_geo.sh          (real drone, device camera)
#   ./launch_vision_geo.sh sim      (Gazebo simulation)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/vision_geo.rviz"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"     # team rule: every UAV node (MAVROS, Gazebo camera too) on domain 30
echo "[launch_vision_geo.sh] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

MODE="${1:-real}"

if [ "$MODE" = "sim" ]; then
  CONFIG="$WORKSPACE/install/vision_geo/share/vision_geo/config/vision_geo_sim.yaml"
  echo "[launch_vision_geo] Starting in SIMULATION mode"
else
  CONFIG="$WORKSPACE/install/vision_geo/share/vision_geo/config/vision_geo.yaml"
  echo "[launch_vision_geo] Starting in REAL DRONE mode"
fi

echo "[launch_vision_geo] Config: $CONFIG"
echo "[launch_vision_geo] RViz:   $RVIZ_CONFIG"
echo ""

# start vision_geo node in background
ros2 run vision_geo vision_geo \
  --ros-args --params-file "$CONFIG" &
NODE_PID=$!
echo "[launch_vision_geo] vision_geo node PID: $NODE_PID"

# small delay so node can start publishing before RViz connects
sleep 1

# start RViz
rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!
echo "[launch_vision_geo] RViz PID: $RVIZ_PID"

# wait for either process to exit, then kill both
wait -n $NODE_PID $RVIZ_PID
echo "[launch_vision_geo] Process exited, shutting down..."
kill $NODE_PID $RVIZ_PID 2>/dev/null
