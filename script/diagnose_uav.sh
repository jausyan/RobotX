#!/bin/bash
# Check every link of the UAV data chain and show on which ROS domain each piece runs:
#   Gazebo camera -> vision -> buoy map ─┐
#   MAVROS (/rian) ──────────────────────┴-> uav_bridge -> /system/... (heartbeat, safe_passage)
# Run it while the sim / drone is up.
# Usage:
#   ./diagnose_uav.sh                       # checks domain 30 (UAV) and domain 0 (ROS default)
#   CAMERA=/my/camera/image_raw ./diagnose_uav.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
MAVROS_NS="${MAVROS_NS:-/rian}"
CAMERA="${CAMERA:-/iris_with_camera_2/camera/image_raw}"

has_node()  { echo "$NODES"  | grep -qE "$1" && echo "yes" || echo "--"; }
# a topic only counts when something PUBLISHES it (a subscriber alone also makes it appear in the list)
has_topic() {
  echo "$TOPICS" | grep -qE "^$1\$" || { echo "--"; return; }
  n=$(timeout 6 ros2 topic info --no-daemon "$1" 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ "${n:-0}" -gt 0 ] && echo "yes" || echo "--"
}
field()     { timeout 4 ros2 topic echo --once --no-daemon "$1" 2>/dev/null | grep -m1 -E "^ *$2:" | awk '{print $2}'; }

for D in 30 0; do
  export ROS_DOMAIN_ID=$D
  NODES="$(timeout 10 ros2 node list --no-daemon --spin-time 2 2>/dev/null)"
  TOPICS="$(timeout 10 ros2 topic list --no-daemon --spin-time 2 2>/dev/null)"
  echo "================ ROS_DOMAIN_ID=$D ================"
  printf "  %-26s %s\n" "Gazebo camera node"      "$(has_node camera_controller)"
  printf "  %-26s %s\n" "MAVROS ($MAVROS_NS)"      "$(has_node "^$MAVROS_NS/mavros")"
  printf "  %-26s %s\n" "vision_geo / vision_node" "$(has_node '/vision_geo_node|/vision_node')"
  printf "  %-26s %s\n" "control (drone_controller)" "$(has_node /drone_controller)"
  printf "  %-26s %s\n" "uav_bridge"               "$(has_node /uav_bridge)"

  if [ "$(has_topic "$MAVROS_NS/state")" = yes ]; then
    printf "  %-26s connected=%s mode=%s\n" "MAVROS state" "$(field $MAVROS_NS/state connected)" "$(field $MAVROS_NS/state mode)"
    printf "  %-26s lat=%s\n" "MAVROS GPS" "$(field $MAVROS_NS/global_position/global latitude)"
  fi
  [ "$(has_topic "$MAVROS_NS/state")" = yes ] || printf "  %-26s %s\n" "MAVROS data" "-- (nothing publishes $MAVROS_NS/state here)"
  if [ "$(has_topic "$CAMERA")" = yes ]; then
    printf "  %-26s %s\n" "camera image rate" "$(timeout 4 ros2 topic hz --no-daemon "$CAMERA" 2>/dev/null | grep -m1 'average rate' | awk '{print $3 " Hz"}')"
  fi
  [ "$(has_topic "$CAMERA")" = yes ] || printf "  %-26s %s\n" "camera images" "-- (nothing publishes $CAMERA here)"
  if [ "$(has_topic /vision_geo/map/buoy_red)" = yes ]; then
    for s in red green entry exit off; do
      n=$(timeout 4 ros2 topic echo --once --no-daemon /vision_geo/map/buoy_$s 2>/dev/null | grep -c latitude)
      printf "  %-26s %s\n" "buoy map: $s" "${n:-0}"
    done
  fi
  [ "$(has_topic /vision_geo/map/buoy_red)" = yes ] || printf "  %-26s %s\n" "buoy map" "-- (vision not publishing /vision_geo/map/* here)"
  if [ "$(has_topic /system/vehicle/uav/heartbeat)" = yes ]; then
    printf "  %-26s state=%s lat=%s phase=%s task=%s\n" "bridge heartbeat" \
      "$(field /system/vehicle/uav/heartbeat state)" "$(field /system/vehicle/uav/heartbeat latitude)" \
      "$(field /system/vehicle/uav/heartbeat flight_phase)" "$(field /system/vehicle/uav/heartbeat current_task)"
    printf "  %-26s %s\n" "safe_passage buoys" "$(timeout 4 ros2 topic echo --once --no-daemon /system/vehicle/uav/task1/safe_passage 2>/dev/null | grep -c 'state:')"
  fi
done

cat <<'EOF'

How to read it:
  * Everything must say "yes" under ROS_DOMAIN_ID=30 and "--" under 0.
    Anything still under 0 was started in a terminal without `export ROS_DOMAIN_ID=30`.
  * heartbeat state=1 (KILLED) / lat=0      -> the bridge does not see MAVROS on domain 30
  * camera on another domain than vision    -> vision gets no images, buoy map stays 0
  * buoy map > 0 but safe_passage buoys = 0 -> bridge and vision are on different domains
  * buoy map all 0                          -> vision not started yet (needs "UAV-GO") or nothing detected
EOF
