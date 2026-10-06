#!/bin/bash
# Check the UAV's /system/... topics against the team interface (RobotX-SoS-communication-ITSN README §3–4).
# Run while uav_bridge is running.
# Usage:
#   ./check_system_topics.sh            # domain 30 (UAV)
#   ./check_system_topics.sh 10         # e.g. on the GCS domain, to see what the DDS Router forwards

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"          # needed to decode rx_msgs
export ROS_DOMAIN_ID="${1:-${ROS_DOMAIN_ID:-30}}"

# topic | type | direction (as seen from the UAV)
EXPECTED="
/system/mission/command|rx_msgs/msg/Command|subscribe
/system/mission/course|rx_msgs/msg/Course|subscribe
/system/mission/status|rx_msgs/msg/MissionStatus|publish
/system/vehicle/uav/heartbeat|rx_msgs/msg/Heartbeat|publish
/system/vehicle/uav/task1/safe_passage|rx_msgs/msg/SafePassage|publish
/system/vehicle/uav/task2/delivery|rx_msgs/msg/ResourceDelivery|publish
/system/vehicle/uav/task3/delivery|rx_msgs/msg/ResourceDelivery|publish
/system/vehicle/usv/heartbeat|rx_msgs/msg/Heartbeat|subscribe
"

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
printf "%-42s %-30s %-10s %s\n" TOPIC TYPE UAV RESULT
LIST="$(ros2 topic list -t 2>/dev/null)"
echo "$EXPECTED" | while IFS='|' read -r topic type dir; do
  [ -z "$topic" ] && continue
  found="$(echo "$LIST" | grep -E "^$topic \[" | sed 's/.*\[\(.*\)\]/\1/')"
  if [ -z "$found" ]; then
    result="MISSING"
  elif [ "$found" != "$type" ]; then
    result="WRONG TYPE ($found)"
  else
    result="OK"
  fi
  printf "%-42s %-30s %-10s %s\n" "$topic" "$type" "$dir" "$result"
done

if echo "$LIST" | grep -q "^/system/vehicle/uav/heartbeat "; then
  echo; echo "Heartbeat rate (team rule: 2 Hz):"
  timeout 4 ros2 topic hz /system/vehicle/uav/heartbeat 2>/dev/null | grep -m1 "average rate"
fi
