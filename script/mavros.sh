PORT="/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_2D004C001151323530363730-if00"
CONFIG="$HOME/RobotX/config/mavros_param.yaml"
MAVROS_LAUNCH="/opt/ros/jazzy/share/mavros/launch"

source /opt/ros/jazzy/setup.bash
# export ROS_DOMAIN_ID=30

sudo chmod 777 "$PORT"

ros2 run mavros mavros_node --ros-args \
  --params-file "$MAVROS_LAUNCH/apm_pluginlists.yaml" \
  --params-file "$MAVROS_LAUNCH/apm_config.yaml" \
  --params-file "$CONFIG" \
  -r __ns:=/rian
