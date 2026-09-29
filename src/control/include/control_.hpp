#ifndef CONTROL___HPP
#define CONTROL___HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include "math_.hpp"
#include "geo_.hpp"
#include "drone_controller_.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <iostream>
#include <iomanip>
#include <limits>

#define RATE 10.0

bool payloadDetected(const std::shared_ptr<DroneController>&node);
bool emberDetected(const std::shared_ptr<DroneController>&node);
bool bunderDetected(const std::shared_ptr<DroneController>&node);
bool artagDetected(const std::shared_ptr<DroneController>&node);
    /**
     * Check if the payload is detected by verifying if its position is not at the origin (0,0,0).
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the payload is detected, false otherwise.
     */    

template<typename T>
bool hasReplied(const std::shared_ptr<DroneController>&node, T& result) {
    return rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS;
}
    /**
     * Check if the service call has been replied to.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - result: reference to the service call result. (promise or future)
     * 
     * returns:
     * - true if the service call was successful, false otherwise.
 */

void initFrame(const std::shared_ptr<DroneController>& node, geometry_msgs::msg::PoseStamped &pose);
    /**
     * Initialize the starting pose 'node' with the current position of the drone and reset payload pose.
     * This is used to reset posee or agg_pose (reference to where the last drone position when a control
     * function is called, it will be modified to the last drone position when the function returns.)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - pose: reference to the geometry_msgs::msg::PoseStamped object to be initialized.
     */
    

void setParam(const std::shared_ptr<DroneController>&node, const std::string &id, int integer_value);
    /**
     * Set a parameter on the drone controller node.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - id: string identifier for the parameter to be set. 
     * - integer_value: integer value to set for the parameter.
     */


void setMode(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, const std::string mode);

void takeoff(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt);
    /**
     * ArduPilot takeoff: set GUIDED, ask for confirmation, arm, send the takeoff command, wait for the altitude.
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - takeoff_alt: float value representing the desired takeoff altitude above home
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     */

void takeoff_no_confirm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt);
    /**
     * Initiate the takeoff procedure for the drone without having to wait for confirmation. Set mode to GUIDED, arm the drone, and send the takeoff command.
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - takeoff_alt: float value representing the desired takeoff altitude, this is based on the rangefinder reading.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - rel_alt: float value representing the relative altitude to be maintained at the correct rangefinder reading.
     */

// void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &target_pose, float max_time = 3.0);
    /**
     * Wait for the drone to stably hover at the target pose while constantly updating pose and heading to mitigate drift.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - target_pose: reference to the geometry_msgs::msg::PoseStamped object representing the target pose for stabilization.
     */

void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration = 3.0);


void waitLand(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);
    /**
     * Wait until the drone has landed by checking its altitude.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */
    

void land(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);
    /**
     * Initiate the landing procedure for the drone by sending a land command.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */

void safeLanding(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float descent_rate);
    /**
     * Perform controlled safe landing with gradual descent in GUIDED mode.
     * Much safer than using LAND mode which can be too fast and cause bouncing.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - posee: reference to current pose (will be updated during descent)
     * - descent_rate: descent speed in m/s (default 0.3 m/s for gentle landing)
     */

void Descend(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee);
    /**
     * Descend the drone to a lower altitude while checking if the payload is detected. Used to take the payload on the ground.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     */
    
void calibrateHoverOrientation(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float max_time, float hover_pitch, float hover_roll, bool calibrate_pitch = true, bool calibrate_roll = true);

void centeringPayload(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float x = 1.0, float y = 0.0, float min_center_time = 0.5, float max_center_pitch = 2.5, float max_center_roll = 2.5, float hover_pitch = 0.0, float hover_roll = 0.0, std::string recovery_method = "local_pose", std::string centering_setpoint_mode = "velocity");
    /**
     * Center the payload by adjusting the drone's position based on the payload's current position.
     * The drone will adjust its velocity to center the payload while maintaining a stable hover.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - speed_xy: float value representing the speed at which the drone should move in the XY plane.
     * - acc: tolerance value for determining if the payload is centered.
     * - maxAccel: float value representing the maximum acceleration allowed during centering.
     * - takeAlt: float value representing the altitude to descend to take the payload.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
      * - status: reference to a boolean indicating whether centering is successful or not. Modified by the function (true if successful, false otherwise)
      */

void centering_payload(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    float step,
    float offset = 0.0,
    float close_threshold = 0.03,
    float max_velocity = 0.35,
    float timeout_sec = 15.0,
    bool downward_camera = true,
    float min_velocity = 0.02);
    /**
     * Center payload based on /payload_pose topic data.
     * Expects x, y, and center_dist in pose.position.{x,y,z}.
     *
     * parameters:
     * - node: shared pointer to DroneController.
     * - rate: control loop rate.
     * - step: number of velocity steps (distance is divided by step).
     * - offset: offset bias applied to x and y error.
     * - close_threshold: centered threshold after center_dist / step.
     * - max_velocity: max velocity command magnitude for x/y.
     * - timeout_sec: timeout for centering.
     * - downward_camera: map camera x/y to body velocity for downward-facing camera.
     * - min_velocity: minimum non-zero velocity magnitude for precise step movement.
     */

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float angle, float speed, float tolerance);
    /**
     * Move the drone to a specified point in local coordinates with a given speed and angle.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - x: float value representing the target x-coordinate in local coordinates.
     * - y: float value representing the target y-coordinate in local coordinates.
     * - z: float value representing the target z-coordinate in local coordinates.
     * - angle: float value representing the angle to turn towards in radians.
     * - speed: float value representing the speed at which the drone should move.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, geometry_msgs::msg::Pose target, float speed, float tolerance, bool allow_centering_payload = false, bool allow_centering_ember = false, bool allow_centering_artag = false, bool disable_z_lock = false);

void landingFaux(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee);

void landingFauxPose(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee);
    /**
     * Descend the drone to a lower altitude using a faux pose method.
     * This method is used to land the drone by gradually decreasing its altitude.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    

void holdPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration);
    /**
     * Wait for a specified duration. In guided mode, the drone should hold the current position when doing nothing.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - duration: float value representing the duration in seconds to hold the position.
     */
    

void waitForPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, mavros_msgs::msg::GlobalPositionTarget raw, double tolerance = 0.00001, bool allow_centering = false, float center_dist = 5.0);
    /**
     * Wait until the drone reaches a specified global position by checking the distance to the target position. Used when the drone moves using waypoint in AUTO mode.
     * (Although its probably better to listen to mavlink's current_wp). see: https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - raw: mavros_msgs::msg::GlobalPositionTarget object containing the target global position.
     * - tolerance: double value representing the tolerance for reaching the target position in degrees.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position. (Not used for now)
     */
    

void sendGlobalRaw(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering = false, float center_dist = 5.0, float tolerance = 0.0000025);
    /**
     * Send a raw global position target to the drone and wait until the drone reaches the target position.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - latt: float value representing the target latitude in degrees.
     * - longg: float value representing the target longitude in degrees.
     * - altt: float value representing the target altitude in meters.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */

void sendGlobalRawAsync(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering, float center_dist);

void sendGlobalRawFromXYZ(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, LatLonAlt zero, double zero_hdg, float x, float y, float rel_alt, bool allow_centerin = false, float center_dist = 5.0);

void correctAltitude(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, double height, float& corrected_rel_alt, double max_speed = 0.25);
    /**
     * Correct the drone's altitude to a specified rangefinder reading by adjusting its velocity.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - vel_pub: shared pointer to the rclcpp::Publisher for publishing velocity commands.
     * - height: double value representing the target altitude in meters.
     * 
     * returns:
     * - corrected_rel_alt: The correct relative altitude after adjusting the drone's position. ready to be used for sendGlobalRaw.
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */

void controlServo(const std::shared_ptr<DroneController>&node, int channel, int pwm);

void controlServoRepeated(const std::shared_ptr<DroneController>& node, int channel, int pwm, int repeat = 3);

void pushMission(const std::shared_ptr<DroneController>&node, std::vector<mavros_msgs::msg::Waypoint> waypoints);

void clearMission(const std::shared_ptr<DroneController>&node);

void waitForWP(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, int seq);

void arm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);

void disarm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);

void moveWithLaser(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float max_speed, float tolerance, bool allow_centering_payload = false, bool allow_centering_ember = false, float koreksi_max_speed = 1.0, float timeout = -1.0);

void holdAtLidarRanges(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float front, float left, float back, float right, bool allow_centering_payload = false, bool allow_centering_ember = false);

float getYawError(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);

void printTitik(std::vector<double>& lat_indoor, std::vector<double>& lon_indoor, std::vector<double>& lat_outdoor, std::vector<double>& lon_outdlat_outdoor, double lat_takeoff, double lon_takeoff);

void LocalMove(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, float up_z, float yaw_angle, float tolerance, bool auto_heading = true);
    /**
     * Execute a local waypoint movement by publishing position commands to /mavros/local_position/pose
     * Movement is relative to the current drone position in the local frame
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - forward_x: distance to move forward (positive) or backward (negative) in meters
     * - left_y: distance to move left (positive) or right (negative) in meters
     * - up_z: distance to move up (positive) or down (negative) in meters
     * - yaw_angle: target yaw angle in radians (0 = no change, use current heading)
     * - tolerance: distance tolerance to consider waypoint reached in meters
     * - auto_heading: if true (default), automatically rotate the drone to face the
     *                 nearest cardinal axis (+x / -x / +y / -y) that best matches
     *                 the requested movement direction.  If false, the yaw is kept
     *                 at its current value (no heading change during the move).
     */

void Rotate(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float rotation_angle);
    /**
     * Execute a rotation in place by publishing position commands with updated orientation
     * Rotation is relative to current heading
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final orientation
    * - rotation_angle: angle to rotate in RADIANS (positive = right/clockwise, negative = left/counter-clockwise)
     */

void rotateByDegrees(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float degrees);
    /**
     * Execute a rotation in place - EASIER VERSION with degrees instead of radians
     * Rotation is relative to current heading
     * 
     * USAGE EXAMPLES:
     * - rotateByDegrees(node, rate, posee, 90.0);   // Rotate 90° RIGHT (clockwise)
     * - rotateByDegrees(node, rate, posee, -90.0);  // Rotate 90° LEFT (counter-clockwise)
     * - rotateByDegrees(node, rate, posee, 180.0);  // Turn around (U-turn)
     * - rotateByDegrees(node, rate, posee, 45.0);   // Rotate 45° RIGHT
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final orientation
    * - degrees: angle to rotate in DEGREES (positive = RIGHT/clockwise, negative = LEFT/counter-clockwise)
     */

void fix_alt(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float req_alt, float tolerance = 0.1, float timeout = 15.0);

void centering_red(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    float speed_xy,
    bool &status,
    float acc,
    float maxAccel,
    float x = 1.0,
    float y = 0.0,
    float min_center_time = 0.01,
    float max_center_pitch = 2.5,
    float max_center_roll = 2.5,
    float hover_pitch = 0.0,
    float hover_roll = 0.0,
    std::string recovery_method = "local_pose",
    float centering_tolerance = 0.20,
    std::string topic = "/vision/red");

struct MissionOrder {
    std::string tin;     // 1st color, lowercase: which tin to drop  ("red"/"green"/"blue", empty if not sent)
    std::string circle;  // 2nd color, lowercase: which circle to drop on (matches vision_geo "circle_<color>")
};

void pubCommand(const std::shared_ptr<DroneController>&node, const std::string &command_text, const std::string &topic = "/mission/order");
    /**
     * Publish a command string (e.g. "UAV-GO") on the mission topic. One publisher per topic is created
     * on the first call and reused afterwards.
     */

bool waitCommand(const std::shared_ptr<DroneController>&node, const std::string &expected_command = "UAV-GO", const std::string &topic = "/mission/order", float timeout = 60.0, bool hold_position = true, std::string *received = nullptr);
    /**
     * Wait for expected_command on the mission topic while holding the current position.
     * Only listens while called. A message matches when it starts with expected_command (case-insensitive),
     * so "UAV-GO:RED:BLUE" matches "UAV-GO"; anything else (e.g. "UAV-HOLD") is logged and ignored.
     *
     * parameters:
     * - timeout: seconds before giving up.
     * - hold_position: publish a hold setpoint at the pose captured when the wait started.
     * - received: optional, filled with the full received command (uppercase), e.g. "UAV-GO:RED:BLUE".
     *
     * returns:
     * - true when the command arrived, false on timeout.
     */

MissionOrder parseCommand(const std::string &command);
    /**
     * Split "UAV-GO:RED:BLUE" into tin = "red", circle = "blue" (empty if not present).
     */

void goToVehicle(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float alt, const std::string &topic = "/USV/global_position/global");
    /**
     * Capture the USV position once from topic, fly there with pushMission() + AUTO at alt,
     * then switch back to GUIDED and hold.
     */

#endif // CONTROL__HPP
