#ifndef DRONE_CONTROLLER_HPP
#define DRONE_CONTROLLER_HPP

/**
 * Alooooo, jadi kan bakalan banyak program code yang beda beda, ntah itu buat testing, buat misi, buat misi 
 * parsial (misal cmn indoor, cmn outdoor, cmn payload, dll), dan semuanya itu butuh node yang interface nya
 * sama, jadi tujuannya class ini dipisah itu biar bisa dipake di semua testing atau misi (do not repeat yourself) 
 * dan juga biar bisa di extend, misal buat testing indoor, outdoor, payload, dll.
 *
 * Node ini bergungsi sebagai interface ke mavros, supaya semua fungsi kontrol gaperlu banyak parameter yang
 * sintaks nya sering lupa (misal reference ke shared pointer ke suatu publisher/client, rate, dll), cukup
 * referensi shared pointer ke node ini buat akses publisher, client, dan subscriber serta data data global,
 * jadi nanti di program testing kalo mau manggil fungsi (misal takeoff) ya cuman perlu parameter node ini sama
 * parameter yg relevan.
 *
 * TODO (Raziq): Bikin behavior tree parser biar ga banyak program testing, tapi banyak xml testing, jadi ga
 * perlu banyak ngebuild ulang kalo mau ubah ubah perilaku drone pas testing.
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include "math_.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>


class DroneController : public rclcpp::Node {
public:
    DroneController();

    // Get fucntions to retrieve current state and pose
    mavros_msgs::msg::State getCurrentState();
    mavros_msgs::msg::GPSRAW getCurrentGPSState();
    geometry_msgs::msg::PoseStamped getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped getCurrentGatePose();
    geometry_msgs::msg::PoseStamped getCurrentPoseArTag();
    geometry_msgs::msg::PoseStamped getCurrentPosePayload();
    geometry_msgs::msg::PoseStamped getCurrentPoseEmber();
    geometry_msgs::msg::PoseStamped getCurrentPoseBunder();
    geometry_msgs::msg::PoseStamped getLastNonZeroPoseArTag();
    geometry_msgs::msg::PoseStamped getLastNonZeroPosePayload();
    geometry_msgs::msg::PoseStamped getLastNonZeroPoseEmber();
    geometry_msgs::msg::PoseStamped getLastNonZeroPoseBunder();
    geometry_msgs::msg::PoseStamped getDroneNonZeroPoseArTag();
    geometry_msgs::msg::PoseStamped getDroneNonZeroPosePayload();
    geometry_msgs::msg::PoseStamped getDroneNonZeroPoseEmber();
    geometry_msgs::msg::PoseStamped getDroneNonZeroPoseBunder();
    geometry_msgs::msg::Point getCurrentEuler();
    geometry_msgs::msg::TwistStamped getCurrentVelocity();
    geometry_msgs::msg::PoseStamped getLastArTagPose();
    geometry_msgs::msg::PoseStamped getLastPayloadPose();
    geometry_msgs::msg::PoseStamped getLastEmberPose();
    geometry_msgs::msg::PoseStamped getLastBunderPose();
    geometry_msgs::msg::AccelWithCovarianceStamped getCurrentAccel();
    sensor_msgs::msg::NavSatFix getCurrentGPSPosition();
    geometry_msgs::msg::PoseStamped getInitDronePose();
    std_msgs::msg::Float64 getCurrentCompassHdg();
    std_msgs::msg::Float64 getCurrentRelAlt();
    sensor_msgs::msg::Range getCurrentRangefinder();
    sensor_msgs::msg::LaserScan getCurrentLaserScan();

    float getObstacleDistance(float angle, float angle_tolerance);

    // Set functions to update current state and pose
    void setLastArTagPose(const geometry_msgs::msg::PoseStamped &pose);
    void setLastPayloadPose(const geometry_msgs::msg::PoseStamped &pose);
    void setLastEmberPose(const geometry_msgs::msg::PoseStamped &pose);
    void setLastBunderPose(const geometry_msgs::msg::PoseStamped &pose);
    void setInitDronePose(const geometry_msgs::msg::PoseStamped &pose);
    void resetNonzeroArTagPose();
    void resetNonzeroPayloadPose();
    void resetNonzeroEmberPose();
    void resetNonzeroBunderPose();

    // Functions to wait for services to be available
    void waitForTakeoffService();
    void waitForLandService();
    void waitForArmingService();
    void waitForSetModeService();
    void waitForServoService();
    void waitForCloseCamService();
    void waitForOpenCamService();
    void waitForSetParamService();
    void waitForPushMissionService();
    void waitForClearMissionService();
    
    // Functions to check if services are ready
    bool isTakeoffServiceReady();
    bool isLandServiceReady();
    bool isArmingServiceReady();
    bool isSetModeServiceReady();
    bool isServoServiceReady();
    bool isCloseCamServiceReady();
    bool isOpenCamServiceReady();
    bool isSetParamServiceReady();
    bool isPushMissionServiceReady();
    bool isClearMissionServiceReady();

    // Functions to call services, returns a future that can be waited on
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_(const std::shared_ptr<mavros_msgs::srv::CommandTOL::Request>& request);
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_(const std::shared_ptr<mavros_msgs::srv::CommandTOL::Request>& request);
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request>& request);
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture setMode_(const std::shared_ptr<mavros_msgs::srv::SetMode::Request>& request);
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture servo_(const std::shared_ptr<mavros_msgs::srv::CommandLong::Request>& request);
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture closeCamera_(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request);
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture openCamera_(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request);
    rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedFuture setParam_(const std::shared_ptr<mavros_msgs::srv::ParamSetV2::Request>& request);
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture pushMission_(const std::shared_ptr<mavros_msgs::srv::WaypointPush::Request>& request);
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedFuture clearMission_(const std::shared_ptr<mavros_msgs::srv::WaypointClear::Request>& request);

    // Functions to publish messages
    void publishLocalPosition(const geometry_msgs::msg::PoseStamped &pose);
    void publishVelocity(const geometry_msgs::msg::Twist &vel);
    void publishLocalVelocity(const geometry_msgs::msg::TwistStamped &vel);
    void publishLocalVelocityUnstamped(const geometry_msgs::msg::Twist &vel);
    void publishOpenCamera(const std_msgs::msg::Bool &open);
    void publishOpenCamera2(const std_msgs::msg::Bool &open);
    void publishSetpointRawGlobal(const mavros_msgs::msg::GlobalPositionTarget &setpoint);
    void publishSetpointRawLocal(const mavros_msgs::msg::PositionTarget &setpoint);

    std::vector<double> lat_indoor, lon_indoor, lat_outdoor, lon_outdoor;

    int current_wp_();
    int reset_wp_();

private:
    // Callback functions to receive messages
    void state_cb(const mavros_msgs::msg::State::ConstSharedPtr msg);                   
    void gps_state_cb(const mavros_msgs::msg::GPSRAW::SharedPtr msg);                   
    void pose_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void gate_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void payload_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void artag_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void ember_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void bunder_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void vel_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
    void accel_cb(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
    void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void compass_hdg_cb(const std_msgs::msg::Float64::SharedPtr msg);
    void rel_alt_cb(const std_msgs::msg::Float64::SharedPtr msg);
    void alt_cb(const sensor_msgs::msg::Range::SharedPtr msg);
    void wp_reached_cb(const mavros_msgs::msg::WaypointReached::SharedPtr msg);
    void laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    template<typename T>
    void waitForService(T &client) {
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) { RCLCPP_ERROR(this->get_logger(), "ROS errrrorr"); return; }
            RCLCPP_INFO(this->get_logger(), "Sekkk nunggu service %s...", client->get_service_name());
        }
    }
    
    mavros_msgs::msg::State                        curr_state;
    mavros_msgs::msg::GPSRAW                       gps_state;
    geometry_msgs::msg::PoseStamped                curr_pose;
    geometry_msgs::msg::PoseStamped                gate_pose;
    geometry_msgs::msg::PoseStamped                artag_pose;
    geometry_msgs::msg::PoseStamped                payload_pose;
    geometry_msgs::msg::PoseStamped                ember_pose;
    geometry_msgs::msg::PoseStamped                bunder_pose;
    geometry_msgs::msg::PoseStamped                nonzero_last_artag_pose;
    geometry_msgs::msg::PoseStamped                nonzero_last_payload_pose;
    geometry_msgs::msg::PoseStamped                nonzero_last_ember_pose;
    geometry_msgs::msg::PoseStamped                nonzero_last_bunder_pose;
    geometry_msgs::msg::PoseStamped                nonzero_drone_artag_pose;
    geometry_msgs::msg::PoseStamped                nonzero_drone_payload_pose;
    geometry_msgs::msg::PoseStamped                nonzero_drone_ember_pose;
    geometry_msgs::msg::PoseStamped                nonzero_drone_bunder_pose;
    geometry_msgs::msg::Point                      curr_euler;
    geometry_msgs::msg::TwistStamped               curr_vel;
    geometry_msgs::msg::PoseStamped                last_artag_pose;
    geometry_msgs::msg::PoseStamped                last_payload_pose;
    geometry_msgs::msg::PoseStamped                last_ember_pose;
    geometry_msgs::msg::PoseStamped                last_bunder_pose;
    geometry_msgs::msg::AccelWithCovarianceStamped curr_accel;
    sensor_msgs::msg::NavSatFix                    curr_gps_position;
    geometry_msgs::msg::PoseStamped                init_drone_pose;
    std_msgs::msg::Float64                         compass_hdg;
    std_msgs::msg::Float64                         curr_rel_alt;
    sensor_msgs::msg::Range                        curr_rangefinder;
    sensor_msgs::msg::LaserScan                    curr_laserscan;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr        local_pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr       local_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr              local_vel_pub_unstamped;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    open_cam_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    open_cam2_pub;
    rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr setpoint_raw_global_pub;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr       setpoint_raw_local_pub;
    
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr    takeoff_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr    land_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr   arming_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr       set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr   servo_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr          close_cam_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr          open_cam_client;
    rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr    set_param_client;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr  push_mission_client;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr clear_mission_client;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr                        state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                gate_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                artag_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                payload_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                ember_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                bunder_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr               vel_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr                         compass_hdg_sub;
    rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr                       gps_state_sub;
    rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                    gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr                        alt_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr                         rel_alt_sub;
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr              wp_reached_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    laser_sub;

    int wp_reached;

};

#endif // DRONE_CONTROLLER_HPP
