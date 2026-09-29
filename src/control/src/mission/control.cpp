#include "control_.hpp"
#include "math_.hpp"
#include "geo_.hpp"
#include "drone_controller_.hpp"
#include <iostream>
#include <vector>
#include <string>

#define RATE 10.0

float takeoff_altitude;
float forward_distance;
float gate_width;
float centering_tolerance;
float waypoint_tolerance;
std::string control_mode;  
float max_velocity;
float proportional_gain;
float rotation_angle; 
float rotation_radians;

float lateral_tolerance;
float vertical_tolerance;
int min_points_vertical;
int min_points_horizontal;
float max_time = 30.0;  // Default timeout for centering  

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(RATE);
    auto node = std::make_shared<DroneController>();
    
    node->declare_parameter<float>("gate_centering.takeoff_altitude", 2.0);
    node->declare_parameter<float>("gate_centering.forward_distance", 3.0);
    node->declare_parameter<float>("gate_centering.gate_width", 1.5);
    node->declare_parameter<float>("gate_centering.centering_tolerance", 0.15);
    node->declare_parameter<float>("gate_centering.waypoint_tolerance", 0.3);
    node->declare_parameter<std::string>("gate_centering.control_mode", "velocity");
    node->declare_parameter<float>("gate_centering.max_velocity", 0.2);
    node->declare_parameter<float>("gate_centering.proportional_gain", 0.5);
    node->declare_parameter<float>("gate_centering.rotation_angle", 90.0);
    
    // Clustering parameters
    node->declare_parameter<float>("gate_centering.detection_range_min", 1.5);
    node->declare_parameter<float>("gate_centering.detection_range_max", 2.5);
    node->declare_parameter<float>("gate_centering.roi_lateral", 2.0);
    node->declare_parameter<float>("gate_centering.cluster_epsilon", 0.15);
    node->declare_parameter<int>("gate_centering.min_cluster_points", 10);
    node->declare_parameter<float>("gate_centering.min_pole_height", 0.5);
    node->declare_parameter<float>("gate_centering.target_height", 0.75);

    
    node->get_parameter("gate_centering.takeoff_altitude", takeoff_altitude);
    node->get_parameter("gate_centering.forward_distance", forward_distance);
    node->get_parameter("gate_centering.gate_width", gate_width);
    node->get_parameter("gate_centering.centering_tolerance", centering_tolerance);
    node->get_parameter("gate_centering.waypoint_tolerance", waypoint_tolerance);
    node->get_parameter("gate_centering.control_mode", control_mode);
    node->get_parameter("gate_centering.max_velocity", max_velocity);
    node->get_parameter("gate_centering.proportional_gain", proportional_gain);
    node->get_parameter("gate_centering.rotation_angle", rotation_angle);
    rotation_radians = rotation_angle * M_PI / 180.0;
    
    RCLCPP_INFO(node->get_logger(), "=== Centering Basic ===");
    RCLCPP_INFO(node->get_logger(), "Takeoff Alt: %.2f m", takeoff_altitude);
    RCLCPP_INFO(node->get_logger(), "Forward Dist: %.2f m", forward_distance);
    RCLCPP_INFO(node->get_logger(), "Lebar Gate: %.2f m", gate_width);
    RCLCPP_INFO(node->get_logger(), "Centering Tolerance: %.2f m", centering_tolerance);
    RCLCPP_INFO(node->get_logger(), "Control Mode: %s", control_mode.c_str());
    if (control_mode == "velocity") {
        RCLCPP_INFO(node->get_logger(), "Max Velocity: %.2f m/s", max_velocity);
        RCLCPP_INFO(node->get_logger(), "Proportional Gain: %.2f", proportional_gain);
    }
    
    geometry_msgs::msg::PoseStamped posee;
    initFrame(node, posee);
    
    while(rclcpp::ok() && node->getCurrentLocalPose().pose.position.z == 0.0) {
        RCLCPP_INFO(node->get_logger(), "Wait local pose data...");
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "TAKING OFF.... %.2f meters...", takeoff_altitude);
    takeoff(node, rate, posee, 15.0);
    holdPosition(node, rate, posee, 10.0);

    RCLCPP_INFO(node->get_logger(), "MOVE FORWARD to %.2f meters...", forward_distance);
    LocalMove(node, rate, posee, 25.0, 0.0, 0.0, 0.0, waypoint_tolerance, false);
    holdPosition(node, rate, posee, 5.0);

    RCLCPP_INFO(node->get_logger(), "MOVE FORWARD to %.2f meters...", forward_distance);
    LocalMove(node, rate, posee, 0.0, 5.0, 0.0, 0.0, waypoint_tolerance, false);
    holdPosition(node, rate, posee, 5.0);

    RCLCPP_INFO(node->get_logger(), "MOVE FORWARD to %.2f meters...", forward_distance);
    LocalMove(node, rate, posee, -35.0, 0.0, 0.0, 0.0, waypoint_tolerance, false);
    holdPosition(node, rate, posee, 5.0);

    // RCLCPP_INFO(node->get_logger(), "CENTERING ARTAG...");
    // bool status = false;
    // centering_artag(node, rate, 0.75, status, 0.05, 0.20, 0.30, 0.0, 0.2, 2.5, 2.5, 0.0, 0.0, "local_pose", 0.05);
    // holdPosition(node, rate, posee, 2.0);

    // RCLCPP_INFO(node->get_logger(), "Step 4: Correcting heading...");
    // correct_heading_artag(node, rate, status, 3.0, 0.5, 30.0);
    // holdPosition(node, rate, posee, 2.0);

    // RCLCPP_INFO(node->get_logger(), "ROTATING... %.2f degrees...", 90.0);
    // rotateByDegrees(node, rate, posee, 90.0);
    // holdPosition(node, rate, posee, 2.0);

    // RCLCPP_INFO(node->get_logger(), "MOVE FORWARD to %.2f meters...", forward_distance);
    // LocalMove(node, rate, posee, 3.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    // holdPosition(node, rate, posee, 5.0);

    //centering_tag(node, rate, 0.1, 0.8);
    // RCLCPP_INFO(node->get_logger(), "CNTERING TAG...");
    // centering_tag(node, rate, 2000.0, 0.0, 10.0, 0.02, 120.0, true, 0.0);
    // holdPosition(node, rate, posee, 2.0);

    setMode(node, rate, "rtl");

    // RCLCPP_INFO(node->get_logger(), "ROTATING... %.2f degrees...", 90.0);
    // rotateByDegrees(node, rate, posee, 90.0);
    // holdPosition(node, rate, posee, 2.0);

    // executeLocalWaypointMove(node, rate, posee, 2.0, 0.0, 0.0, 0.0, waypoint_tolerance);

    // bool centering_status = false;
    // RCLCPP_INFO(node->get_logger(), "CENTERING GATE...");
    // centering_gate(node, rate, posee, gate_width, centering_tolerance, centering_status, 0.2, 0.5, 60.0, false, false);
    // holdPosition(node, rate, posee, 2.0);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "CENTERED TO GATE!");
    //     executeLocalWaypointMove(node, rate, posee, 10.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 2.0);
    // } else {
    //     RCLCPP_WARN(node->get_logger(), "CENTERING FAILED");
    //     setMode(node, rate, "rtl");
    // }
    
    RCLCPP_INFO(node->get_logger(), "================= Mission Complete =================");
    rclcpp::shutdown();
    return 0;
}