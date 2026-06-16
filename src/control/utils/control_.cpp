#include "control_.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <algorithm>

namespace {
std::string normalizeFlightMode(std::string mode) {
    std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);
    if (mode == "RTL") {
        return "AUTO.RTL";
    }
    if (mode == "LAND") {
        return "AUTO.LAND";
    }
    if (mode == "MISSION" || mode == "AUTO") {
        return "AUTO.MISSION";
    }
    if (mode == "LOITER" || mode == "HOLD") {
        return "AUTO.LOITER";
    }
    return mode;
}
} 

bool payloadDetected(const std::shared_ptr<DroneController>&node){
    /**
     * Check if the payload is detected by verifying if its position is not at the origin (0,0,0).]
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the payload is detected, false otherwise.
     */
    geometry_msgs::msg::PoseStamped payload_pose = node->getCurrentPosePayload();
    return (payload_pose.pose.position.x != 0.000 && payload_pose.pose.position.y != 0.000 && payload_pose.pose.position.z != 0.000);
}

bool emberDetected(const std::shared_ptr<DroneController>&node){
    /**
     * Check if the ember is detected by verifying if its position is not at the origin (0,0,0).
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the ember is detected, false otherwise.
     */
    geometry_msgs::msg::PoseStamped ember_pose = node->getCurrentPoseEmber();
    return (ember_pose.pose.position.x != 0.000 && ember_pose.pose.position.y != 0.000 && ember_pose.pose.position.z != 0.000);
}

bool bunderDetected(const std::shared_ptr<DroneController>&node){
    /**
     * Check if the ember is detected by verifying if its position is not at the origin (0,0,0).
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the ember is detected, false otherwise.
     */
    geometry_msgs::msg::PoseStamped bunder_pose = node->getCurrentPoseBunder();
    return (bunder_pose.pose.position.x != 0.000 && bunder_pose.pose.position.y != 0.000 && bunder_pose.pose.position.z != 0.000);
}

bool artagDetected(const std::shared_ptr<DroneController>&node){
    geometry_msgs::msg::PoseStamped artag_pose = node->getCurrentPoseArTag();
    return (artag_pose.pose.position.x != 0.0 || artag_pose.pose.position.y != 0.0);
}


void initFrame(const std::shared_ptr<DroneController>& node, geometry_msgs::msg::PoseStamped &pose) {
    /**
     * Initialize the starting pose 'node' with the current position of the drone and reset payload pose.
     * This is used to reset posee or agg_pose (reference to where the last drone position when a control
     * function is called, it will be modified to the last drone position when the function returns.)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - pose: reference to the geometry_msgs::msg::PoseStamped object to be initialized.
     */
    geometry_msgs::msg::PoseStamped last_payload_pose = node->getLastPayloadPose();
    setPosition(last_payload_pose,0,0,0);
    node->setLastPayloadPose(last_payload_pose);
    for(int i = 0; i < 30 && rclcpp::ok(); i++) {
        geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
        setPosition(pose, curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z);
        pose.pose.orientation = curr_pose.pose.orientation;
        rclcpp::spin_some(node); rclcpp::Rate(RATE).sleep();
    }
}


void setParam(const std::shared_ptr<DroneController>&node, const std::string &id, int integer_value) {\
    /**
     * Set a parameter on the drone controller node.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - id: string identifier for the parameter to be set.
     * - integer_value: integer value to set for the parameter.
     */
    auto set_param_req = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();

    set_param_req->force_set = true;
    set_param_req->param_id = id;
    set_param_req->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    set_param_req->value.integer_value = integer_value;

    RCLCPP_INFO(node->get_logger(), "Sekkk nunggu service param_set");
    node->waitForSetParamService();

    if (node->isSetParamServiceReady()) {
        auto result_future = node->setParam_(set_param_req);

        if (hasReplied(node, result_future)) {
            auto result = result_future.get();
            if (result->success) { RCLCPP_INFO(node->get_logger(), "GGWP Param berubah haruse"); } else { RCLCPP_ERROR(node->get_logger(), "Nooo gagal ngubah param"); }
        } else { RCLCPP_ERROR(node->get_logger(), "Timeout / gagal mendapatkan respon dari service"); }
    } else { RCLCPP_ERROR(node->get_logger(), "Set param service tidak tersedia setelah menunggu"); }
}

void setMode(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, const std::string mode) {
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    const std::string normalized_mode = normalizeFlightMode(mode);
    set_mode_request->custom_mode = normalized_mode;
    
    RCLCPP_INFO(node->get_logger(), "Attempting to set mode to %s...", normalized_mode.c_str());
    node->waitForSetModeService();
    
    if (node->isSetModeServiceReady()) {   
        auto set_mode_result = node->setMode_(set_mode_request);   
        if (rclcpp::spin_until_future_complete(node, set_mode_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = set_mode_result.get();
            if (result->mode_sent) {
                RCLCPP_INFO(node->get_logger(), "Mode set to %s", normalized_mode.c_str());
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to set mode to %s", normalized_mode.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call SetMode service");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "SetMode service not ready");
    }
    rate.sleep();
}

// void takeoff(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt) {
//     /**
//      * Initiate the takeoff procedure for the drone. Set mode to GUIDED, arm the drone, and send the takeoff command.
//      * parameters:
//      * - node: shared pointer to the DroneController node instance.
//      * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
//      * - takeoff_alt: float value representing the desired takeoff altitude, this is based on the rangefinder reading.
//      * 
//      * returns:
//      * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
//      * - rel_alt: float value representing the relative altitude to be maintained at the correct rangefinder reading.
//      */
//     geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
//     hold_pose.pose.position.z = std::max(hold_pose.pose.position.z, 0.0);
//     streamOffboardSetpoint(node, hold_pose, 40, 20.0);

//     setMode(node, rate, "OFFBOARD");

//     auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//     arm_request->value = true;
//     node->waitForArmingService();
//     if (node->isArmingServiceReady()) {
//         auto arm_result = node->arm_(arm_request);
//         if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
//             auto response = arm_result.get();
//             if (response->success) {
//                 RCLCPP_INFO(node->get_logger(), "Vehicle armed");
//             } else {
//                 RCLCPP_ERROR(node->get_logger(), "Arming rejected by FCU");
//                 return;
//             }
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
//             return;
//         }
//     } else {
//         RCLCPP_ERROR(node->get_logger(), "Arming service not ready");
//         return;
//     }

//     int command_takeoff = -1;
//     while (rclcpp::ok()) {
//         RCLCPP_INFO(node->get_logger(), "Press 1 + Enter to takeoff, or 0 + Enter to disarm.");
//         if (!(std::cin >> command_takeoff)) {
//             std::cin.clear();
//             std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//             RCLCPP_WARN(node->get_logger(), "Invalid input. Please enter 1 or 0.");
//             continue;
//         }
//         if (command_takeoff == 1 || command_takeoff == 0) {
//             break;
//         }
//         RCLCPP_WARN(node->get_logger(), "Invalid option %d. Please enter 1 or 0.", command_takeoff);
//     }

//     if (command_takeoff == 0) {
//         RCLCPP_INFO(node->get_logger(), "Takeoff canceled. Disarming vehicle...");
//         disarm(node, rate);
//         return;
//     }  

float applyPrecisionStep(float command, float min_velocity, float max_velocity) {
    const float clamped = static_cast<float>(clamp(max_velocity, command));
    if (std::abs(clamped) < 1e-6f) {
        return 0.0f;
    }
    if (std::abs(clamped) < min_velocity) {
        return std::copysign(min_velocity, clamped);
    }
    return clamped;
}

//     geometry_msgs::msg::PoseStamped target_pose = node->getCurrentLocalPose();
//     target_pose.pose.position.z = takeoff_alt;
//     streamOffboardSetpoint(node, target_pose, 10, 20.0);

//     RCLCPP_INFO(node->get_logger(), "Waiting to reach altitude %.2f m (timeout: 20s)...", takeoff_alt);
//     auto start_time = node->now();
//     const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(20.0);
//     bool reached = false;
//     rclcpp::Rate fast_rate(20.0);
//     while (rclcpp::ok()) {
//         const auto current_pose = node->getCurrentLocalPose();
//         const double alt = current_pose.pose.position.z;
//         if (std::abs(takeoff_alt - alt) <= 0.15) {
//             reached = true;
//             break;
//         }
//         if ((node->now() - start_time) > timeout_duration) {
//             RCLCPP_WARN(node->get_logger(), "Timeout waiting to reach takeoff altitude");
//             break;
//         }
//         target_pose.header.stamp = node->now();
//         target_pose.header.frame_id = "map";
//         node->publishLocalPosition(target_pose);
//         rclcpp::spin_some(node);
//         fast_rate.sleep();
//     }

//     if (reached) {
//         RCLCPP_INFO(node->get_logger(), "Reached takeoff altitude!");
//     } else {
//         RCLCPP_WARN(node->get_logger(), "Did not reach takeoff altitude within timeout");
//     }

//     posee.pose.position.z = node->getCurrentLocalPose().pose.position.z;
// }

void takeoff(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt) {
    /**
     * Initiate the takeoff procedure for the drone. Set mode to OFFBOARD, arm the drone, and send the takeoff command.
     * For PX4: Stream setpoint at high rate BEFORE switching to OFFBOARD mode
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - takeoff_alt: float value representing the desired takeoff altitude, this is based on the rangefinder reading.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - rel_alt: float value representing the relative altitude to be maintained at the correct rangefinder reading.
     */
    
    RCLCPP_INFO(node->get_logger(), "=== TAKEOFF ===");
    RCLCPP_INFO(node->get_logger(), "Target altitude: %.2f meters", takeoff_alt);
    
    // Get current position - NO OFFSET CORRECTION, use raw local position Z
    geometry_msgs::msg::PoseStamped target_pose = node->getCurrentLocalPose();
    // Set target altitude directly from local position z + desired takeoff altitude
    target_pose.pose.position.z = target_pose.pose.position.z + takeoff_alt;
    
    RCLCPP_INFO(node->get_logger(), "Current Z: %.3f m, Target Z: %.3f m", 
                node->getCurrentLocalPose().pose.position.z, target_pose.pose.position.z);
    
    // PX4 CRITICAL: Stream setpoints BEFORE switching to OFFBOARD mode
    // Send at least 100 setpoints at 20Hz (2 seconds) before mode switch
    rclcpp::Rate fast_rate(20.0); // 20 Hz for PX4
    for(int i = 0; i < 40 && rclcpp::ok(); i++) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Setting OFFBOARD mode...");
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "OFFBOARD";   
    
    if (node->isSetModeServiceReady()) {   
        auto set_mode_result = node->setMode_(set_mode_request);   
        if (rclcpp::spin_until_future_complete(node, set_mode_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = set_mode_result.get();
            if (result->mode_sent) {
                RCLCPP_INFO(node->get_logger(), "OFFBOARD mode set");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to set OFFBOARD mode");
                return;
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call SetMode service");
            return;
        }
    }
    
    // Continue streaming while waiting
    for(int i = 0; i < 10 && rclcpp::ok(); i++) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Arming vehicle...");
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

    if (node->isArmingServiceReady()) {
        auto arm_result = node->arm_(arm_request);
        if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = arm_result.get();
            if (result->success) {
                RCLCPP_INFO(node->get_logger(), "Vehicle armed");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to arm: %s", result->result ? "true" : "false");
                return;
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
            return;
        }
    }

    RCLCPP_INFO(node->get_logger(), "=== TAKEOFF CONFIRMATION ===");
    RCLCPP_INFO(node->get_logger(), "takeoff 1 if yes ?");  
    int confirmation;
    std::cin >> confirmation;
    
    if (confirmation != 1) {
        RCLCPP_WARN(node->get_logger(), "Takeoff cancelled by user");
        return;
    }
    
    // Continue streaming during arm
    for(int i = 0; i < 10 && rclcpp::ok(); i++) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Ascending to %.2f meters...", takeoff_alt);
    
    // For PX4 OFFBOARD: Just stream position setpoints, no separate takeoff command needed
    // The drone will automatically follow the z position setpoint
    auto start_time = node->now();
    const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(30.0);
    bool reached = false;
    double current_alt = 0.0;

    while (rclcpp::ok()) {
        // Keep streaming setpoint at high rate (CRITICAL for PX4 OFFBOARD)
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        
        // Get current altitude - NO OFFSET, use raw local position Z
        current_alt = node->getCurrentLocalPose().pose.position.z;
        
        if (fmod((node->now() - start_time).seconds(), 0.1) < 0.05) { // Log every ~1 second
            RCLCPP_INFO(node->get_logger(), "Altitude: %.2f m / %.2f m (target)", 
                        current_alt, target_pose.pose.position.z);
        }

        // Check if reached target altitude (direct comparison without offset)
        if (fabs(target_pose.pose.position.z - current_alt) < 0.15) {
            reached = true; 
            break;
        }

        if ((node->now() - start_time) > timeout_duration) {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting to reach takeoff altitude");
            break;
        }

        rclcpp::spin_some(node);
        fast_rate.sleep(); // Keep streaming at 20Hz
    }

    if (reached) {
        RCLCPP_INFO(node->get_logger(), "Reached takeoff altitude! (%.2f m)", current_alt);
    } else {
        RCLCPP_WARN(node->get_logger(), "Did not reach takeoff altitude within timeout");
    }
    
    rate.sleep();
    posee.pose.position.z = node->getCurrentLocalPose().pose.position.z;
}

void takeoff_no_confirm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt) {
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
    // auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    // set_mode_request->custom_mode = "GUIDED";   
    
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

    // if (node->isSetModeServiceReady()) {   
    //     auto set_mode_result = node->setMode_(set_mode_request);   
    //     if (rclcpp::spin_until_future_complete(node, set_mode_result) == rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_INFO(node->get_logger(), "Set mode to GUIDED");
    //     } else {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call SetMode service");
    //     }
    // }
    // rate.sleep();

    if (node->isArmingServiceReady()) {
        auto arm_result = node->arm_(arm_request);
        if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = arm_result.get();
            if (response->success) {
                RCLCPP_INFO(node->get_logger(), "Vehicle armed");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Arming rejected by FCU");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
        }
    }
    rate.sleep();

    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    takeoff_request->altitude = takeoff_alt + node->getCurrentRelAlt().data;
    if(takeoff_request->altitude <= takeoff_alt - node->getCurrentRangefinder().range)
    {
        takeoff_request->altitude = takeoff_alt;
    }

    if (node->isTakeoffServiceReady()) {
        for (int i = 0; i<5; ++i)
        {
            auto takeoff_result = node->takeoff_(takeoff_request);
            if (rclcpp::spin_until_future_complete(node, takeoff_result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Takeoff command sent, request: %f, range: %f, rel_alt: %f, z: %f",
                    takeoff_request->altitude, node->getCurrentRangefinder().range, node->getCurrentRelAlt().data, node->getCurrentLocalPose().pose.position.z);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to send takeoff command");
            }
        }
    }

    RCLCPP_INFO(node->get_logger(), "Waiting to reach altitude %.2f m (timeout: 5s)...", takeoff_alt);
    auto start_time = node->now();
    const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(5.0);

    bool reached = false;

    while (rclcpp::ok()) {
        double alt = node->getCurrentRangefinder().range;
        RCLCPP_INFO(node->get_logger(), "Current altitude: %.2f m", alt);

        if ( 0.1 >= abs(0.55 - alt)) {
            reached = true; 
            break;
        }

        if ((node->now() - start_time) > timeout_duration) {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting to reach takeoff altitude");
            break;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (reached) {
        RCLCPP_INFO(node->get_logger(), "Reached takeoff altitude!");
    } else {
        RCLCPP_WARN(node->get_logger(), "Did not reach takeoff altitude within timeout");
    }

    rate.sleep();
    posee.pose.position.z = node->getCurrentLocalPose().pose.position.z;
}

// void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &target_pose, float max_time) {
//     /**
//      * Wait for the drone to stably hover at the target pose while constantly updating pose and heading to mitigate drift.
//      * 
//      * parameters:
//      * - node: shared pointer to the DroneController node instance.
//      * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
//      * - target_pose: reference to the geometry_msgs::msg::PoseStamped object representing the target pose for stabilization.
//      */
//     geometry_msgs::msg::PoseStamped new_pose;
//     rclcpp::Time start_time = node->now();

//     while (rclcpp::ok()) {
//         rclcpp::Time current_time = node->now();
//         float elapsed_time = (current_time - start_time).seconds();
//         // if (isNear(curr_pose.pose.position, target_pose.pose.position.x, target_pose.pose.position.y, 0.05) && (abs(getHeading(curr_pose.pose.orientation) - getHeading(target_pose.pose.orientation)) < 0.05)){
//         if (isNear(node->getCurrentLocalPose().pose.position, target_pose.pose.position.x, target_pose.pose.position.y, 0.05)){
//             RCLCPP_INFO(node->get_logger(), "++++++++ Stabilized ++++++++++++++++++");
//             break;
//         }

//         if (elapsed_time > max_time) {
//             RCLCPP_WARN(node->get_logger(), "It took too long to stabilize! The drone might be wobbly");
//             break;
//         }

//         setPosition(new_pose, target_pose.pose.position.x, target_pose.pose.position.y, node->getCurrentLocalPose().pose.position.z);
//         setHeading(new_pose.pose.orientation, getHeading(target_pose.pose.orientation));
//         node->publishLocalPosition(new_pose);
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
// }

void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration){
    /**
     * Wait for a specified duration. In guided mode, the drone should hold the current position when doing nothing.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - duration: float value representing the duration in seconds to hold the position.
     */
    rclcpp::Time start_time = node->now();
    while (rclcpp::ok()) {
        rclcpp::Time current_time = node->now();
        double elapsed_time = (current_time - start_time).seconds();
        
        node->publishLocalPosition(posee);
        RCLCPP_INFO(node->get_logger(), "++++++++++++++++++++ Stabilize ++++++++++++++++++++");
        if (elapsed_time > duration){break;}

        rclcpp::spin_some(node); rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "+++++++++++++++++ Selesai Stabilize +++++++++++++++++++");

    posee = node->getCurrentLocalPose();
}

void waitLand(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate){
    /**
     * Wait until the drone has landed by checking its altitude.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */
    while (rclcpp::ok() && node->getCurrentLocalPose().pose.position.z > 0.1){
        RCLCPP_INFO(node->get_logger(), "current altitude: %f", node->getCurrentLocalPose().pose.position.z);
        rclcpp::spin_some(node); rate.sleep();
    }
}

void land(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate){
    /**
     * Initiate the landing procedure for the drone by sending a land command.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */
    auto land_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_req->altitude = 0;
    if (node->isLandServiceReady()) {
        auto result = node->land_(land_req);
        if (hasReplied(node,result)) { RCLCPP_INFO(node->get_logger(), "land command sent"); waitLand(node,rate);} 
        else { RCLCPP_ERROR(node->get_logger(), "failed to send land command");}
    }
}

void safeLanding(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float descent_rate = 0.3) {
    /**
     * Perform controlled safe landing with gradual descent in GUIDED mode.
     * Much safer than using LAND mode which can be too fast.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - posee: reference to current pose (will be updated during descent)
     * - descent_rate: descent speed in m/s (default 0.3 m/s = gentle descent)
     */
    RCLCPP_INFO(node->get_logger(), "=== Starting Safe Landing ===");
    RCLCPP_INFO(node->get_logger(), "Descent rate: %.2f m/s", descent_rate);
    
    rclcpp::Rate fast_rate(10.0); // 10Hz for ArduPilot
    
    // Get current position
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    
    float initial_altitude = current_pose.pose.position.z;
    float target_ground_altitude = 0.05f; // Stop just above ground (5cm)
    
    RCLCPP_INFO(node->get_logger(), "Current altitude: %.2f m", initial_altitude);
    
    // Phase 1: Controlled descent to near ground
    auto descent_start = node->now();
    
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        float current_alt = current_pose.pose.position.z;
        
        // Check if near ground
        if (current_alt <= target_ground_altitude) {
            RCLCPP_INFO(node->get_logger(), "Near ground (%.2f m), proceeding to disarm...", current_alt);
            break;
        }
        
        // Calculate time-based descent
        float elapsed = (node->now() - descent_start).seconds();
        float desired_alt = initial_altitude - (descent_rate * elapsed);
        
        // Don't go below ground
        if (desired_alt < target_ground_altitude) {
            desired_alt = target_ground_altitude;
        }
        
        // Set target position (maintain x,y, only decrease z)
        target_pose.pose.position.x = current_pose.pose.position.x;
        target_pose.pose.position.y = current_pose.pose.position.y;
        target_pose.pose.position.z = desired_alt;
        target_pose.pose.orientation = current_pose.pose.orientation;
        
        // Stream setpoint
        target_pose.header.stamp = node->now();
        target_pose.header.frame_id = "map";
        node->publishLocalPosition(target_pose);
        
        // Log progress
        if (fmod(elapsed, 1.0) < 0.05) { // Every ~1 second
            RCLCPP_INFO(node->get_logger(), "Landing... Alt: %.2f m -- %.2f m", 
                       current_alt, desired_alt);
        }
        
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    // Phase 2: Hold position briefly at ground level
    RCLCPP_INFO(node->get_logger(), "Holding at ground level for 0.5s...");
    target_pose = node->getCurrentLocalPose();
    target_pose.pose.position.z = 0.0; // Target ground
    
    auto hold_start = node->now();
    while ((node->now() - hold_start).seconds() < 0.5 && rclcpp::ok()) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    // Phase 3: Disarm (safe on ground)
    RCLCPP_INFO(node->get_logger(), "Disarming...");
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = false; // Disarm
    
    if (node->isArmingServiceReady()) {
        auto result = node->arm_(arm_request);
        if (hasReplied(node, result)) {
            auto response = result.get();
            if (response->success) {
                RCLCPP_INFO(node->get_logger(), "Disarmed successfully");
            } else {
                RCLCPP_WARN(node->get_logger(), "Disarm command sent but not confirmed");
            }
        }
    }
    
    // Continue streaming at ground for safety
    for (int i = 0; i < 20; i++) { // 1 second
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Safe landing complete!");
    posee = node->getCurrentLocalPose();
}

void Descend(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee){
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
    RCLCPP_INFO(node->get_logger(), "===================== Turun ===================");
    geometry_msgs::msg::TwistStamped vel;
    bool was_descending = false, trying_descending = false;

    vel.twist.linear.x = 0.0;
    vel.twist.linear.y = 0.0;
    vel.twist.linear.z = -0.6;

    while (rclcpp::ok()){
        node->publishLocalVelocity(vel);
        if (node->getCurrentVelocity().twist.linear.z < -0.05 && !trying_descending)  { RCLCPP_INFO(node->get_logger(), "Descend started");trying_descending = true;}
        if (node->getCurrentVelocity().twist.linear.z < -0.35)                        { RCLCPP_INFO(node->get_logger(), "Descending...")  ;was_descending = true;}
        if (was_descending && payloadDetected(node))                                  { node->setLastPayloadPose(node->getCurrentPosePayload());}
        if (abs(node->getCurrentVelocity().twist.linear.z) < 0.055 && was_descending) { RCLCPP_INFO(node->get_logger(), "Landed")         ;break;}
        rclcpp::spin_some(node);rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Latest offset: %f, %f", node->getLastPayloadPose().pose.position.x, node->getLastPayloadPose().pose.position.y);
    posee = node->getCurrentLocalPose();
}

void calibrateHoverOrientation(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float max_time, float hover_pitch, float hover_roll, bool calibrate_pitch, bool calibrate_roll) {
    RCLCPP_INFO(node->get_logger(), "------------- Calibrate Hover Orientation --------------");
    stabilize(node, rate, posee, max_time);

    posee = node->getCurrentLocalPose();
    hover_pitch = calibrate_pitch ? getPitch(posee.pose.orientation) : hover_pitch;
    hover_roll = calibrate_roll ? getRoll(posee.pose.orientation) : hover_roll;

    RCLCPP_INFO(node->get_logger(), "hover_pitch = %f, hover_roll = %f", hover_pitch, hover_roll);
}

void centering_payload(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    float step,
    float offset,
    float close_threshold,
    float max_velocity,
    float timeout_sec,
    bool downward_camera,
    float min_velocity) {
    if (step <= 0.0f) {
        RCLCPP_ERROR(node->get_logger(), "centering_payload: step must be > 0");
        return;
    }

    std_msgs::msg::Float64MultiArray::SharedPtr latest_payload_msg = nullptr;
    auto payload_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/payload_pose",
        10,
        [&latest_payload_msg](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            latest_payload_msg = msg;
        });

    (void)payload_sub;
    const auto start_time = node->now();
    bool ever_detected = false;
    geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::TwistStamped cmd_vel;

    RCLCPP_INFO(node->get_logger(), "centering_payload: waiting for /payload_pose...");
    while (rclcpp::ok()) {
        if ((node->now() - start_time).seconds() > timeout_sec) {
            RCLCPP_WARN(node->get_logger(), "centering_payload: timeout after %.1f sec", timeout_sec);
            break;
        }

        if (!latest_payload_msg) {
            hold_pose.header.stamp = node->now();
            hold_pose.header.frame_id = "map";
            node->publishLocalPosition(hold_pose);
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "centering_payload: target not detected, holding position");
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }

        if (latest_payload_msg->data.size() < 3) {
            hold_pose.header.stamp = node->now();
            hold_pose.header.frame_id = "map";
            node->publishLocalPosition(hold_pose);
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "centering_payload: /payload_pose data must contain [x, y, center_dist], got %zu",
                latest_payload_msg->data.size());
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }

        const float x = static_cast<float>(latest_payload_msg->data[0]);
        const float y = static_cast<float>(latest_payload_msg->data[1]);
        const float center_dist = static_cast<float>(latest_payload_msg->data[2]);
        const bool detected_now =
            std::isfinite(x) && std::isfinite(y) && std::isfinite(center_dist) &&
            (std::abs(x) > 1e-4f || std::abs(y) > 1e-4f || std::abs(center_dist) > 1e-4f);

        if (detected_now) {
            ever_detected = true;
        }

        if (!ever_detected) {
            hold_pose.header.stamp = node->now();
            hold_pose.header.frame_id = "map";
            node->publishLocalPosition(hold_pose);
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "centering_payload: waiting for first valid target, holding position");
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }

        // Camera frame error with configurable offset.
        const float cam_err_x = x - offset;
        const float cam_err_y = y - offset;

        // Downward camera mapping:
        // image +x (right) -> drone right  => body y negative (left-positive convention)
        // image +y (down)  -> drone back   => body x negative (forward-positive convention)
        float body_err_x = cam_err_x;
        float body_err_y = cam_err_y;
        if (downward_camera) {
            body_err_x = -cam_err_y;
            body_err_y = -cam_err_x;
        }

        const float fallback_dist = std::hypot(body_err_x, body_err_y);
        const float usable_dist = (std::abs(center_dist) > 1e-6f) ? std::abs(center_dist) : fallback_dist;
        const float scaled_dist = usable_dist / step;

        if (scaled_dist <= close_threshold) {
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.linear.z = 0.0;
            node->publishLocalVelocity(cmd_vel);
            RCLCPP_INFO(node->get_logger(), "CENTEREDD...!! (scaled_dist=%.4f)", scaled_dist);
            break;
        }

        cmd_vel.twist.linear.x = applyPrecisionStep(body_err_x / step, min_velocity, max_velocity);
        cmd_vel.twist.linear.y = applyPrecisionStep(body_err_y / step, min_velocity, max_velocity);
        cmd_vel.twist.linear.z = 0.0;
        node->publishLocalVelocity(cmd_vel);

        RCLCPP_INFO_THROTTLE(
            node->get_logger(), *node->get_clock(), 800,
            "centering_payload: cam(x,y)=(%.3f,%.3f) body_err=(%.3f,%.3f) center_dist=%.3f scaled=%.3f vx=%.3f vy=%.3f",
            x, y, body_err_x, body_err_y, center_dist, scaled_dist, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);

        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void centeringPayload(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float x, float y, float min_center_time, float max_center_pitch, float max_center_roll, float hover_pitch, float hover_roll, std::string recovery_method, std::string centering_setpoint_mode) {
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
     * - dropAlt: float value representing the altitude to descend to take the payload.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - status: reference to a boolean indicating whether centering is successful or not. Modified by the function (true if successful, false otherwise)
     */

    RCLCPP_INFO(node->get_logger(), "------------- Starting to Center Payload --------------");
    
    status = false;
    rclcpp::Time start_time = node->now();
    rclcpp::Time last_detected_time = node->now(); // Waktu terakhir payload terdeteksi
    const float dropAlt = 0.6;

    // const float speed_xy = 0.4;
    float roll, pitch; bool detected = false;
    geometry_msgs::msg::PoseStamped avg_payload_pose; int avg_sample = 1; int buffer_size = 4;
    std::vector<geometry_msgs::msg::PoseStamped> cache;
    geometry_msgs::msg::PoseStamped last_last_payload_pose = posee;
    geometry_msgs::msg::TwistStamped lastVel,velocity_msg;
    geometry_msgs::msg::Point error, err_i, speed_local, payload_offset;
    float payload_to_left, payload_to_front, payload_to_right, payload_to_back;
    lastVel = node->getCurrentVelocity();

    err_i.x = 0;
    err_i.y = 0;
    float speed_i = 0.0;

    payload_offset.x = -limit(node->getLastPayloadPose().pose.position.x,0.1);
    payload_offset.y = -limit(node->getLastPayloadPose().pose.position.y,0.1);

    rclcpp::Time last_center_time = rclcpp::Time(0,0);
    bool centered = false;

    if(!payloadDetected(node))
    {
        if(node->getLastNonZeroPosePayload().pose.position.x != 0.0 && node->getLastNonZeroPosePayload().pose.position.y != 0.0 && node->getLastNonZeroPosePayload().pose.position.z != 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Tadi sempet detect tapi sekarang ngilang, coba ke pose deteksi terakhir ya...");

            // Get current pose
            geometry_msgs::msg::PoseStamped last_drone_nonzero_pose = node->getDroneNonZeroPosePayload();

            // Get current payload pose in local frame
            geometry_msgs::msg::PoseStamped payload_pose = node->getLastNonZeroPosePayload();
            error = -node->getLastNonZeroPosePayload().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, last_drone_nonzero_pose.pose.orientation);

            geometry_msgs::msg::PoseStamped payload_local_frame = last_drone_nonzero_pose;
            payload_local_frame.pose.position.x = last_drone_nonzero_pose.pose.position.x + speed_local.x;
            payload_local_frame.pose.position.y = last_drone_nonzero_pose.pose.position.y + speed_local.y;

            moveToPoint(node, rate, posee, payload_local_frame.pose, 0.25, acc*2.0);
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Kayaknya ini payload nya kemajuan, coba maju dikit ya...");
            
            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
            geometry_msgs::msg::Point maju;
            float heading = getHeading(curr_pose.pose.orientation);
            maju.x = x;
            maju.y = y;
            maju = rotatePoint(maju, heading);
            
            curr_pose.pose.position.x += maju.x;
            curr_pose.pose.position.y += maju.y;
            moveToPoint(node, rate, posee, curr_pose.pose, 0.25, acc*2.0, true, false);

        }
    }

    while (rclcpp::ok()) {
        if(payloadDetected(node)){
            // Filters out old payload data to avoid drift
            if(node->getLastPayloadPose().pose.position.x == node->getCurrentPosePayload().pose.position.y && node->getLastPayloadPose().pose.position.x == node->getCurrentPosePayload().pose.position.y && node->getLastPayloadPose().pose.position.x == node->getCurrentPosePayload().pose.position.y) {
                velocity_msg = zero(velocity_msg);
                RCLCPP_INFO(node->get_logger(), "old payload position data, sending zero velocity to avoid drifting");
                node->publishLocalVelocity(velocity_msg);
                continue;
            }   

            // Get current pose
            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
            // last_last_payload_pose=curr_pose;

            // Get current payload pose in local frame
            geometry_msgs::msg::PoseStamped payload_pose = node->getCurrentPosePayload();
            error = -node->getCurrentPosePayload().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, curr_pose.pose.orientation);
            
            geometry_msgs::msg::PoseStamped payload_local_frame = curr_pose;
            payload_local_frame.pose.position.x = curr_pose.pose.position.x + speed_local.x;
            payload_local_frame.pose.position.y = curr_pose.pose.position.y + speed_local.y;
            payload_local_frame.pose.position.z = curr_pose.pose.position.z - speed_local.z;
            
            if(recovery_method == "lidar")
            {
                sensor_msgs::msg::LaserScan curr_scan = node->getCurrentLaserScan();
                payload_to_front = curr_scan.ranges[0] - error.x;
                payload_to_left  = curr_scan.ranges[(int)curr_scan.ranges.size()/4] - error.y;
                payload_to_back  = curr_scan.ranges[(int)curr_scan.ranges.size()/2] + error.x;
                payload_to_right = curr_scan.ranges[(int)curr_scan.ranges.size()*0.75] + error.y;
            }

            // ini harusnya ngelog angka yang sama terus, kalo beda beda jauh ada yang ga beres
            RCLCPP_INFO(node->get_logger(), "PAYLOAD LOCAL | x: %.2f y: %.2f z: %.2f",payload_local_frame.pose.position.x,payload_local_frame.pose.position.y,payload_local_frame.pose.position.z);

            // ======================= Filters out spikes to avoid jitters during centering (low pass filter by using moving average)
            // if(avg_sample <= 1) {avg_payload_pose = payload_local_frame; avg_sample++; cache.push_back(payload_local_frame);}
            // else if(avg_sample < buffer_size)
            // {
            //     avg_payload_pose.pose.position.x += (payload_local_frame.pose.position.x) * ((avg_sample - 1)/ avg_sample);
            //     avg_payload_pose.pose.position.y += (payload_local_frame.pose.position.y) * ((avg_sample - 1)/ avg_sample);
            //     avg_payload_pose.pose.position.z += (payload_local_frame.pose.position.z) * ((avg_sample - 1)/ avg_sample);
            //     cache.push_back(payload_local_frame);
            //     avg_sample++;
            // }
            // else
            // {
            //     cache.push_back(payload_local_frame);
            //     cache.erase(cache.begin());
            //     avg_payload_pose = centroid(cache);
            // }
            // RCLCPP_INFO(node->get_logger(), "PAYLOAD LOCAL AVG | x: %.2f y: %.2f z: %.2f", avg_payload_pose.pose.position.x, avg_payload_pose.pose.position.y, avg_payload_pose.pose.position.z);
            // const float threshold = 0.1; //trial dulu biar tau cocok nya brp
            // ========================================================================================================================================
            
            // limit speed and acceleration
            err_i.x += speed_local.x*speed_i;
            err_i.y += speed_local.y*speed_i;
            speed_local = speed_local * speed_xy;
            speed_local = speed_local + err_i;
            velocity_msg = cast(speed_local);
            velocity_msg = limitDelta(maxAccel/RATE, velocity_msg ,lastVel);
            velocity_msg = limit(0.4, velocity_msg);
            // velocity_msg.twist.linear.z = -limit(0.4, node->getCurrentRangefinder().range - dropAlt);
            
            velocity_msg.twist.linear.z = 0.0;
            
            lastVel = velocity_msg;
            
            // geometry_msgs::msg::PoseStamped pose_msg = avg_payload_pose;
            geometry_msgs::msg::PoseStamped pose_msg = payload_local_frame;
            pose_msg.pose.position.z = posee.pose.position.z;
            pose_msg.pose.orientation = posee.pose.orientation;
            
            if (centering_setpoint_mode == "velocity") {node->publishLocalVelocity(velocity_msg);}
            else if (centering_setpoint_mode == "position") {node->publishLocalPosition(pose_msg);}
            
            roll = getRoll(curr_pose.pose.orientation);
            pitch = getPitch(curr_pose.pose.orientation);
            
            // RCLCPP_INFO(node->get_logger(), "range: %f | dropAlt: %f", node->getCurrentRangefinder().range,dropAlt);
            // RCLCPP_INFO(node->get_logger(), "x: %.2f y: %.2f d: %.2f r: %.1f p: %.1f, cp: %d", payload_pose.pose.position.x,payload_pose.pose.position.y, dist(payload_pose.pose.position, -payload_offset), fabs(roll), fabs(pitch));
            RCLCPP_INFO(node->get_logger(), "Centering... dist: %f, acc: %f, roll+hvr_roll: %f, max_roll: %f, pitch+hvr_pitch: %f, max_pitch: %f", dist(error.x, error.y, 0.0, 0.0), acc, fabs(roll*180/3.14) - hover_roll, max_center_roll, fabs(pitch*180/3.14) - hover_pitch , max_center_pitch);
            
            // if (isNear(payload_local_frame.pose.position, curr_pose.pose.position, acc) && (fabs(roll*180/3.14) - hover_roll<= max_center_roll) && (fabs(pitch*180/3.14) - hover_pitch <= max_center_pitch) && !centered) {
            if (dist(error.x, error.y, 0.0, 0.0) <= acc && (fabs(roll*180/3.14) - hover_roll<= max_center_roll) && (fabs(pitch*180/3.14) - hover_pitch <= max_center_pitch) && !centered) {
                RCLCPP_INFO(node->get_logger(), "------------ Payload centered ------------");
                velocity_msg = zero(velocity_msg);
                node->publishLocalVelocity(velocity_msg);
                rclcpp::spin_some(node); rate.sleep();
                status = true; posee = curr_pose;
                last_center_time = node->now();
                centered = true;
            }
            // else if(!(isNear(payload_local_frame.pose.position, curr_pose.pose.position, acc) && (fabs(roll*180/3.14) - hover_roll <= max_center_roll) && (fabs(pitch*180/3.14) - hover_pitch <= max_center_pitch)))
            else if(!(dist(error.x, error.y, 0.0, 0.0) <= acc*1.25) && centered)
            {
                RCLCPP_INFO(node->get_logger(), "Center cancelled. dist: %f, acc: %f, roll+hvr_roll: %f, max_roll: %f, pitch+hvr_pitch: %f, max_pitch: %f", dist(error.x, error.y, 0.0, 0.0), acc, fabs(roll*180/3.14) - hover_roll, max_center_roll, fabs(pitch*180/3.14) - hover_pitch , max_center_pitch);
                centered = false;
            }
            
            last_detected_time = node->now();
            last_last_payload_pose = payload_local_frame;
            last_last_payload_pose.pose.position.z = curr_pose.pose.position.z;
            node->setLastPayloadPose(payload_pose);

            if(centered){RCLCPP_INFO(node->get_logger(), "payload centered for %.3f", (node->now() - last_center_time).seconds());}

            if(centered && node->now() - last_center_time >= rclcpp::Duration::from_seconds(min_center_time))
            {
                break;
            }
        } else {
            auto now = node->now(); detected = false;
            RCLCPP_INFO(node->get_logger(), "OII PAYLOAD MANA : %.2f", now.seconds());

            err_i.x = 0;
            err_i.y = 0;

            // --- KALO 7 detik galiat apa apa bahkan setelah recovery
            if ((now - last_detected_time).seconds() > 10 && (last_last_payload_pose.pose.position.x != 0.0 && last_last_payload_pose.pose.position.y != 0.0 && last_last_payload_pose.pose.position.z != 0.0)) {
                RCLCPP_INFO(node->get_logger(), "--- NOOO YAUDAH LANJUT AJA -----");
                status = true;
                break;
            } else if(last_last_payload_pose.pose.position.x != 0.0 && last_last_payload_pose.pose.position.y != 0.0 && last_last_payload_pose.pose.position.z != 0.0) {
                RCLCPP_INFO(node->get_logger(), "--- COBA KU SEND POSISI AWAL YAA, BISMILLAH ---");
                if(recovery_method == "local_pose")
                {
                    moveToPoint(node, rate, posee, last_last_payload_pose.pose, 0.25, acc*2.0);
                    
                    // for (int i=0; i<5; i++) {
                    //     rclcpp::spin_some(node); rate.sleep();
                    // }

                    holdPosition(node, rate, posee, 0.5);

                    if(!payloadDetected(node))
                    {
                        if(node->getLastNonZeroPosePayload().pose.position.x != 0.0 && node->getLastNonZeroPosePayload().pose.position.y != 0.0 && node->getLastNonZeroPosePayload().pose.position.z != 0.0)
                        {
                            RCLCPP_INFO(node->get_logger(), "Masi ga liat, coba ke nonzero payload pose terakhir ya...");

                            // Get current pose
                            geometry_msgs::msg::PoseStamped last_drone_nonzero_pose = node->getDroneNonZeroPosePayload();

                            // Get current payload pose in local frame
                            geometry_msgs::msg::PoseStamped payload_pose = node->getLastNonZeroPosePayload();
                            error = -node->getLastNonZeroPosePayload().pose.position;
                            error = reflect(error);
                            speed_local = point_rotation_by_quaternion(error, last_drone_nonzero_pose.pose.orientation);

                            geometry_msgs::msg::PoseStamped payload_local_frame = last_drone_nonzero_pose;
                            payload_local_frame.pose.position.x = last_drone_nonzero_pose.pose.position.x + speed_local.x;
                            payload_local_frame.pose.position.y = last_drone_nonzero_pose.pose.position.y + speed_local.y;

                            moveToPoint(node, rate, posee, payload_local_frame.pose, 0.25, acc*2.0);
                        }
                        if(!payloadDetected(node)) {
                            RCLCPP_INFO(node->get_logger(), "--- JIR MASI GA KELIATAN, KAYAKNYA GPS TOLOL, COBA MUNDUR LAHH ---");
                            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
                            geometry_msgs::msg::Point maju;
                            float heading = getHeading(curr_pose.pose.orientation);
                            maju.x = -0.5;
                            maju.y = 0.0;
                            maju = rotatePoint(maju, heading);
                            
                            curr_pose.pose.position.x += maju.x;
                            curr_pose.pose.position.y += maju.y;
                            moveToPoint(node, rate, posee, curr_pose.pose, 0.25, acc*2.0, true, false);
                        }

                    }
                }
                else if(recovery_method == "lidar")
                {
                    RCLCPP_INFO(node->get_logger(), "Trying to recover using lidar ranges, %f, %f, %f, %f", payload_to_front, payload_to_left, payload_to_back, payload_to_right);
                    holdAtLidarRanges(node, rate, posee, (payload_to_front != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_front) && payload_to_front) ? payload_to_front : -1.0, (payload_to_left != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_left) && payload_to_left) ? payload_to_left : -1.0, (payload_to_back != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_back) && payload_to_back) ? payload_to_back : -1.0, (payload_to_right != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_right) && payload_to_right) ? payload_to_right : -1.0, false, false);
                }
            }
        }

        rclcpp::spin_some(node); rate.sleep();
    }

    posee = node->getCurrentLocalPose();
    node->resetNonzeroPayloadPose();
}

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float angle, float speed, float tolerance) {
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
    float last_x, last_y, distance, progress=0.0; int i =0;
    geometry_msgs::msg::PoseStamped cmd_pose = node->getCurrentLocalPose();
    last_x = cmd_pose.pose.position.x;
    last_y = cmd_pose.pose.position.y;
    distance = dist(cmd_pose.pose.position,x,y);
    geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
    while(rclcpp::ok() && !isNear(curr_pose.pose.position, x, y, tolerance)){
        RCLCPP_INFO(node->get_logger(), "dist: %f goto progress: %f",distance,progress);
        cmd_pose.pose.position.x = last_x+progress*(x-last_x);
        cmd_pose.pose.position.y = last_y+progress*(y-last_y);
        cmd_pose.pose.position.z = z + (node->getCurrentLocalPose().pose.position.z - node->getCurrentRangefinder().range);
        progress = (progress<1)? progress+1/((distance/speed)*20.0): 1.0;
        node->publishLocalPosition(cmd_pose);
        rclcpp::spin_some(node);
        rate.sleep();
        curr_pose = node->getCurrentLocalPose();
    }
    
    posee = node->getCurrentLocalPose();
}

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, geometry_msgs::msg::Pose target, float speed, float tolerance, bool allow_centering_payload, bool allow_centering_ember, bool allow_centering_artag, bool disable_z_lock) {
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
     * returns:moveToPoint
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    // float last_x, last_y, distance, progress=0.0; int i =0;
    geometry_msgs::msg::PoseStamped cmd_pose = node->getCurrentLocalPose();
    cmd_pose.pose = target;
    
    // last_x = cmd_pose.pose.position.x;
    // last_y = cmd_pose.pose.position.y;
    // distance = dist(cmd_pose.pose.position, target.position);
    geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();

    float distance_to_target = dist(posee.pose.position, target.position);
    float vel_z = (target.position.z - node->getCurrentRangefinder().range) / dist(curr_pose.pose.position, target.position);
    bool alt_reached = false;

    while(rclcpp::ok() && !isNear(curr_pose.pose.position, target.position, tolerance)){
        float distance = dist(curr_pose.pose.position, target.position);
        RCLCPP_INFO(node->get_logger(), "moving..., dist: %f",distance);
        curr_pose = node->getCurrentLocalPose();

        geometry_msgs::msg::TwistStamped vel;
        vel.twist.linear.x = (target.position.x - curr_pose.pose.position.x);
        vel.twist.linear.y = (target.position.y - curr_pose.pose.position.y);
        
        vel = limit(speed, vel);
        vel.twist.linear.z = disable_z_lock ? (std::abs(target.position.z - node->getCurrentRangefinder().range) > 0.15 && alt_reached ? vel_z : 0.8*(target.position.z - node->getCurrentRangefinder().range)) : 0.0;
        alt_reached = std::abs(target.position.z - node->getCurrentRangefinder().range) > 0.15 && !alt_reached;
        node->publishLocalVelocity(vel);
        
        rclcpp::spin_some(node);
        rate.sleep();
        
        if(payloadDetected(node) && allow_centering_payload)
        {
            RCLCPP_INFO(node->get_logger(), "Payload Spotted");
            geometry_msgs::msg::TwistStamped vel;
            vel = zero(vel);
            node->publishLocalVelocity(vel);
            rclcpp::spin_some(node); rate.sleep();
            break;
        }
        if(emberDetected(node) && allow_centering_ember)
        {
            RCLCPP_INFO(node->get_logger(), "Ember Spotted");
            geometry_msgs::msg::TwistStamped vel;
            vel = zero(vel);
            node->publishLocalVelocity(vel);
            rclcpp::spin_some(node); rate.sleep();
            break;
        }
        if(artagDetected(node) && allow_centering_artag)
        {
            RCLCPP_INFO(node->get_logger(), "ArTag Spotted during recovery!");
            geometry_msgs::msg::TwistStamped vel;
            vel = zero(vel);
            node->publishLocalVelocity(vel);
            rclcpp::spin_some(node); rate.sleep();
            break;
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "NYAMPEEEE");
    geometry_msgs::msg::TwistStamped vel;
    vel = zero(vel);
    node->publishLocalVelocity(vel);
    rclcpp::spin_some(node); rate.sleep();
    posee = node->getCurrentLocalPose();
}

void landingFaux(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee){
    RCLCPP_INFO(node->get_logger(), "===================== Turun ===================");
    geometry_msgs::msg::TwistStamped vel;
    bool was_descending = false, trying_descending = false;

    // float heading;
    // geometry_msgs::msg::PoseStamped last_last_payload_pose = posee;
    // geometry_msgs::msg::TwistStamped lastVel, vel;
    // geometry_msgs::msg::Point error, err_i, speed_local,payload_offset;

    // lastVel = node->getCurrentVelocity();

    // err_i.x = 0;
    // err_i.y = 0;
    // float speed_i = 0.0;
    // float test = 0;

    // payload_offset.x = -limit(node->getLastPayloadPose().pose.position.x,0.1);
    // payload_offset.y = -limit(node->getLastPayloadPose().pose.position.y,0.1);

    
    while (rclcpp::ok()){
        // heading = getHeading(posee.pose.orientation);
        // error = -node->getCurrentPosePayload().pose.position-payload_offset;
        // error = reflect(error);
        // speed_local = rotatePoint(error,heading);

        // err_i.x += speed_local.x*speed_i;
        // err_i.y += speed_local.y*speed_i;

        // speed_local = speed_local * 0.275;
        // speed_local = speed_local + err_i;

        // vel = cast(speed_local);
        // vel = limitDelta(0.3/RATE, vel ,lastVel);
        // vel = limit(0.4, vel);

        // vel.twist.linear.z = -0.625;
        // vel.twist.linear.x = posee.pose.position.x - node->getCurrentLocalPose().pose.position.x;
        // vel.twist.linear.y = posee.pose.position.y - node->getCurrentLocalPose().pose.position.y;
        // vel.twist.linear.z = -0.725;
        vel.twist.linear.x = 0.0;
        vel.twist.linear.y = 0.0;
        vel.twist.linear.z = -0.69;

        node->publishLocalVelocity(vel);
        if (node->getCurrentVelocity().twist.linear.z < -0.05 && !trying_descending)  { RCLCPP_INFO(node->get_logger(), "Descend started");trying_descending = true;}
        if (node->getCurrentVelocity().twist.linear.z < -0.35)                        { RCLCPP_INFO(node->get_logger(), "Descending...")  ;was_descending = true;}
        if (was_descending && payloadDetected(node))                                  { node->setLastPayloadPose(node->getCurrentPosePayload());}
        if (abs(node->getCurrentVelocity().twist.linear.z) < 0.055 && was_descending) { RCLCPP_INFO(node->get_logger(), "Landed"); break;}
        rclcpp::spin_some(node);rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Latest offset: %f, %f",node->getLastPayloadPose().pose.position.x,node->getLastPayloadPose().pose.position.y);
    posee = node->getCurrentLocalPose();
}

void landingFauxPose(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee){
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
    RCLCPP_INFO(node->get_logger(), "===================== Turun ====================");

    geometry_msgs::msg::PoseStamped pose = posee;

    float descend_speed = 0.4; 
    bool was_descending=false, trying_descending = false;
    
    while(rclcpp::ok()){
        pose.pose.position.x = posee.pose.position.x;
        pose.pose.position.y = posee.pose.position.y;
        pose.pose.position.z -= descend_speed/RATE;
        node->publishLocalPosition(pose);

        if (node->getCurrentVelocity().twist.linear.z<-0.05 && !trying_descending){  RCLCPP_INFO(node->get_logger(),"Descending..."); trying_descending = true; }
        if (node->getCurrentVelocity().twist.linear.z<-0.3){ was_descending =true;}
        if (abs(node->getCurrentVelocity().twist.linear.z)<0.05 && was_descending){RCLCPP_INFO(node->get_logger(), "Landed"); posee = node->getCurrentLocalPose(); break;}

        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void holdPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration){
    /**
     * Hold current position for a specified duration.
     * For ArduPilot GUIDED mode: Stream setpoints to maintain position
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - duration: float value representing the duration in seconds to hold the position.
     */
    RCLCPP_INFO(node->get_logger(), "Holding position for %.1f seconds...", duration);
    
    rclcpp::Time start_time = node->now();
    rclcpp::Rate fast_rate(10.0); // 10Hz for ArduPilot GUIDED
    
    // Get current position to hold
    geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    
    while (rclcpp::ok()) {
        rclcpp::Time current_time = node->now();
        double elapsed_time = (current_time - start_time).seconds();
        
        // ArduPilot: Stream setpoint to maintain position
        hold_pose.header.stamp = node->now();
        node->publishLocalPosition(hold_pose);
        
        if (fmod(elapsed_time, 1.0) < 0.05) { // Log every ~1 second
            RCLCPP_INFO(node->get_logger(), "Holding... %.1f/%.1f seconds", elapsed_time, duration);
        }
        
        if (elapsed_time > duration){break;}

        rclcpp::spin_some(node); 
        fast_rate.sleep(); // Use fast rate for continuous streaming
    }
    
    RCLCPP_INFO(node->get_logger(), "Hold position complete");
    posee = node->getCurrentLocalPose();
}

void waitForPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, mavros_msgs::msg::GlobalPositionTarget raw, double tolerance, bool allow_centering, float center_dist) {
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
    sensor_msgs::msg::NavSatFix init_global = node->getCurrentGPSPosition();

    while (rclcpp::ok()) {
        double dlat = raw.latitude - node->getCurrentGPSPosition().latitude;
        double dlon = raw.longitude - node->getCurrentGPSPosition().longitude;
        double dalt = raw.altitude - node->getCurrentGPSPosition().altitude;

        double distance = std::sqrt(dlat * dlat + dlon * dlon);
        double dist_m = latLonToMeter(init_global.latitude, init_global.longitude, node->getCurrentGPSPosition().latitude, node->getCurrentGPSPosition().longitude);

        RCLCPP_INFO(node->get_logger(), "%.4f - CURRR DISTANCEE: %f, meter: %.2f, payload: %.2f", distance, dist_m, node->getCurrentPosePayload().pose.position.x);

        if (distance < tolerance) {
            RCLCPP_INFO(node->get_logger(), "Reached target positionnnn");
            break;
        }

        if(payloadDetected(node) && allow_centering){
            RCLCPP_INFO(node->get_logger(), "Payload spotted");
            // distancee_to_payload = sqrt((pose.pose.position.x - init_pose.pose.position.x) * (pose.pose.position.x - init_pose.pose.position.x) + (pose.pose.position.y - init_pose.pose.position.y) * (pose.pose.position.y - init_pose.pose.position.y));
            break;
        }

        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void sendGlobalRaw(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering, float center_dist, float tolerance) {
    /**
     * Send a raw global position target to the drone and wait until the drone reaches the target position.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - latt: float value representing the target latitude in degrees.
     * - lonn: float value representing the target longitude in degrees.
     * - altt: float value representing the target altitude in meters.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    mavros_msgs::msg::GlobalPositionTarget raw;
    raw.header.stamp = node->now();
    raw.header.frame_id = "map";
    raw.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    raw.type_mask = 
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW_RATE;

    raw.latitude = latt;
    raw.longitude = lonn;
    raw.altitude = altt;

    RCLCPP_INFO(node->get_logger(), "Sending target: [%.8f, %.8f, %.2f]", latt, lonn, altt);

    for (int i = 0; i < 20; i++) {
        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    waitForPosition(node, rate, raw, tolerance, allow_centering, center_dist);
    // waitForPosition(node, rate, latt, lonn, altt, );

    posee = node->getCurrentLocalPose();
}

void sendGlobalRawAsync(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering, float center_dist) {
    /**
     * Send a raw global position target to the drone and wait until the drone reaches the target position.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - latt: float value representing the target latitude in degrees.
     * - lonn: float value representing the target longitude in degrees.
     * - altt: float value representing the target altitude in meters.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    mavros_msgs::msg::GlobalPositionTarget raw;
    raw.header.stamp = node->now();
    raw.header.frame_id = "map";
    raw.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    raw.type_mask = 
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW_RATE;

    raw.latitude = latt;
    raw.longitude = lonn;
    raw.altitude = altt;

    RCLCPP_INFO(node->get_logger(), "Sending target: [%.8f, %.8f, %.2f]", latt, lonn, altt);

    for (int i = 0; i < 1; i++) {
        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // waitForPosition(node, rate, raw, 0.0000025, allow_centering, center_dist);
    // waitForPosition(node, rate, latt, lonn, altt, );

    posee = node->getCurrentLocalPose();
}

void sendGlobalRawFromXYZ(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, LatLonAlt zero, double zero_hdg, float x, float y, float rel_alt, bool allow_centering, float center_dist) {
    mavros_msgs::msg::GlobalPositionTarget raw = create_globalpos_from_xyz(x, y, rel_alt, zero, zero_hdg);
    raw.header.stamp = node->now();
    RCLCPP_INFO(node->get_logger(), "Sending target: [%.8f, %.8f, %.2f]", raw.latitude, raw.longitude, raw.altitude);

    for (int i = 0; i < 20; i++) {
        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // waitForPosition(node, rate, raw.latitude, raw.longitude, raw.altitude);

    posee = node->getCurrentLocalPose();
}

void correctAltitude(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, double height, float &corrected_rel_alt, double max_speed) {
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
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     * - The correct relative altitude after adjusting the drone's position. ready to be used for sendGlobalRaw.
     */
    RCLCPP_INFO(node->get_logger(), "fixing altitude..");
    geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
    rclcpp::Time last_correct_time = rclcpp::Time(0,0);
    bool hit = false;

    const float MAX_SPEED = max_speed;

    const float P = 1.0;
    const float I = 0.0;
    const float D = 0.0;

    float total_error = 0;
    float last_error = 0;
    
    while
    (   
        !hit ||
        (rclcpp::ok() && abs(node->getCurrentRangefinder().range-height)>0.05) ||
        (hit && node->now() - last_correct_time <= rclcpp::Duration::from_seconds(2.0))
    )   
    {
        total_error += height - node->getCurrentRangefinder().range;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = limit(MAX_SPEED, P*(height - node->getCurrentRangefinder().range) + I*(total_error) + D*(last_error - (height - node->getCurrentRangefinder().range)));

        RCLCPP_INFO(node->get_logger(), "height: %f current_alt.range: %f",height,node->getCurrentRangefinder().range);
        last_error = (height - node->getCurrentRangefinder().range);
        node->publishLocalVelocityUnstamped(cmd_vel);
        rclcpp::spin_some(node);
        rate.sleep();

        if(!(abs(node->getCurrentRangefinder().range-height)>0.05) && !hit)
        {
            last_correct_time = node->now();
            hit = true;
            RCLCPP_INFO(node->get_logger(), "HIT!, last_correct_time: %f",last_correct_time.seconds());
        }
        else if((abs(node->getCurrentRangefinder().range-height)>0.05))
        {
            RCLCPP_INFO(node->get_logger(), "left the tolerance zone, resetting");
            hit = false;
        }

    }
    corrected_rel_alt = node->getCurrentRelAlt().data;
    posee = node->getCurrentLocalPose();
}

void controlServo(const std::shared_ptr<DroneController>&node, int channel, int pwm) {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    // MAV_CMD_DO_SET_SERVO == 183
    request->command = 183; request->param1 = channel; request->param2 = pwm;
    request->param3 = 0; request->param4 = 0; request->param5 = 0; request->param6 = 0; request->param7 = 0;

    auto future_result = node->servo_(request);

    if (hasReplied(node,future_result)) { auto result = future_result.get();
        if (result->success) { RCLCPP_INFO(node->get_logger(), "Channel: %d servo signal sent", channel);} 
        else                 { RCLCPP_ERROR(node->get_logger(), "Servo signal failed.");}
    } else {RCLCPP_ERROR(node->get_logger(), "Failed to receive response");}
}

void controlServoRepeated(const std::shared_ptr<DroneController>& node, int channel, int pwm, int repeat) {
    for (int i = 0; i < repeat; ++i) {
        controlServo(node, channel, pwm);
    }
}

void pushMission(const std::shared_ptr<DroneController>&node, std::vector<mavros_msgs::msg::Waypoint> waypoints) {
    waypoints.insert(waypoints.begin() + 0, waypoints[0]); // I dont know why but the first waypoint is always skipped when pushing mission, so I duplicate it
    RCLCPP_INFO(node->get_logger(), "Pushing mission of size %d", waypoints.size());
    auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    request->start_index = 0;
    request->waypoints = waypoints;

    auto future_result = node->pushMission_(request);

    if (hasReplied(node,future_result)) { auto result = future_result.get();
        if (result->success) { RCLCPP_INFO(node->get_logger(), "Mission pushed");} 
        else                 { RCLCPP_ERROR(node->get_logger(), "failed to push mission");}
    } else {RCLCPP_ERROR(node->get_logger(), "Failed to receive response");}
}

void clearMission(const std::shared_ptr<DroneController>&node) {
    auto request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();

    auto future_result = node->clearMission_(request);

    if (hasReplied(node,future_result)) { auto result = future_result.get();
        if (result->success) { RCLCPP_INFO(node->get_logger(), "Mission cleared"); node->reset_wp_();} 
        else                 { RCLCPP_ERROR(node->get_logger(), "failed to clear mission");}
    } else {RCLCPP_ERROR(node->get_logger(), "Failed to receive response");}
}

void waitForWP(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, int seq) {
    while (rclcpp::ok()) {
        if(node->current_wp_() >= seq){
            RCLCPP_INFO(node->get_logger(), "wp reached! :)");
            break;
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void arm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate) {
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;
    node->waitForArmingService();

    if (node->isArmingServiceReady()) {
        auto arm_result = node->arm_(arm_request);
        if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = arm_result.get();
            if (response->success) {
                RCLCPP_INFO(node->get_logger(), "Vehicle armed");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Arming rejected by FCU");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
        }
    }
    rate.sleep();
}

void disarm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate) {
    auto disarm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    disarm_request->value = false;
    node->waitForArmingService();

    if (node->isArmingServiceReady()) {
        auto disarm_result = node->arm_(disarm_request);
        if (rclcpp::spin_until_future_complete(node, disarm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = disarm_result.get();
            if (response->success) {
                RCLCPP_INFO(node->get_logger(), "Vehicle disarmed");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Disarm rejected by FCU");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to disarm vehicle");
        }
    }
    rate.sleep();
}

void moveWithLaser(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float max_speed, float tolerance, bool allow_centering_payload, bool allow_centering_ember, float koreksi_max_speed, float timeout) {
    /**
     * Listen, I did so many one liner in this function, I'd advise you to just ask me (palel) if you want to understand it. maybe in the future
     * ill create a better comment snippet for this function, but for now. this will do.
     */
    float P_maju = 1.0;
    float I_maju = 0.0;
    float D_maju = 0.0;
    float total_error_maju = 0;
    float last_error_maju = 0;
    
    float P_koreksi = 1.0;
    float I_koreksi = 0.0;
    float D_koreksi = 0.0;
    float total_error_koreksi = 0;
    float last_error_koreksi = 0;

    geometry_msgs::msg::TwistStamped cmd_vel;
    geometry_msgs::msg::PoseStamped vel_pose;
    sensor_msgs::msg::LaserScan start_scan = node->getCurrentLaserScan();
     
    float x_start = start_scan.ranges[0];
    float y_start = start_scan.ranges[(int)start_scan.ranges.size()/4];
    
    float x_2_edge = (x_start == std::numeric_limits<float>::infinity() || std::isnan(x_start)) ? 1.25 : std::abs(x_start - x);
    float y_2_edge = (y_start == std::numeric_limits<float>::infinity() || std::isnan(y_start)) ? 1.25 : std::abs(y_start - y);
    
    float front = start_scan.ranges[0];
    float left =  start_scan.ranges[(int)start_scan.ranges.size()/4];
    float back =  start_scan.ranges[(int)start_scan.ranges.size()/2];
    float right = start_scan.ranges[(int)start_scan.ranges.size()*0.75];

    float last_front = 4.0;
    float last_left = 4.0;
     
    front = front == std::numeric_limits<float>::infinity() ? 4.0f : front;
    left = left == std::numeric_limits<float>::infinity() ? 2.1f : left;
    back = back == std::numeric_limits<float>::infinity() ? 4.0f : back;
    right = right == std::numeric_limits<float>::infinity() ? 2.1f : right;
     
    float vel_z = (z - node->getCurrentRangefinder().range) / sqrt(x*x + y*y);
     
    //  float max_time = timeout == -1 ? sqrt(x*x + y*y) / (max_speed*0.25) : timeout;
    float max_time = timeout == -1 ? 9999.9999 : timeout;
    rclcpp::Time start_time = node->now();

    bool alt_reached = false;
     
    geometry_msgs::msg::Point target = posee.pose.position + rotatePoint(x, y, getHeading(posee.pose.orientation));

    while(rclcpp::ok())
    {
        float drone_heading = getHeading(posee.pose.orientation);
        sensor_msgs::msg::LaserScan curr_scan = node->getCurrentLaserScan();
        front = curr_scan.ranges[0] != std::numeric_limits<float>::infinity() ? curr_scan.ranges[0] : front;
        left =  (curr_scan.ranges[(int)curr_scan.ranges.size()/4] != std::numeric_limits<float>::infinity() && !std::isnan(curr_scan.ranges[(int)curr_scan.ranges.size()/4])) ? std::min(2.1f, curr_scan.ranges[(int)curr_scan.ranges.size()/4]) : left;
        back =  curr_scan.ranges[(int)curr_scan.ranges.size()/2] != std::numeric_limits<float>::infinity() ? curr_scan.ranges[(int)curr_scan.ranges.size()/2] : back;
        right = (curr_scan.ranges[(int)curr_scan.ranges.size()*0.75] != std::numeric_limits<float>::infinity() && !std::isnan(curr_scan.ranges[(int)curr_scan.ranges.size()*0.75])) ? std::min(2.1f, curr_scan.ranges[(int)curr_scan.ranges.size()*0.75]) : right;
        float err_x = x ? (front != std::numeric_limits<float>::infinity() ? front - x_2_edge : 7.5): 0.0;
        float err_y = y ? (left != std::numeric_limits<float>::infinity() ? left - y_2_edge : 7.5): 0.0;
        
        vel_pose.pose.position.x = x ? limit(max_speed, P_maju*err_x + I_maju*total_error_maju + D_maju*last_error_maju) : (front + back <= 2.85f ? limit(koreksi_max_speed, P_koreksi*(front - back) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (back == 2.1f && front != 2.1f ? limit(koreksi_max_speed, P_koreksi*(front - 1.25 ) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (front == 2.1f & back != 2.1f ? limit(koreksi_max_speed, P_koreksi*(1.25 - back) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : 0.0)));
        vel_pose.pose.position.y = y ? limit(max_speed, P_maju*err_y + I_maju*total_error_maju + D_maju*last_error_maju) : (left + right <= 2.85f ? limit(koreksi_max_speed, P_koreksi*(left - right) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (right == 2.1f && left != 2.1f ? limit(koreksi_max_speed, P_koreksi*(left - 1.25  ) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (left == 2.1f & right != 2.1f ? limit(koreksi_max_speed, P_koreksi*(1.25 - right) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : 0.0)));
        
        RCLCPP_INFO(node->get_logger(), "RANGE | front: %f, left: %f, back : %f, right : %f, err_x: %f, err_y: %f, velx: %f, vely: %f", front, left, back, right, err_x, err_y, vel_pose.pose.position.x, vel_pose.pose.position.y);
        
        float velx_before_rotate = vel_pose.pose.position.x;
        float vely_before_rotate = vel_pose.pose.position.y;
        
        vel_pose.pose.position = rotatePoint(vel_pose.pose.position, drone_heading);
        cmd_vel = cast(vel_pose.pose.position);
        cmd_vel.twist.linear.z = std::abs(z - node->getCurrentRangefinder().range) > 0.15 && alt_reached ? vel_z : 0.8*(z - node->getCurrentRangefinder().range);
        alt_reached = std::abs(z - node->getCurrentRangefinder().range) > 0.15 && !alt_reached;
        
        // RCLCPP_INFO(node->get_logger(), "LOG Z %f | mode: %s, twist: %f, vel_z: %f, rangefinder: %f", z, std::abs(z - node->getCurrentRangefinder().range) > 0.15 ? "vel" : "hold", cmd_vel.twist.linear.z, vel_z, node->getCurrentRangefinder().range);
        // RCLCPP_INFO(node->get_logger(), "bfr rotate: (%f , %f). afr rotate: (%f, %f)", velx_before_rotate, vely_before_rotate, vel_pose.pose.position.x, vel_pose.pose.position.y);
        // RCLCPP_INFO(node->get_logger(), "MOVING WITH LASER | dist2target: %f, time elapsed: %f, front: %f, left: %f, back: %f, right: %f", dist(node->getCurrentLocalPose().pose.position, target), (node->now() - start_time).seconds(), front, left, back, right);

        if(std::isnan(cmd_vel.twist.linear.x) || std::isnan(cmd_vel.twist.linear.y))
        {
            RCLCPP_ERROR(node->get_logger(), "GOT NAN: bfr rotate: (%f , %f). afr rotate: (%f, %f)", velx_before_rotate, vely_before_rotate, vel_pose.pose.position.x, vel_pose.pose.position.y);
            continue;
        }

        if(payloadDetected(node) && allow_centering_payload)
        {
            RCLCPP_INFO(node->get_logger(), "Payload Spotted");
            cmd_vel = zero(cmd_vel);
            // cmd_vel = node->getCurrentVelocity();
            // cmd_vel.twist.linear.x *= -0.75;
            // cmd_vel.twist.linear.y *= -0.75;
            node->publishLocalVelocity(cmd_vel);
            
            rclcpp::spin_some(node); rate.sleep();
            
            break;
        }
        if(emberDetected(node) && allow_centering_ember)
        {
            RCLCPP_INFO(node->get_logger(), "Ember Spotted");
            cmd_vel = zero(cmd_vel);
            // cmd_vel = node->getCurrentVelocity();
            // cmd_vel.twist.linear.x *= -0.75;
            // cmd_vel.twist.linear.y *= -0.75;
            node->publishLocalVelocity(cmd_vel);
            
            rclcpp::spin_some(node); rate.sleep();

            break;
        }
        if(isNear(node->getCurrentLocalPose().pose.position, target, tolerance) || (node->now() - start_time).seconds() >= max_time || (x && front <= 1.25 && (abs(front - last_front) <= 0.25 || last_front == 4.0)) || (y && left <= 1.25 && (abs(left - last_left) <= 0.25 || last_left == 4.0)))
        {
            RCLCPP_INFO(node->get_logger(), "Reached target positionnn, dist2target: %f, time elapsed: %f, front: %f, left: %f", dist(node->getCurrentLocalPose().pose.position, target), (node->now() - start_time).seconds(), front, left);
            // cmd_vel = node->getCurrentVelocity();
            // cmd_vel.twist.linear.x *= -0.25;
            // cmd_vel.twist.linear.y *= -0.25;
            cmd_vel = zero(cmd_vel);
            node->publishLocalVelocity(cmd_vel);

            rclcpp::spin_some(node); rate.sleep();

            break;
        }

        node->publishLocalVelocity(cmd_vel);
        
        total_error_maju += x ? err_x : err_y;
        last_error_maju = x ? err_x : err_y;

        total_error_koreksi += x ? err_y : err_x;
        last_error_koreksi = x ? err_y : err_x;

        if(x && (abs(front - last_front) <= 0.25 || last_front == 4.0))
        {
            last_front = front;
        }
        if(y && (abs(left - last_left) <= 0.25 || last_front == 4.0))
        {
            last_front = front;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    posee = node->getCurrentLocalPose();
}

void holdAtLidarRanges(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float front, float left, float back, float right, bool allow_centering_payload, bool allow_centering_ember)
{
    RCLCPP_INFO(node->get_logger(), "Moving the drone at lidar ranges of: front: %f, left: %f, back: %f, right: %f", front, left, back, right);
    float err_x, err_y;
    geometry_msgs::msg::Point vel_pose;
    geometry_msgs::msg::TwistStamped cmd_vel;
    while(rclcpp::ok())
    {

        if(front != -1.0)
        {
            err_x = node->getCurrentLaserScan().ranges[0] != std::numeric_limits<float>::infinity() ?  node->getCurrentLaserScan().ranges[0] - front : 0.0;
            vel_pose.x = limit(0.3, 1.5*err_x);
        }
        else if(back != -1.0)
        {
            err_x = node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/2] != std::numeric_limits<float>::infinity() ?  back - node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/2] : 0.0;
            vel_pose.x = limit(0.3, 1.5*err_x);
        }
        else
        {
            vel_pose.x = 0.0;
        }

        if(left != -1.0)
        {
            err_y = node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/4] != std::numeric_limits<float>::infinity() ?  node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/4] - left : 0.0;
            vel_pose.y = limit(0.3, 1.5*err_y);
        }
        else if(right != -1.0)
        {
            err_y = node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()*3/4] != std::numeric_limits<float>::infinity() ?  right - node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()*3/4] : 0.0;
            vel_pose.y = limit(0.3, 1.5*err_y);
        }
        else
        {
            vel_pose.y = 0.0;
        }

        float drone_heading = getHeading(posee.pose.orientation);
        RCLCPP_INFO(node->get_logger(), "err_x: %f, err_y: %f, cmd_x: %f, cmd_y: %f, heading: %f", err_x, err_y, vel_pose.x, vel_pose.y, drone_heading);
        vel_pose = rotatePoint(vel_pose, drone_heading);
        cmd_vel.twist.linear.x = vel_pose.x;
        cmd_vel.twist.linear.y = vel_pose.y;
        cmd_vel.twist.linear.z = 0.0;
        
        node->publishLocalVelocity(cmd_vel);

        if(payloadDetected(node) && allow_centering_payload)
        {
            RCLCPP_INFO(node->get_logger(), "Payload Spotted");
            break;
        }
        if(emberDetected(node) && allow_centering_ember)
        {
            RCLCPP_INFO(node->get_logger(), "Ember Spotted");
            break;
        }

        if(abs(err_x) < 0.01 && abs(err_y) < 0.01)
        {
            RCLCPP_INFO(node->get_logger(), "Reached target positionnn");
            break;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }
    posee = node->getCurrentLocalPose();
}

float getYawError(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate)
{
    float width = 60.0*DEG2RAD;
    sensor_msgs::msg::LaserScan curr_scan = node->getCurrentLaserScan();
    int left_index = curr_scan.ranges.size()/(2*M_PI) * width/2;
    int right_index = curr_scan.ranges.size() - left_index;

    while(curr_scan.ranges[left_index] < 0.4 || curr_scan.ranges[left_index] > 2.0 || std::isnan(curr_scan.ranges[left_index]) || curr_scan.ranges[left_index] == std::numeric_limits<float>::infinity())
    {
        left_index--;
        if(left_index < 0) {
            RCLCPP_ERROR(node->get_logger(), "depan ga kliatan");
            return 0.0;
        }
    }
    while(curr_scan.ranges[right_index] < 0.4 || curr_scan.ranges[right_index] > 2.0 || std::isnan(curr_scan.ranges[right_index]) || curr_scan.ranges[right_index] == std::numeric_limits<float>::infinity())
    {
        right_index++;
        if(right_index > curr_scan.ranges.size()) {
            RCLCPP_ERROR(node->get_logger(), "depan ga kliatan");
            return 0.0;
        }
    }

    float x1 = curr_scan.ranges[left_index] * cos((curr_scan.ranges.size() / (2*M_PI)) * left_index);
    float y1 = curr_scan.ranges[left_index] * sin((curr_scan.ranges.size() / (2*M_PI)) * left_index);
    float x2 = curr_scan.ranges[right_index] * cos((curr_scan.ranges.size() / (2*M_PI)) * right_index);
    float y2 = curr_scan.ranges[right_index] * sin((curr_scan.ranges.size() / (2*M_PI)) * right_index);

    return atan2(x2 - x1, y1 - y2);

}

void printTitik(std::vector<double>& lat_indoor, std::vector<double>& lon_indoor, std::vector<double>& lat_outdoor, std::vector<double>& lon_outdoor, double lat_takeoff, double lon_takeoff) {
    std::cout << "    lat_takeoff: " << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lat_takeoff << std::endl;
    std::cout << "    lon_takeoff: " << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lon_takeoff << std::endl << std::endl;
    if(lat_indoor.size()) {
        std::cout << "    lat_indoor: [";
        for(int i = 0; i < lat_indoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lat_indoor[i];
            if(i != lat_indoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout << "    lon_indoor: [";
        for(int i = 0; i < lon_indoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lon_indoor[i];
            if(i != lon_indoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout<<std::endl;
    }
    if(lat_outdoor.size())
    {
        std::cout << "    lat_outdoor: [";
        for(int i = 0; i < lat_outdoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lat_outdoor[i];
            if(i != lat_outdoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout << "    lon_outdoor: [";
        for(int i = 0; i < lon_outdoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lon_outdoor[i];
            if(i != lon_outdoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout<<std::endl;
    }
}

void LocalMove(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, float up_z, float yaw_angle, float tolerance) {
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    float current_yaw = getHeading(current_pose.pose.orientation);
    RCLCPP_INFO(node->get_logger(), "Current heading: %.2f degrees", current_yaw * 180.0 / M_PI);
    float global_x = forward_x * cos(current_yaw) - left_y * sin(current_yaw);
    float global_y = forward_x * sin(current_yaw) + left_y * cos(current_yaw);
    target_pose.pose.position.x = current_pose.pose.position.x + global_x;
    target_pose.pose.position.y = current_pose.pose.position.y + global_y;
    target_pose.pose.position.z = current_pose.pose.position.z + up_z;
    const double heading_eps = 1e-4;
    const bool has_translation = std::abs(global_x) > heading_eps || std::abs(global_y) > heading_eps;
    if (yaw_angle != 0.0) {
        setHeading(target_pose.pose.orientation, yaw_angle);
    } else if (has_translation) {
        const double target_yaw = std::atan2(global_y, global_x);
        setHeading(target_pose.pose.orientation, target_yaw);
    } else {
        target_pose.pose.orientation = current_pose.pose.orientation;
    }
    target_pose.header.stamp = node->now();
    target_pose.header.frame_id = "map";
    RCLCPP_INFO(node->get_logger(), "Moving from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Body frame: forward=%.2f, left=%.2f -> Global frame: x=%.2f, y=%.2f", forward_x, left_y, global_x, global_y);
    rclcpp::Rate fast_rate(10.0);
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        float current_distance = sqrt(pow(target_pose.pose.position.x - current_pose.pose.position.x, 2) + pow(target_pose.pose.position.y - current_pose.pose.position.y, 2) + pow(target_pose.pose.position.z - current_pose.pose.position.z, 2));
        if (current_distance < tolerance) {
            RCLCPP_INFO(node->get_logger(), "Waypoint reached! Distance: %.3f m", current_distance);
            break;
        }
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Distance to target: %.3f m", current_distance);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    posee = node->getCurrentLocalPose();
}

// void LocalMove(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, float up_z, float yaw_angle, float tolerance, float step) {
//     geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//     geometry_msgs::msg::PoseStamped target_pose = current_pose;
//     float current_yaw = getHeading(current_pose.pose.orientation);
//     RCLCPP_INFO(node->get_logger(), "Current heading: %.2f degrees", current_yaw * 180.0 / M_PI);
//     float global_x = forward_x * cos(current_yaw) - left_y * sin(current_yaw);
//     float global_y = forward_x * sin(current_yaw) + left_y * cos(current_yaw);
//     target_pose.pose.position.x = current_pose.pose.position.x + global_x;
//     target_pose.pose.position.y = current_pose.pose.position.y + global_y;
//     target_pose.pose.position.z = current_pose.pose.position.z + up_z;
//     const double heading_eps = 1e-4;
//     const bool has_translation = std::abs(global_x) > heading_eps || std::abs(global_y) > heading_eps;
//     if (has_translation) {
//         const double target_yaw = std::atan2(global_y, global_x);
//         setHeading(target_pose.pose.orientation, target_yaw);
//     } else if (yaw_angle != 0.0) {
//         setHeading(target_pose.pose.orientation, yaw_angle);
//     } else {
//         target_pose.pose.orientation = current_pose.pose.orientation;
//     }
//     target_pose.header.stamp = node->now();
//     target_pose.header.frame_id = "map";
    
//     RCLCPP_INFO(node->get_logger(), "Moving from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
//     RCLCPP_INFO(node->get_logger(), "Body frame: forward=%.2f, left=%.2f -> Global frame: x=%.2f, y=%.2f", forward_x, left_y, global_x, global_y);
//     rclcpp::Rate fast_rate(10.0); // 10Hz for ArduPilot
//     rclcpp::Time last_time = node->now();
//     while (rclcpp::ok()) {
//         current_pose = node->getCurrentLocalPose();
//         float current_distance = sqrt(pow(target_pose.pose.position.x - current_pose.pose.position.x, 2) + pow(target_pose.pose.position.y - current_pose.pose.position.y, 2) + pow(target_pose.pose.position.z - current_pose.pose.position.z, 2));
//         if (current_distance < tolerance) {
//             RCLCPP_INFO(node->get_logger(), "Waypoint reached! Distance: %.3f m", current_distance);
//             break;
//         }
//         geometry_msgs::msg::PoseStamped cmd_pose = target_pose;
//         if (step > 0.0f) {
//             const auto now = node->now();
//             const double dt = std::max(1e-3, (now - last_time).seconds());
//             last_time = now;
//             const double step_distance = step * dt;
//             if (current_distance > step_distance) {
//                 const double dx = target_pose.pose.position.x - current_pose.pose.position.x;
//                 const double dy = target_pose.pose.position.y - current_pose.pose.position.y;
//                 const double dz = target_pose.pose.position.z - current_pose.pose.position.z;
//                 const double inv_dist = 1.0 / std::max(current_distance, 1e-6f);
//                 cmd_pose.pose.position.x = current_pose.pose.position.x + dx * inv_dist * step_distance;
//                 cmd_pose.pose.position.y = current_pose.pose.position.y + dy * inv_dist * step_distance;
//                 if (std::abs(up_z) < 1e-3) {
//                     cmd_pose.pose.position.z = target_pose.pose.position.z;
//                 } else {
//                     cmd_pose.pose.position.z = current_pose.pose.position.z + dz * inv_dist * step_distance;
//                 }
//             }
//         }
//         cmd_pose.header.stamp = node->now();
//         node->publishLocalPosition(cmd_pose);
//         RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Distance to target: %.3f m", current_distance);        
//         rclcpp::spin_some(node);
//         fast_rate.sleep();
//     }
//     posee = node->getCurrentLocalPose();
// }

void Rotate(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float rotation_angle) {
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    float current_yaw = getHeading(current_pose.pose.orientation);
    float target_yaw = current_yaw + rotation_angle;
    while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
    RCLCPP_INFO(node->get_logger(), "Rotating from %.2f degrees to %.2f degrees", current_yaw * 180.0 / M_PI, target_yaw * 180.0 / M_PI);
    target_pose.pose.position = current_pose.pose.position;
    setHeading(target_pose.pose.orientation, target_yaw);
    target_pose.header.stamp = node->now();
    target_pose.header.frame_id = "map";
    float rotation_tolerance = 5.0 * M_PI / 180.0;
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        current_yaw = getHeading(current_pose.pose.orientation);
        float angle_diff = target_yaw - current_yaw;
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        if (fabs(angle_diff) < rotation_tolerance) {
            RCLCPP_INFO(node->get_logger(), "Rotation complete! Current heading: %.2f degrees", current_yaw * 180.0 / M_PI);
            break;
        }
        target_pose.pose.position = current_pose.pose.position;
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Angle to target: %.2f degrees", angle_diff * 180.0 / M_PI);
        rclcpp::spin_some(node);
        rate.sleep();
    }
    posee = node->getCurrentLocalPose();
}

void rotateByDegrees(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float degrees) {
    float radians = degrees * M_PI / 180.0;
    RCLCPP_INFO(node->get_logger(), "Rotating %.1f degrees %s...", fabs(degrees), degrees >= 0 ? "RIGHT (clockwise)" : "LEFT (counter-clockwise)");
    Rotate(node, rate, posee, radians);
}

void strafeRight(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                 geometry_msgs::msg::PoseStamped &posee, float distance, float tolerance) {
    /**
     * Strafe (move sideways) to the right relative to drone's current heading
     * Positive distance moves to the right
     */
    RCLCPP_INFO(node->get_logger(), "Strafing RIGHT %.2f meters", distance);
    
    // Call LocalMove with negative left_y (right is negative left)
    // forward_x = 0 (no forward/backward movement)
    // left_y = -distance (negative = right)
    // up_z = 0 (no altitude change)
    // yaw_angle = 0 (keep current heading)
    LocalMove(node, rate, posee, 0.0, -distance, 0.0, 0.0, tolerance);
}

void strafeLeft(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                geometry_msgs::msg::PoseStamped &posee, float distance, float tolerance) {
    /**
     * Strafe (move sideways) to the left relative to drone's current heading
     * Positive distance moves to the left
     */
    RCLCPP_INFO(node->get_logger(), "Strafing LEFT %.2f meters", distance);
    
    // Call LocalMove with positive left_y
    // forward_x = 0 (no forward/backward movement)
    // left_y = +distance (positive = left)
    // up_z = 0 (no altitude change)
    // yaw_angle = 0 (keep current heading)
    LocalMove(node, rate, posee, 0.0, distance, 0.0, 0.0, tolerance);
}

bool detectGatePosts(const std::shared_ptr<DroneController>&node, float &left_post_angle, 
                     float &right_post_angle, float &left_post_distance, float &right_post_distance,
                     float gate_width, float max_detection_range) {
    /**
     * Detect two gate posts using 2D lidar scan
     * Algorithm:
     * 1. Get lidar scan data
     * 2. Find clusters of points (obstacles)
     * 3. Identify two clusters that are approximately gate_width apart
     * 4. Calculate angle and distance to each post
     */
    
    sensor_msgs::msg::LaserScan scan = node->getCurrentLaserScan();
    
    if (scan.ranges.empty()) {
        RCLCPP_WARN(node->get_logger(), "Lidar scan is empty!");
        return false;
    }
    
    // Find all obstacles (points closer than max_detection_range)
    struct ObstaclePoint {
        float angle;
        float distance;
        int index;
    };
    
    std::vector<ObstaclePoint> obstacles;
    
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        float range = scan.ranges[i];
        
        // Filter out invalid readings and too far points
        if (std::isfinite(range) && range > scan.range_min && 
            range < scan.range_max && range < max_detection_range) {
            
            float angle = scan.angle_min + i * scan.angle_increment;
            obstacles.push_back({angle, range, (int)i});
        }
    }
    
    if (obstacles.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Not enough obstacles detected: %zu", obstacles.size());
        return false;
    }
    
    // Cluster obstacles to find distinct posts
    // Simple clustering: find gaps in consecutive points
    std::vector<std::vector<ObstaclePoint>> clusters;
    std::vector<ObstaclePoint> current_cluster;
    
    float cluster_gap_threshold = 0.3; // 0.3 radians (~17 degrees) gap = new cluster (more sensitive)
    
    current_cluster.push_back(obstacles[0]);
    
    for (size_t i = 1; i < obstacles.size(); i++) {
        float angle_diff = obstacles[i].angle - obstacles[i-1].angle;
        
        if (angle_diff > cluster_gap_threshold) {
            // Start new cluster
            if (current_cluster.size() >= 2) { // Minimum 2 points to be a valid post (more sensitive)
                clusters.push_back(current_cluster);
            }
            current_cluster.clear();
        }
        current_cluster.push_back(obstacles[i]);
    }
    
    // Don't forget the last cluster
    if (current_cluster.size() >= 2) {
        clusters.push_back(current_cluster);
    }
    
    if (clusters.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Only %zu cluster(s) detected, need at least 2 for gate posts", 
                   clusters.size());
        return false;
    }
    
    // Find the centroid of each cluster (average position)
    struct Post {
        float angle;
        float distance;
        float x; // Cartesian x
        float y; // Cartesian y
    };
    
    std::vector<Post> posts;
    
    for (const auto& cluster : clusters) {
        float avg_x = 0.0;
        float avg_y = 0.0;
        
        for (const auto& point : cluster) {
            avg_x += point.distance * cos(point.angle);
            avg_y += point.distance * sin(point.angle);
        }
        
        avg_x /= cluster.size();
        avg_y /= cluster.size();
        
        float post_distance = sqrt(avg_x * avg_x + avg_y * avg_y);
        float post_angle = atan2(avg_y, avg_x);
        
        posts.push_back({post_angle, post_distance, avg_x, avg_y});
    }
    
    // Find two posts that are approximately gate_width apart
    float best_match_error = std::numeric_limits<float>::max();
    int best_left_idx = -1;
    int best_right_idx = -1;
    
    for (size_t i = 0; i < posts.size(); i++) {
        for (size_t j = i + 1; j < posts.size(); j++) {
            // Calculate distance between posts
            float dx = posts[j].x - posts[i].x;
            float dy = posts[j].y - posts[i].y;
            float distance_between = sqrt(dx * dx + dy * dy);
            
            // Check if distance matches gate_width (within 30% tolerance)
            float error = fabs(distance_between - gate_width);
            float tolerance_ratio = 0.3; // 30% tolerance
            
            if (error < gate_width * tolerance_ratio && error < best_match_error) {
                best_match_error = error;
                
                // Determine left and right based on angle
                if (posts[i].angle > posts[j].angle) {
                    best_left_idx = i;
                    best_right_idx = j;
                } else {
                    best_left_idx = j;
                    best_right_idx = i;
                }
            }
        }
    }
    
    if (best_left_idx == -1 || best_right_idx == -1) {
        RCLCPP_WARN(node->get_logger(), 
                   "Could not find two posts matching gate width %.2f m (checked %zu posts)", 
                   gate_width, posts.size());
        return false;
    }
    
    // Output the results
    left_post_angle = posts[best_left_idx].angle;
    left_post_distance = posts[best_left_idx].distance;
    right_post_angle = posts[best_right_idx].angle;
    right_post_distance = posts[best_right_idx].distance;
    
    float detected_width = sqrt(
        pow(posts[best_left_idx].x - posts[best_right_idx].x, 2) +
        pow(posts[best_left_idx].y - posts[best_right_idx].y, 2)
    );
    
    RCLCPP_INFO(node->get_logger(), 
               "Gate detected! Left: angle=%.2f° dist=%.2fm, Right: angle=%.2f° dist=%.2fm, Width=%.2fm",
               left_post_angle * 180.0 / M_PI, left_post_distance,
               right_post_angle * 180.0 / M_PI, right_post_distance,
               detected_width);
    
    return true;
}

bool detectGatePostsNear(const std::shared_ptr<DroneController>&node, float &left_post_angle, 
                         float &right_post_angle, float &left_post_distance, float &right_post_distance,
                         float gate_width, float max_detection_range, 
                         float forward_zone_min, float max_lateral_angle) {
    /**
     * Detect two gate posts using 2D lidar scan - PRIORITIZE NEAREST FORWARD GATE
     * 
     * NEW STRATEGY for handling multiple gates (side-by-side scenario):
     * 1. Filter posts by forward zone (x > forward_zone_min)
     * 2. Find all gate candidates matching gate_width
     * 3. SCORE each gate based on forward alignment (angle to center)
     * 4. Select gate with BEST alignment (closest to 0° heading)
     * 
     * This ensures drone always targets the gate directly in front,
     * not the one on the side when multiple gates are present.
     */
    
    sensor_msgs::msg::LaserScan scan = node->getCurrentLaserScan();
    
    if (scan.ranges.empty()) {
        RCLCPP_WARN(node->get_logger(), "Lidar scan Kosongg!");
        return false;
    }
    
    // Find all obstacles (points closer than max_detection_range)
    struct ObstaclePoint {
        float angle;
        float distance;
        int index;
    };
    
    std::vector<ObstaclePoint> obstacles;
    
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        float range = scan.ranges[i];
        
        // Filter out invalid readings and too far points
        if (std::isfinite(range) && range > scan.range_min && 
            range < scan.range_max && range < max_detection_range) {
            
            float angle = scan.angle_min + i * scan.angle_increment;
            
            // STRATEGY 1: Filter by forward zone
            float x = range * cos(angle);
            if (x > forward_zone_min) {  // Only consider obstacles in forward zone
                obstacles.push_back({angle, range, (int)i});
            }
        }
    }
    
    if (obstacles.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Not enough obstacles in forward zone: %zu", obstacles.size());
        return false;
    }
    
    // Cluster obstacles to find distinct posts
    std::vector<std::vector<ObstaclePoint>> clusters;
    std::vector<ObstaclePoint> current_cluster;
    
    // PHASE 1 FIX: Reduced threshold for better detection when close to gate
    float cluster_gap_threshold = 0.2; // 0.2 radians (~11.5 degrees) gap = new cluster (was 0.3)
    
    current_cluster.push_back(obstacles[0]);
    
    for (size_t i = 1; i < obstacles.size(); i++) {
        float angle_diff = obstacles[i].angle - obstacles[i-1].angle;
        
        if (angle_diff > cluster_gap_threshold) {
            // Start new cluster
            if (current_cluster.size() >= 2) {
                clusters.push_back(current_cluster);
            }
            current_cluster.clear();
        }
        current_cluster.push_back(obstacles[i]);
    }
    
    // Don't forget the last cluster
    if (current_cluster.size() >= 2) {
        clusters.push_back(current_cluster);
    }
    
    if (clusters.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Only %zu cluster(s) detected in forward zone, need at least 2 for gate posts", 
                   clusters.size());
        return false;
    }
    
    // Calculate centroid of each cluster
    struct Post {
        float angle;
        float distance;
        float x; // Cartesian x (forward)
        float y; // Cartesian y (lateral)
    };
    
    std::vector<Post> posts;
    
    for (const auto& cluster : clusters) {
        float avg_x = 0.0;
        float avg_y = 0.0;
        
        for (const auto& point : cluster) {
            avg_x += point.distance * cos(point.angle);
            avg_y += point.distance * sin(point.angle);
        }
        
        avg_x /= cluster.size();
        avg_y /= cluster.size();
        
        float post_distance = sqrt(avg_x * avg_x + avg_y * avg_y);
        float post_angle = atan2(avg_y, avg_x);
        
        posts.push_back({post_angle, post_distance, avg_x, avg_y});
    }
    
    // STRATEGY 2: Find gate candidates and score them by forward alignment
    struct GateCandidate {
        int left_idx;
        int right_idx;
        float center_angle;      // Angle to gate center (0° = straight ahead)
        float forward_distance;  // Average forward distance (x coordinate)
        float width_error;       // Error from expected gate_width
        float alignment_score;   // Lower is better (closer to 0° heading)
    };
    
    std::vector<GateCandidate> gate_candidates;
    
    for (size_t i = 0; i < posts.size(); i++) {
        for (size_t j = i + 1; j < posts.size(); j++) {
            // Calculate distance between posts
            float dx = posts[j].x - posts[i].x;
            float dy = posts[j].y - posts[i].y;
            float distance_between = sqrt(dx * dx + dy * dy);
            
            // PHASE 1 FIX: Increased tolerance for better detection when close to gate
            // Check if distance matches gate_width (within 50% tolerance, was 30%)
            float width_error = fabs(distance_between - gate_width);
            float tolerance_ratio = 0.5; // 50% tolerance (was 0.3)
            
            if (width_error < gate_width * tolerance_ratio) {
                // Valid gate candidate found
                
                // Calculate gate center
                float center_x = (posts[i].x + posts[j].x) / 2.0;
                float center_y = (posts[i].y + posts[j].y) / 2.0;
                float center_angle = atan2(center_y, center_x);
                
                // Calculate forward distance (average x)
                float forward_dist = (posts[i].x + posts[j].x) / 2.0;
                
                // STRATEGY 3: Score based on forward alignment
                // Lower score = better (gate more aligned with heading)
                float alignment_score = fabs(center_angle);
                
                // Skip gates that are too far to the side
                if (alignment_score > max_lateral_angle) {
                    continue;  // Gate is too much to the side, skip it
                }
                
                // Determine left and right based on angle
                int left_idx, right_idx;
                if (posts[i].angle > posts[j].angle) {
                    left_idx = i;
                    right_idx = j;
                } else {
                    left_idx = j;
                    right_idx = i;
                }
                
                gate_candidates.push_back({
                    left_idx, 
                    right_idx, 
                    center_angle, 
                    forward_dist,
                    width_error, 
                    alignment_score
                });
            }
        }
    }
    
    if (gate_candidates.empty()) {
        RCLCPP_WARN(node->get_logger(), 
                   "Gaada point cloud yang bisa di cluster %.2f m depan kosong, gate gakedetek (checked %zu posts)", 
                   gate_width, posts.size());
        return false;
    }
    
    // STRATEGY 4: Select gate with BEST forward alignment (lowest alignment_score)
    auto best_gate = std::min_element(gate_candidates.begin(), gate_candidates.end(),
        [](const GateCandidate& a, const GateCandidate& b) {
            return a.alignment_score < b.alignment_score;
        });
    
    // Output the results
    left_post_angle = posts[best_gate->left_idx].angle;
    left_post_distance = posts[best_gate->left_idx].distance;
    right_post_angle = posts[best_gate->right_idx].angle;
    right_post_distance = posts[best_gate->right_idx].distance;
    
    float detected_width = sqrt(
        pow(posts[best_gate->left_idx].x - posts[best_gate->right_idx].x, 2) +
        pow(posts[best_gate->left_idx].y - posts[best_gate->right_idx].y, 2)
    );
    
    RCLCPP_INFO(node->get_logger(), 
               "NEAREST FORWARD GATE detected! Left: angle=%.2f° dist=%.2fm, Right: angle=%.2f° dist=%.2fm, Width=%.2fm, Center angle=%.2f° (selected from %zu candidates)",
               left_post_angle * 180.0 / M_PI, left_post_distance,
               right_post_angle * 180.0 / M_PI, right_post_distance,
               detected_width, best_gate->center_angle * 180.0 / M_PI, gate_candidates.size());
    
    return true;
}


void centering_gate(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    geometry_msgs::msg::PoseStamped &posee,
    float gate_width,
    float tolerance,
    bool &status,
    float max_velocity,
    float proportional_gain,
    float max_time,
    bool yaw_alignment_enabled,
    bool test_mode)
{
    if (test_mode) {
        RCLCPP_INFO(node->get_logger(), "===== CENTERING TEST MODE =====");
    } else {
        RCLCPP_INFO(node->get_logger(), "===== STARTING GATE CENTERING =====");
    }
    RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
                gate_width, tolerance);
    if (!test_mode) {
        RCLCPP_INFO(node->get_logger(), "WARNING: THIS IS NOT A TEST!");
    }
    
    float detection_range_min = 1.5f;
    float detection_range_max = 2.5f;
    float roi_lateral = 2.0f;
    int min_cluster_points = 2;
    node->get_parameter("gate_centering.detection_range_min", detection_range_min);
    node->get_parameter("gate_centering.detection_range_max", detection_range_max);
    node->get_parameter("gate_centering.roi_lateral", roi_lateral);
    node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    const float x_min = detection_range_min;  
    const float x_max = detection_range_max;  
    const float y_min = -roi_lateral;         
    const float y_max = roi_lateral;          
    const float z_min = 0.0f;  
    const float z_max = 3.0f;  
    const int num_bins = 60;  
    const float bin_size = (y_max - y_min) / num_bins;
    const double yaw_tolerance = 4.0 * M_PI / 180.0;
    const double yaw_gain = 0.5;
    const double max_yaw_rate = 0.6;
    const int min_points_per_pole = min_cluster_points;  
    const float gate_width_min = 0.8f;  
    const float gate_width_max = 2.5f;  
    RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]", x_min, x_max, y_min, y_max, z_min, z_max);
    RCLCPP_INFO(node->get_logger(), "Grid: %d bins %.2fm, Min points/pole: %d", num_bins, bin_size, min_points_per_pole);
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud = nullptr;
    
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        [&latest_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            latest_cloud = msg;
        });

    auto start_time = node->now();
    int consecutive_centered_count = 0;
    const int required_centered_frames = 1; 
    status = false;
    bool yaw_aligned = !yaw_alignment_enabled;
    
    geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
    float initial_z = initial_pose.pose.position.z;

    geometry_msgs::msg::PoseStamped hold_pose = initial_pose;
    auto publish_hold_pose = [&node](const geometry_msgs::msg::PoseStamped &pose) {
        geometry_msgs::msg::PoseStamped hold_cmd = pose;
        hold_cmd.header.stamp = node->now();
        hold_cmd.header.frame_id = "map";
        node->publishLocalPosition(hold_cmd);
    };
    auto refresh_hold_pose = [&hold_pose](const geometry_msgs::msg::PoseStamped &pose) {
        hold_pose = pose;
        hold_pose.header.frame_id = "map";
    };
    
    while (rclcpp::ok()) {
        auto elapsed = (node->now() - start_time).seconds();
        if (elapsed > max_time) {
            RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
            break;
        }
        
        if (!latest_cloud || latest_cloud->width == 0) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "Waiting for Livox PointCloud2 data on /livox/lidar...");
            if (!test_mode) {
                publish_hold_pose(hold_pose);
            }
            
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
    int bins[num_bins] = {0};  
    double bin_x_sum[num_bins] = {0.0};
        int total_roi_points = 0;
        
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud, "z");
        size_t num_points = latest_cloud->width * latest_cloud->height;
        
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

            if (x >= x_min && x <= x_max &&
                y >= y_min && y <= y_max &&
                z >= z_min && z <= z_max) {
                
                int bin_idx = static_cast<int>((y - y_min) / bin_size);

                if (bin_idx >= 0 && bin_idx < num_bins) {
                    bins[bin_idx]++;
                    bin_x_sum[bin_idx] += x;
                    total_roi_points++;
                }
            }
        }
        
        if (total_roi_points < min_points_per_pole * 2) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Insufficient points in ROI: %d (need %d)", total_roi_points, min_points_per_pole * 2);
            if (!test_mode) {
                publish_hold_pose(hold_pose);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        int first_peak_idx = -1;
        int first_peak_count = 0;
        
        for (int i = 0; i < num_bins; i++) {
            if (bins[i] > first_peak_count) {
                first_peak_count = bins[i];
                first_peak_idx = i;
            }
        }
        
        if (first_peak_count < min_points_per_pole) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "First peak too weak: %d points (need %d)", first_peak_count, min_points_per_pole);
            if (!test_mode) {
                publish_hold_pose(hold_pose);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
    
        int second_peak_idx = -1;
        int second_peak_count = 0;
        const int min_peak_separation = 10;  
        
        for (int i = 0; i < num_bins; i++) {
            if (std::abs(i - first_peak_idx) < min_peak_separation) continue;
            if (bins[i] > second_peak_count) {
                second_peak_count = bins[i];
                second_peak_idx = i;
            }
        }
        
        if (second_peak_count < min_points_per_pole) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Second peak too weak: %d points (need %d)", second_peak_count, min_points_per_pole);
            if (!test_mode) {
                publish_hold_pose(hold_pose);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        int left_idx = (first_peak_idx < second_peak_idx) ? first_peak_idx : second_peak_idx;
        int right_idx = (first_peak_idx < second_peak_idx) ? second_peak_idx : first_peak_idx;
    float left_y = y_min + (left_idx + 0.5f) * bin_size;  
    float right_y = y_min + (right_idx + 0.5f) * bin_size;
    float detected_width = right_y - left_y;
    double left_x_mean = bins[left_idx] > 0 ? bin_x_sum[left_idx] / bins[left_idx] : 0.0;
    double right_x_mean = bins[right_idx] > 0 ? bin_x_sum[right_idx] / bins[right_idx] : 0.0;
    double yaw_error = yaw_alignment_enabled
        ? std::atan2(left_x_mean - right_x_mean, std::max(static_cast<double>(detected_width), 1e-3))
        : 0.0;

        if (detected_width < gate_width_min || detected_width > gate_width_max) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Invalid gate width: %.2fm (expected %.2f-%.2fm)", detected_width, gate_width_min, gate_width_max);
            if (!test_mode) {
                publish_hold_pose(hold_pose);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }

        float center_y = (left_y + right_y) / 2.0f;
        float lateral_error = center_y;  
        if (!yaw_aligned && std::abs(yaw_error) < yaw_tolerance) {
            yaw_aligned = true;
        }
    bool is_centered = (std::abs(lateral_error) < tolerance) && (!yaw_alignment_enabled || std::abs(yaw_error) < yaw_tolerance);
        if (is_centered) {
            yaw_aligned = true;
            consecutive_centered_count++;
            if (test_mode) {
                RCLCPP_INFO(node->get_logger(), "CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm | YawErr:%.2fdeg", consecutive_centered_count, required_centered_frames, left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI);
            } else {
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500, "CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm | YawErr: %.2fdeg | Pts[L:%d R:%d]", consecutive_centered_count, required_centered_frames, left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI, bins[left_idx], bins[right_idx]);
            }

            if (!test_mode) {
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                refresh_hold_pose(current_pose);
                publish_hold_pose(hold_pose);
            }
            
            if (consecutive_centered_count >= required_centered_frames) {
                if (test_mode) {
                    RCLCPP_INFO(node->get_logger(), "TEST MODE SUCCESSFUL!");
                } else {
                    RCLCPP_INFO(node->get_logger(), "GATE CENTERING SUCCESSFUL!");
                }
                status = true;
                break;
            }
        } else {
            consecutive_centered_count = 0;     
            float target_velocity_body_y = proportional_gain * lateral_error; 
            target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            double target_yaw_rate = 0.0;
            if (!yaw_aligned) {
                target_yaw_rate = std::max(-max_yaw_rate, std::min(max_yaw_rate, yaw_gain * yaw_error));
            }
            if (test_mode) {
                std::string direction = (lateral_error > 0) ? "RIGHT" : "LEFT";
                RCLCPP_INFO(node->get_logger(), "NOT CENTERED | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm %s | Vel:%.3fm/s | YawErr:%.2fdeg", left_y, right_y, detected_width, lateral_error, direction.c_str(), target_velocity_body_y, yaw_error * 180.0 / M_PI);
            }
        
            if (!test_mode) {
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                refresh_hold_pose(current_pose);
                float current_yaw = getHeading(current_pose.pose.orientation);
                float vx_body = 0.0;  
                float vy_body = target_velocity_body_y;
                float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
                float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                float altitude_error = initial_z - current_pose.pose.position.z;
                float vz_global = 0.0;
                if (std::abs(altitude_error) > 0.1f) {
                    vz_global = 0.3f * altitude_error; 
                    vz_global = std::max(-0.2f, std::min(0.2f, vz_global));
                }
                
                geometry_msgs::msg::TwistStamped twist_cmd;
                twist_cmd.header.stamp = node->now();
                twist_cmd.header.frame_id = "map";  
                twist_cmd.twist.linear.x = vx_global;
                twist_cmd.twist.linear.y = vy_global;
                twist_cmd.twist.linear.z = vz_global;
                twist_cmd.twist.angular.z = target_yaw_rate;
                
                node->publishLocalVelocity(twist_cmd);
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500, "Centering: Gate[L:%.2f R:%.2f W:%.2fm] | Err:%.3fm | Vel_body_y:%.3f | YawErr:%.2fdeg | Vel_map[x:%.3f y:%.3f] | Yaw:%.1f° | Pts[L:%d R:%d]", left_y, right_y, detected_width, lateral_error, target_velocity_body_y, yaw_error * 180.0 / M_PI, vx_global, vy_global, current_yaw * 180.0 / M_PI, bins[left_idx], bins[right_idx]);
            }
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    if (!test_mode) {
        geometry_msgs::msg::PoseStamped final_pose = node->getCurrentLocalPose();
        refresh_hold_pose(final_pose);
        publish_hold_pose(hold_pose);
    }
    
    if (status) {
        if (test_mode) {
            RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE COMPLETED ===");
        } else {
            RCLCPP_INFO(node->get_logger(), "=== GATE CENTERING COMPLETED ===");
        }
    } else {
        if (test_mode) {
            RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE FAILED ===");
        } else {
            RCLCPP_ERROR(node->get_logger(), "=== GATE CENTERING FAILED ===");
        }
    }
}

void centering_tag(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    float step,
    float offset,
    float close_threshold,
    float max_velocity,
    float timeout_sec,
    bool downward_camera,
    float min_velocity)
{
    if (step <= 0.0f) {
        RCLCPP_ERROR(node->get_logger(), "centering_tag: step must be > 0");
        return;
    }

    std_msgs::msg::Float64MultiArray::SharedPtr latest_tag_msg = nullptr;
    auto tag_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/tag_pose",
        10,
        [&latest_tag_msg](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            latest_tag_msg = msg;
        });

    (void)tag_sub;
    const auto start_time = node->now();
    bool ever_detected = false;
    geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    float target_z = hold_pose.pose.position.z;
    const float tolerance_px = close_threshold;
    auto publish_hold_pose = [&node](const geometry_msgs::msg::PoseStamped &pose) {
        geometry_msgs::msg::PoseStamped hold_cmd = pose;
        hold_cmd.header.stamp = node->now();
        hold_cmd.header.frame_id = "map";
        node->publishLocalPosition(hold_cmd);
    };
    auto refresh_hold_pose = [&hold_pose, &target_z](const geometry_msgs::msg::PoseStamped &pose) {
        hold_pose = pose;
        hold_pose.pose.position.z = target_z;
        hold_pose.header.frame_id = "map";
    };
    geometry_msgs::msg::TwistStamped cmd_vel;
    geometry_msgs::msg::TwistStamped last_cmd_vel;
    bool have_last_cmd = false;
    float filtered_err_x = 0.0f;
    float filtered_err_y = 0.0f;
    bool have_filtered = false;
    const float filter_alpha = 0.2f;
    const float max_accel = std::max(0.02f, max_velocity);

    RCLCPP_INFO(node->get_logger(),
                "centering_tag params: step=%.1f px/(m/s), close_threshold=%.1f px, max_vel=%.2f, min_vel=%.2f, offset=%.2f, downward_camera=%s",
                step, tolerance_px, max_velocity, min_velocity, offset, downward_camera ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "centering_tag: waiting for /tag_pose...");
    while (rclcpp::ok()) {
        if ((node->now() - start_time).seconds() > timeout_sec) {
            RCLCPP_WARN(node->get_logger(), "centering_tag: timeout after %.1f sec", timeout_sec);
            break;
        }

        if (!latest_tag_msg) {
            publish_hold_pose(hold_pose);
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "centering_tag: target not detected, holding position");
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }

        if (latest_tag_msg->data.size() < 3) {
            publish_hold_pose(hold_pose);
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "centering_tag: /tag_pose data must contain [x, y, center_dist], got %zu",
                latest_tag_msg->data.size());
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }

        const float x = static_cast<float>(latest_tag_msg->data[0]);
        const float y = static_cast<float>(latest_tag_msg->data[1]);
        const float center_dist = static_cast<float>(latest_tag_msg->data[2]);
        const bool detected_now =
            std::isfinite(x) && std::isfinite(y) && std::isfinite(center_dist) &&
            (std::abs(x) > 1e-4f || std::abs(y) > 1e-4f || std::abs(center_dist) > 1e-4f);

        if (detected_now) {
            ever_detected = true;
        }

        if (!ever_detected) {
            publish_hold_pose(hold_pose);
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "centering_tag: waiting for first valid target, holding position");
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
        refresh_hold_pose(current_pose);

        const float cam_err_x = x - offset;
        const float cam_err_y = y - offset;
        float body_err_x = cam_err_x;
        float body_err_y = cam_err_y;
        if (downward_camera) {
            body_err_x = -cam_err_y;
            body_err_y = -cam_err_x;
        }

        const float pos_error = std::hypot(body_err_x, body_err_y);

        if (pos_error <= tolerance_px) {
            publish_hold_pose(hold_pose);
            RCLCPP_INFO(node->get_logger(), "CENTEREDD...!! (pos_error=%.2f px)", pos_error);
            break;
        }

        if (!have_filtered) {
            filtered_err_x = body_err_x;
            filtered_err_y = body_err_y;
            have_filtered = true;
        } else {
            filtered_err_x = filter_alpha * body_err_x + (1.0f - filter_alpha) * filtered_err_x;
            filtered_err_y = filter_alpha * body_err_y + (1.0f - filter_alpha) * filtered_err_y;
        }

        const float effective_min_velocity = (pos_error <= tolerance_px * 2.0f) ? 0.0f : min_velocity;
        cmd_vel.twist.linear.x = applyPrecisionStep(filtered_err_x / step, effective_min_velocity, max_velocity);
        cmd_vel.twist.linear.y = applyPrecisionStep(filtered_err_y / step, effective_min_velocity, max_velocity);
        const float z_error = target_z - current_pose.pose.position.z;
        float vz_hold = 0.3f * z_error;
        vz_hold = std::max(-0.3f, std::min(0.3f, vz_hold));
        cmd_vel.twist.linear.z = vz_hold;

        if (have_last_cmd) {
            cmd_vel = limitDelta(max_accel / RATE, cmd_vel, last_cmd_vel);
        }
        last_cmd_vel = cmd_vel;
        have_last_cmd = true;
        node->publishLocalVelocity(cmd_vel);

        RCLCPP_INFO_THROTTLE(
            node->get_logger(), *node->get_clock(), 800,
            "centering_tag: cam(x,y)=(%.1f,%.1f) body_err=(%.1f,%.1f) pos_err=%.1f px vx=%.3f vy=%.3f",
            x, y, body_err_x, body_err_y, pos_error, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y);

        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void centeringGateLivoxSimple(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    geometry_msgs::msg::PoseStamped &posee,
    float gate_width,
    float tolerance,
    bool &status,
    float max_velocity,
    float proportional_gain,
    float max_time,
    bool test_mode)
{
    if (test_mode) {
        RCLCPP_INFO(node->get_logger(), "=== LIVOX CENTERING TEST MODE ===");
    } else {
        RCLCPP_INFO(node->get_logger(), "=== Starting SIMPLE Livox Gate Centering ===");
    }
    RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
                gate_width, tolerance);
    if (!test_mode) {
        RCLCPP_INFO(node->get_logger(), "Strategy: Grid binning + peak detection (O(n) complexity)");
    }
    
    // Read ROI parameters from config (allows dynamic adjustment without recompile!)
    float detection_range_min = 1.5f;
    float detection_range_max = 2.5f;
    float roi_lateral = 2.0f;
    int min_cluster_points = 10;
    
    node->get_parameter("gate_centering.detection_range_min", detection_range_min);
    node->get_parameter("gate_centering.detection_range_max", detection_range_max);
    node->get_parameter("gate_centering.roi_lateral", roi_lateral);
    node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
    // ROI parameters - NOW FROM CONFIG!
    const float x_min = detection_range_min;  // Forward distance min
    const float x_max = detection_range_max;  // Forward distance max
    const float y_min = -roi_lateral;         // Lateral min (left)
    const float y_max = roi_lateral;          // Lateral max (right)
    const float z_min = 0.0f;  // Height min
    const float z_max = 3.0f;  // Height max (increased for long range)
    
    // Grid binning parameters - ADAPTIVE
    const int num_bins = 60;  // More bins for better resolution
    const float bin_size = (y_max - y_min) / num_bins;
    const int min_points_per_pole = min_cluster_points;  // FROM CONFIG!
    
    // Gate validation parameters - MORE TOLERANT for long range
    const float gate_width_min = 0.8f;  // More tolerant
    const float gate_width_max = 2.5f;  // More tolerant
    
    RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
                x_min, x_max, y_min, y_max, z_min, z_max);
    RCLCPP_INFO(node->get_logger(), "Grid: %d bins %.2fm, Min points/pole: %d",
                num_bins, bin_size, min_points_per_pole);
    
    // Subscribe to Livox LiDAR data (PointCloud2)
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud = nullptr;
    
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        [&latest_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            latest_cloud = msg;
        });
    
    RCLCPP_INFO(node->get_logger(), "Subscribing to Livox PointCloud2 on /livox/lidar");

    // Centering control variables
    auto start_time = node->now();
    int consecutive_centered_count = 0;
    const int required_centered_frames = 10; // 1 second at 10Hz
    status = false;
    
    // Get initial altitude for stability
    geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
    float initial_z = initial_pose.pose.position.z;
    
    // ArduPilot: Prepare hold position for streaming when waiting
    geometry_msgs::msg::PoseStamped hold_pose = initial_pose;
    
    while (rclcpp::ok()) {
        // Check timeout
        auto elapsed = (node->now() - start_time).seconds();
        if (elapsed > max_time) {
            RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
            break;
        }
        
        if (!latest_cloud || latest_cloud->width == 0) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                "Waiting for Livox PointCloud2 data on /livox/lidar...");
            
            // ArduPilot: Stream setpoint to maintain position while waiting
            if (!test_mode) {
                hold_pose = node->getCurrentLocalPose();
                hold_pose.header.stamp = node->now();
                node->publishLocalPosition(hold_pose);
            }
            
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // ===== STEP 1: Extract and filter points by ROI using PointCloud2 =====
        
        // PointCloud2 structure:
        // - header: std_msgs/Header
        // - height: uint32 (1 for unordered)
        // - width: uint32 (number of points)
        // - fields: PointField[] (x, y, z, intensity, etc.)
        // - is_bigendian: bool
        // - point_step: uint32 (size of one point in bytes)
        // - row_step: uint32 (width * point_step)
        // - data: uint8[] (actual point data)
        
        // ===== STEP 2: Grid binning - count points per lateral bin =====
        
        int bins[num_bins] = {0};  // Initialize all bins to 0
        int total_roi_points = 0;
        
        // Create iterators for x, y, z fields
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud, "z");
        
        // Get total number of points
        size_t num_points = latest_cloud->width * latest_cloud->height;
        
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
            
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            
            // Skip invalid points
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
            // Filter by ROI
            if (x >= x_min && x <= x_max &&
                y >= y_min && y <= y_max &&
                z >= z_min && z <= z_max) {
                
                // Calculate bin index
                int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
                // Clamp to valid range
                if (bin_idx >= 0 && bin_idx < num_bins) {
                    bins[bin_idx]++;
                    total_roi_points++;
                }
            }
        }
        
        if (total_roi_points < min_points_per_pole * 2) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                "Insufficient points in ROI: %d (need %d)", 
                total_roi_points, min_points_per_pole * 2);
            
            // Stop lateral movement but maintain altitude
            if (!test_mode) {
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                float altitude_error = initial_z - current_pose.pose.position.z;
                float vz_global = 0.0;
                if (std::abs(altitude_error) > 0.05f) {
                    vz_global = 0.4f * altitude_error;
                    vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
                }
                
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = node->now();
                stop_cmd.header.frame_id = "map";
                stop_cmd.twist.linear.z = vz_global;  // Maintain altitude
                node->publishLocalVelocity(stop_cmd);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // ===== STEP 3: Find 2 peak bins (left & right poles) =====
        
        // Find first peak (highest bin count)
        int first_peak_idx = -1;
        int first_peak_count = 0;
        
        for (int i = 0; i < num_bins; i++) {
            if (bins[i] > first_peak_count) {
                first_peak_count = bins[i];
                first_peak_idx = i;
            }
        }
        
        if (first_peak_count < min_points_per_pole) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                "First peak too weak: %d points (need %d)", 
                first_peak_count, min_points_per_pole);
            
            // Stop lateral movement but maintain altitude
            if (!test_mode) {
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                float altitude_error = initial_z - current_pose.pose.position.z;
                float vz_global = 0.0;
                if (std::abs(altitude_error) > 0.05f) {
                    vz_global = 0.4f * altitude_error;
                    vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
                }
                
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = node->now();
                stop_cmd.header.frame_id = "map";
                stop_cmd.twist.linear.z = vz_global;
                node->publishLocalVelocity(stop_cmd);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // Find second peak (must be at least 10 bins away from first peak)
        int second_peak_idx = -1;
        int second_peak_count = 0;
        const int min_peak_separation = 10;  // ~1.0m minimum separation
        
        for (int i = 0; i < num_bins; i++) {
            // Must be far enough from first peak
            if (std::abs(i - first_peak_idx) < min_peak_separation) continue;
            
            if (bins[i] > second_peak_count) {
                second_peak_count = bins[i];
                second_peak_idx = i;
            }
        }
        
        if (second_peak_count < min_points_per_pole) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                "Second peak too weak: %d points (need %d)", 
                second_peak_count, min_points_per_pole);
            
            // Stop lateral movement but maintain altitude
            if (!test_mode) {
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                float altitude_error = initial_z - current_pose.pose.position.z;
                float vz_global = 0.0;
                if (std::abs(altitude_error) > 0.05f) {
                    vz_global = 0.4f * altitude_error;
                    vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
                }
                
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = node->now();
                stop_cmd.header.frame_id = "map";
                stop_cmd.twist.linear.z = vz_global;
                node->publishLocalVelocity(stop_cmd);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // ===== STEP 4: Determine left & right poles =====
        
        int left_idx = (first_peak_idx < second_peak_idx) ? first_peak_idx : second_peak_idx;
        int right_idx = (first_peak_idx < second_peak_idx) ? second_peak_idx : first_peak_idx;
        
        float left_y = y_min + (left_idx + 0.5f) * bin_size;  // Center of bin
        float right_y = y_min + (right_idx + 0.5f) * bin_size;
        
        float detected_width = right_y - left_y;
        
        // ===== STEP 5: Validate gate width =====
        
        if (detected_width < gate_width_min || detected_width > gate_width_max) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                "Invalid gate width: %.2fm (expected %.2f-%.2fm)",
                detected_width, gate_width_min, gate_width_max);
            
            // Stop lateral movement but maintain altitude
            if (!test_mode) {
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                float altitude_error = initial_z - current_pose.pose.position.z;
                float vz_global = 0.0;
                if (std::abs(altitude_error) > 0.05f) {
                    vz_global = 0.4f * altitude_error;
                    vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
                }
                
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = node->now();
                stop_cmd.header.frame_id = "map";
                stop_cmd.twist.linear.z = vz_global;
                node->publishLocalVelocity(stop_cmd);
            }
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // ===== STEP 6: Calculate lateral offset =====
        
        float center_y = (left_y + right_y) / 2.0f;
        float lateral_error = center_y;  // Positive = gate to the right
        
        // ===== STEP 7: Check if centered =====
        
        bool is_centered = (std::abs(lateral_error) < tolerance);
        
        if (is_centered) {
            consecutive_centered_count++;
            
            if (test_mode) {
                RCLCPP_INFO(node->get_logger(),
                    "CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
                    consecutive_centered_count, required_centered_frames,
                    left_y, right_y, detected_width, lateral_error);
            } else {
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                    "CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm | Pts[L:%d R:%d]",
                    consecutive_centered_count, required_centered_frames,
                    left_y, right_y, detected_width, lateral_error,
                    bins[left_idx], bins[right_idx]);
            }
            
            // Stop lateral movement when centered, but maintain altitude!
            if (!test_mode) {
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                
                // Altitude control (gentle correction to maintain initial altitude)
                float altitude_error = initial_z - current_pose.pose.position.z;
                float vz_global = 0.0;
                if (std::abs(altitude_error) > 0.05f) {  // Tighter tolerance when centered
                    vz_global = 0.4f * altitude_error;  // Slightly more aggressive
                    vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
                }
                
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = node->now();
                stop_cmd.header.frame_id = "map";
                stop_cmd.twist.linear.x = 0.0;  // No forward/backward
                stop_cmd.twist.linear.y = 0.0;  // No lateral movement
                stop_cmd.twist.linear.z = vz_global;  // Maintain altitude!
                node->publishLocalVelocity(stop_cmd);
            }
            
            if (consecutive_centered_count >= required_centered_frames) {
                if (test_mode) {
                    RCLCPP_INFO(node->get_logger(), "TEST MODE: Centering validation SUCCESSFUL!");
                } else {
                    RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESSFUL!");
                }
                status = true;
                break;
            }
        } else {
            consecutive_centered_count = 0;
            
            // ===== STEP 8: Calculate and publish velocity command =====
            
            // In BODY frame: X=forward, Y=left, Z=up
            // lateral_error > 0 means gate is to the RIGHT, need to move RIGHT (negative Y in body frame... wait no!)
            // Actually in Livox/ROS convention:
            // - Y positive = left side
            // - Y negative = right side
            // So if gate center has positive Y, it's on the left, we need to move left (positive Y velocity)
            // If gate center has negative Y, it's on the right, we need to move right (negative Y velocity)
            // Simple: velocity should match the error sign (no negative!)
            
            float target_velocity_body_y = proportional_gain * lateral_error;  // No negative sign!
            
            // Apply velocity limits
            target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
            if (test_mode) {
                std::string direction = (lateral_error > 0) ? "RIGHT" : "LEFT";
                
                RCLCPP_INFO(node->get_logger(),
                    "NOT CENTERED | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm %s | Vel:%.3fm/s",
                    left_y, right_y, detected_width, lateral_error, direction.c_str(), target_velocity_body_y);
            }
            
            if (!test_mode) {
                // Get current pose for frame transformation
                geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                float current_yaw = getHeading(current_pose.pose.orientation);
                
                // Transform velocity from BODY frame to GLOBAL frame (map)
                // This is critical for correct movement regardless of drone orientation!
                // Global frame transformation:
                // vx_global = vx_body * cos(yaw) - vy_body * sin(yaw)
                // vy_global = vx_body * sin(yaw) + vy_body * cos(yaw)
                float vx_body = 0.0;  // No forward/backward during centering
                float vy_body = target_velocity_body_y;
                
                float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
                float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
                // Altitude control (gentle correction)
                float altitude_error = initial_z - current_pose.pose.position.z;
                float vz_global = 0.0;
                if (std::abs(altitude_error) > 0.1f) {
                    vz_global = 0.3f * altitude_error;  // Gentle proportional control
                    vz_global = std::max(-0.2f, std::min(0.2f, vz_global));
                }
                
                // Publish velocity command in GLOBAL frame (map/odom)
                geometry_msgs::msg::TwistStamped twist_cmd;
                twist_cmd.header.stamp = node->now();
                twist_cmd.header.frame_id = "map";  // Global frame for correct orientation
                twist_cmd.twist.linear.x = vx_global;
                twist_cmd.twist.linear.y = vy_global;
                twist_cmd.twist.linear.z = vz_global;
                
                node->publishLocalVelocity(twist_cmd);
                
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                    "Centering: Gate[L:%.2f R:%.2f W:%.2fm] | Err:%.3fm | Vel_body_y:%.3f | Vel_map[x:%.3f y:%.3f] | Yaw:%.1f° | Pts[L:%d R:%d]",
                    left_y, right_y, detected_width, lateral_error, target_velocity_body_y, 
                    vx_global, vy_global, current_yaw * 180.0 / M_PI,
                    bins[left_idx], bins[right_idx]);
            }
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    // Final stop
    if (!test_mode) {
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = node->now();
        stop_cmd.header.frame_id = "map";
        node->publishLocalVelocity(stop_cmd);
    }
    
    if (status) {
        if (test_mode) {
            RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation COMPLETED ===");
        } else {
            RCLCPP_INFO(node->get_logger(), "=== SIMPLE Livox Gate Centering COMPLETED ===");
        }
    } else {
        if (test_mode) {
            RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation FAILED ===");
        } else {
            RCLCPP_ERROR(node->get_logger(), "=== SIMPLE Livox Gate Centering FAILED ===");
        }
    }
}


void centering_artag(
    const std::shared_ptr<DroneController>& node,
    rclcpp::Rate &rate,
    float speed_xy,
    bool &status,
    float acc,
    float maxAccel,
    float x,
    float y,
    float min_center_time,
    float max_center_pitch,
    float max_center_roll,
    float hover_pitch,
    float hover_roll,
    std::string recovery_method,
    float centering_tolerance)
{
    (void)x; (void)y; // Unused but kept aja
    RCLCPP_INFO(node->get_logger(), "=== Starting ArTag Centering (PnP Meters) ===");
    RCLCPP_INFO(node->get_logger(), "Deadband (acc): %.3fm, Finish Tolerance: %.3fm", acc, centering_tolerance);
    status = false;
    auto last_detected_time = node->now();
    auto last_center_time = node->now();
    bool centered = false;
    geometry_msgs::msg::TwistStamped velocity_msg;
    geometry_msgs::msg::TwistStamped last_vel;
    geometry_msgs::msg::Point error_p, last_error, error_d;
    bool have_last_error = false;
    const float Kp = 1.0f;
    const float Kd = 0.1f;
    while (rclcpp::ok()) {
        auto artag_pose = node->getCurrentPoseArTag();
        if (artag_pose.pose.position.x != 0.0 || artag_pose.pose.position.y != 0.0) {
            last_detected_time = node->now();
            // Mapping Downward Camera to Body Frame:
            // Body Forward (X) = -Camera Y
            // Body Left (Y)    = -Camera X
            error_p.x = -artag_pose.pose.position.y;
            error_p.y = -artag_pose.pose.position.x;
            if (have_last_error) {
                error_d.x = (error_p.x - last_error.x) * RATE;
                error_d.y = (error_p.y - last_error.y) * RATE;
            } else {
                error_d.x = 0; error_d.y = 0;
                have_last_error = true;
            }
            last_error = error_p;
            float dist_xy = std::hypot(error_p.x, error_p.y);
            // PD Control with Deadband
            float cmd_forward = Kd * error_d.x;
            float cmd_left = Kd * error_d.y;
            if (dist_xy > acc) {
                cmd_forward += Kp * error_p.x;
                cmd_left += Kp * error_p.y;
            }
            velocity_msg.twist.linear.x = cmd_forward;
            velocity_msg.twist.linear.y = cmd_left;
            velocity_msg.twist.linear.z = 0.0;
            // Smooth velocity
            velocity_msg = limitDelta(maxAccel / RATE, velocity_msg, last_vel);
            velocity_msg = limit(speed_xy, velocity_msg);
            node->publishLocalVelocity(velocity_msg);
            last_vel = velocity_msg;
            // Check centering conditions
            float roll = std::abs(getRoll(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_roll;
            float pitch = std::abs(getPitch(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_pitch;
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500, "ArTag: dist=%.3f, roll=%.1f, pitch=%.1f", dist_xy, roll, pitch);
            if (dist_xy <= centering_tolerance && roll <= max_center_roll && pitch <= max_center_pitch) {
                if (!centered) {
                    centered = true;
                    last_center_time = node->now();
                }
                if ((node->now() - last_center_time).seconds() >= min_center_time) {
                    RCLCPP_INFO(node->get_logger(), "++++++++++++ ArTag CENTERED! ++++++++++++");
                    status = true;
                    break;
                }
            } else {
                centered = false;
            }
            
        } else {
            // Tag Lost
            centered = false;
            have_last_error = false;
            auto time_lost = (node->now() - last_detected_time).seconds();
            if (time_lost > 30.0) {
                RCLCPP_WARN(node->get_logger(), "ArTag lost > 30s, aborting.");
                break;
            }
            
            if (recovery_method == "local_pose") {
                auto last_nonzero = node->getLastNonZeroPoseArTag();
                if (last_nonzero.pose.position.x != 0.0) {
                    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Lost, returning to site...");
                    // Use standard moveToPoint to return to detection site
                    // We need to transform the last seen tag error relative to the drone pose at that moment
                    auto drone_pose_at_detection = node->getDroneNonZeroPoseArTag();
                    geometry_msgs::msg::Point tag_error_body;
                    tag_error_body.x = -last_nonzero.pose.position.y;
                    tag_error_body.y = -last_nonzero.pose.position.x;                   
                    float heading = getHeading(drone_pose_at_detection.pose.orientation);
                    auto tag_pos_global = drone_pose_at_detection.pose.position + rotatePoint(tag_error_body, heading);                   
                    geometry_msgs::msg::Pose target;
                    target.position = tag_pos_global;
                    target.orientation = drone_pose_at_detection.pose.orientation;
                    // Small step towards target, enabling ArTag interrupt
                    moveToPoint(node, rate, drone_pose_at_detection, target, 0.3, acc*2.0, false, false, true);
                }
            }
            node->publishLocalVelocity(zero(velocity_msg));
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
    node->publishLocalVelocity(zero(velocity_msg));
    RCLCPP_INFO(node->get_logger(), "ArTag centering finished. Status: %s", status ? "SUCCESS" : "FAILED");
}

void centering_object(
    const std::shared_ptr<DroneController>& node,
    rclcpp::Rate &rate,
    float speed_xy,
    bool &status,
    float acc,
    float maxAccel,
    float x,
    float y,
    float min_center_time,
    float max_center_pitch,
    float max_center_roll,
    float hover_pitch,
    float hover_roll,
    std::string recovery_method,
    float centering_tolerance)
{
    (void)x; (void)y;
    RCLCPP_INFO(node->get_logger(), "=== Starting Object Centering (PnP Meters) ===");
    RCLCPP_INFO(node->get_logger(), "Deadband: %.3fm  Finish tolerance: %.3fm", acc, centering_tolerance);
    status = false;
    auto last_detected_time = node->now();
    auto last_center_time = node->now();
    bool centered = false;
    geometry_msgs::msg::TwistStamped velocity_msg;
    geometry_msgs::msg::TwistStamped last_vel;
    geometry_msgs::msg::Point error_p, last_error, error_d;
    bool have_last_error = false;
    const float Kp = 1.0f;
    const float Kd = 0.1f;

    while (rclcpp::ok()) {
        auto payload_pose = node->getCurrentPosePayload();
        if (payload_pose.pose.position.x != 0.0 || payload_pose.pose.position.y != 0.0) {
            last_detected_time = node->now();

            // Downward camera → body frame: body_x = -cam_y, body_y = -cam_x
            error_p.x = -payload_pose.pose.position.y;
            error_p.y = -payload_pose.pose.position.x;

            if (have_last_error) {
                error_d.x = (error_p.x - last_error.x) * RATE;
                error_d.y = (error_p.y - last_error.y) * RATE;
            } else {
                error_d.x = 0.0; error_d.y = 0.0;
                have_last_error = true;
            }
            last_error = error_p;

            const float dist_xy = std::hypot(error_p.x, error_p.y);

            // PD control with deadband on P term
            float cmd_x = Kd * error_d.x;
            float cmd_y = Kd * error_d.y;
            if (dist_xy > acc) {
                cmd_x += Kp * error_p.x;
                cmd_y += Kp * error_p.y;
            }

            velocity_msg.twist.linear.x = cmd_x;
            velocity_msg.twist.linear.y = cmd_y;
            velocity_msg.twist.linear.z = 0.0;
            velocity_msg = limitDelta(maxAccel / RATE, velocity_msg, last_vel);
            velocity_msg = limit(speed_xy, velocity_msg);
            node->publishLocalVelocity(velocity_msg);
            last_vel = velocity_msg;

            const float roll  = std::abs(getRoll(node->getCurrentLocalPose().pose.orientation)  * 180.0f / M_PI) - hover_roll;
            const float pitch = std::abs(getPitch(node->getCurrentLocalPose().pose.orientation) * 180.0f / M_PI) - hover_pitch;

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                "Object centering: dist=%.3fm  roll=%.1f  pitch=%.1f  vx=%.3f  vy=%.3f",
                dist_xy, roll, pitch, velocity_msg.twist.linear.x, velocity_msg.twist.linear.y);

            if (dist_xy <= centering_tolerance && roll <= max_center_roll && pitch <= max_center_pitch) {
                if (!centered) {
                    centered = true;
                    last_center_time = node->now();
                }
                if ((node->now() - last_center_time).seconds() >= min_center_time) {
                    RCLCPP_INFO(node->get_logger(), "++++++++++++ Object CENTERED! ++++++++++++");
                    status = true;
                    break;
                }
            } else {
                centered = false;
            }

        } else {
            // Object lost
            centered = false;
            have_last_error = false;
            const double time_lost = (node->now() - last_detected_time).seconds();

            if (time_lost > 30.0) {
                RCLCPP_WARN(node->get_logger(), "Object lost > 30s, aborting centering.");
                break;
            }

            if (recovery_method == "local_pose") {
                const auto last_nonzero = node->getLastNonZeroPosePayload();
                if (last_nonzero.pose.position.x != 0.0 || last_nonzero.pose.position.y != 0.0) {
                    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                        "Object lost, returning to last detection site...");
                    auto drone_at_detection = node->getDroneNonZeroPosePayload();
                    geometry_msgs::msg::Point obj_body;
                    obj_body.x = -last_nonzero.pose.position.y;
                    obj_body.y = -last_nonzero.pose.position.x;
                    const float heading = getHeading(drone_at_detection.pose.orientation);
                    geometry_msgs::msg::Pose target;
                    target.position = drone_at_detection.pose.position + rotatePoint(obj_body, heading);
                    target.orientation = drone_at_detection.pose.orientation;
                    // allow_centering_payload=true so we break immediately when object reappears
                    moveToPoint(node, rate, drone_at_detection, target, 0.3f, acc * 2.0f, true, false, false);
                }
            }
            node->publishLocalVelocity(zero(velocity_msg));
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    node->publishLocalVelocity(zero(velocity_msg));
    node->resetNonzeroPayloadPose();
    RCLCPP_INFO(node->get_logger(), "Object centering finished. Status: %s", status ? "SUCCESS" : "FAILED");
}

void correct_heading_artag(
    const std::shared_ptr<DroneController>& node,
    rclcpp::Rate &rate,
    bool &status,
    float yaw_acc,
    float min_align_time,
    float timeout,
    float hover_pitch,
    float hover_roll)
{
    RCLCPP_INFO(node->get_logger(), "=== Starting ArTag Heading Correction (Closest 90 Deg) ===");
    status = false;
    auto start_time = node->now();
    auto last_detected_time = node->now();
    auto last_align_time = node->now();
    bool aligned = false;
    geometry_msgs::msg::TwistStamped velocity_msg;
    float last_yaw_error = 0.0f;
    bool have_last_error = false;
    // Control Gains
    const float Kp_yaw = 0.6f;
    const float Kd_yaw = 0.15f;
    while (rclcpp::ok()) {
        auto elapsed = (node->now() - start_time).seconds();
        if (elapsed > timeout) {
            RCLCPP_WARN(node->get_logger(), "Heading correction timeout.");
            break;
        }

        auto artag_pose = node->getCurrentPoseArTag();

        if (artag_pose.pose.position.x != 0.0 || artag_pose.pose.position.y != 0.0) {
            last_detected_time = node->now();

            // 1. Get raw relative yaw from ArTag detection
            double tag_yaw_raw = getHeading(artag_pose.pose.orientation);

            // 2. Shortest path to any 90-degree edge (0, 90, 180, 270)
            // Trick: atan2(sin(4*theta), cos(4*theta)) / 4
            float yaw_error = static_cast<float>(atan2(sin(4.0 * tag_yaw_raw), cos(4.0 * tag_yaw_raw)) / 4.0);

            // 3. PD Control
            float d_error = 0.0f;
            if (have_last_error) {
                d_error = (yaw_error - last_yaw_error) * RATE;
            }
            have_last_error = true;
            last_yaw_error = yaw_error;

            float cmd_yaw = (Kp_yaw * yaw_error) + (Kd_yaw * d_error);
            
            // Limit rotation speed for safety
            cmd_yaw = std::max(-0.6f, std::min(0.6f, cmd_yaw));

            velocity_msg.twist.linear.x = 0.0;
            velocity_msg.twist.linear.y = 0.0;
            velocity_msg.twist.linear.z = 0.0;
            velocity_msg.twist.angular.z = cmd_yaw;

            node->publishLocalVelocity(velocity_msg);

            // 4. Check alignment conditions
            float yaw_err_deg = std::abs(yaw_error * 180.0 / M_PI);
            float roll = std::abs(getRoll(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_roll;
            float pitch = std::abs(getPitch(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_pitch;

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                "Heading Alignment: err=%.2f deg, roll=%.1f, pitch=%.1f", yaw_err_deg, roll, pitch);

            if (yaw_err_deg <= yaw_acc && roll <= 2.5 && pitch <= 2.5) {
                if (!aligned) {
                    aligned = true;
                    last_align_time = node->now();
                }
                if ((node->now() - last_align_time).seconds() >= min_align_time) {
                    RCLCPP_INFO(node->get_logger(), "Heading ALIGNED to ArTag edge!");
                    status = true;
                    break;
                }
            } else {
                aligned = false;
            }

        } else {
            // Tag Lost
            aligned = false;
            have_last_error = false;
            velocity_msg.twist.angular.z = 0.0;
            node->publishLocalVelocity(velocity_msg);

            if ((node->now() - last_detected_time).seconds() > 5.0) {
                RCLCPP_WARN(node->get_logger(), "ArTag lost during heading correction.");
                break;
            }
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    velocity_msg.twist.angular.z = 0.0;
    node->publishLocalVelocity(velocity_msg);
}

void CorrectHeading(
    const std::shared_ptr<DroneController>& node,
    rclcpp::Rate &rate,
    bool &status,
    float yaw_acc,
    float min_align_time,
    float timeout,
    float hover_pitch,
    float hover_roll)
{
    RCLCPP_INFO(node->get_logger(), "=== Starting Absolute ArTag Heading Alignment (CorrectHeading) ===");
    status = false;
    auto start_time = node->now();
    auto last_detected_time = node->now();
    auto last_align_time = node->now();
    bool aligned = false;

    // MANDATORY STABILIZATION: Wait 1s
    RCLCPP_INFO(node->get_logger(), "Stabilizing for 1.0s before alignment...");
    for(int i=0; i<10 && rclcpp::ok(); ++i) {
        geometry_msgs::msg::TwistStamped zero_vel;
        node->publishLocalVelocity(zero_vel);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    geometry_msgs::msg::TwistStamped velocity_msg;
    float last_yaw_error = 0.0f;
    float yaw_error_integral = 0.0f;
    bool have_last_error = false;

    // PID Control Gains
    const float Kp_yaw = 0.65f;
    const float Ki_yaw = 0.05f; 
    const float Kd_yaw = 0.18f;
    const float max_i = 0.2f;

    while (rclcpp::ok()) {
        auto elapsed = (node->now() - start_time).seconds();
        if (elapsed > timeout) {
            RCLCPP_WARN(node->get_logger(), "Heading alignment timeout.");
            break;
        }

        auto artag_pose = node->getCurrentPoseArTag();

        if (artag_pose.pose.position.x != 0.0 || artag_pose.pose.position.y != 0.0) {
            last_detected_time = node->now();

            // Extract relative orientation
            double tag_roll = getRoll(artag_pose.pose.orientation) * 180.0 / M_PI;
            double tag_pitch = getPitch(artag_pose.pose.orientation) * 180.0 / M_PI;
            double tag_yaw_raw_rad = getHeading(artag_pose.pose.orientation);
            double tag_yaw_deg = tag_yaw_raw_rad * 180.0 / M_PI;

            // Absolute shortest path to tag's zero heading (no sin(4*theta) symmetry)
            float yaw_error = static_cast<float>(atan2(sin(tag_yaw_raw_rad), cos(tag_yaw_raw_rad)));

            // PID Calculation
            float d_error = 0.0f;
            if (have_last_error) {
                d_error = (yaw_error - last_yaw_error) * RATE;
                yaw_error_integral += yaw_error / RATE;
                yaw_error_integral = std::max(-max_i, std::min(max_i, yaw_error_integral));
            }
            have_last_error = true;
            last_yaw_error = yaw_error;

            float cmd_yaw = (Kp_yaw * yaw_error) + (Ki_yaw * yaw_error_integral) + (Kd_yaw * d_error);
            cmd_yaw = std::max(-0.6f, std::min(0.6f, cmd_yaw));

            velocity_msg.twist.linear.x = 0.0;
            velocity_msg.twist.linear.y = 0.0;
            velocity_msg.twist.linear.z = 0.0;
            velocity_msg.twist.angular.z = cmd_yaw;

            node->publishLocalVelocity(velocity_msg);

            // Alignment metrics
            float yaw_err_deg = std::abs(yaw_error * 180.0 / M_PI);
            float drone_roll = std::abs(getRoll(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_roll;
            float drone_pitch = std::abs(getPitch(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_pitch;

            RCLCPP_INFO(node->get_logger(),
                "ALIGN TAG: Y:%.1f deg | Err:%.2f deg | Cmd:%.2f | Drone: R:%.1f P:%.1f",
                tag_yaw_deg, yaw_err_deg, cmd_yaw, drone_roll, drone_pitch);

            // Success Gate
            if (yaw_err_deg <= yaw_acc && drone_roll <= 3.0 && drone_pitch <= 3.0 && (node->now() - start_time).seconds() > 2.0) {
                if (!aligned) {
                    aligned = true;
                    last_align_time = node->now();
                }
                if ((node->now() - last_align_time).seconds() >= min_align_time) {
                    RCLCPP_INFO(node->get_logger(), "Heading ALIGNED to Absolute ArTag Front!");
                    status = true;
                    break;
                }
            } else {
                aligned = false;
            }

        } else {
            // Tag Lost
            aligned = false;
            have_last_error = false;
            yaw_error_integral = 0;
            velocity_msg.twist.angular.z = 0.0;
            node->publishLocalVelocity(velocity_msg);

            if ((node->now() - last_detected_time).seconds() > 5.0) {
                RCLCPP_WARN(node->get_logger(), "ArTag lost during absolute heading alignment.");
                break;
            }
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    velocity_msg.twist.angular.z = 0.0;
    node->publishLocalVelocity(velocity_msg);
}
void reOrientation(
    const std::shared_ptr<DroneController>& node,
    rclcpp::Rate &rate,
    bool &status,
    float yaw_acc,
    float min_align_time,
    float timeout,
    float hover_pitch,
    float hover_roll)
{
    RCLCPP_INFO(node->get_logger(), "=== Starting ArTag reOrientation (Centering Logic) ===");
    status = false;
    auto start_time = node->now();
    auto last_detected_time = node->now();
    auto last_center_time = node->now();
    bool centered = false;

    geometry_msgs::msg::TwistStamped velocity_msg;
    float error_p = 0.0f;
    float last_error = 0.0f;
    float error_d = 0.0f;
    bool have_last_error = false;

    // Control Gains (Aggressive enough but damped)
    const float Kp = 0.8f;
    const float Kd = 0.12f;

    while (rclcpp::ok()) {
        auto elapsed = (node->now() - start_time).seconds();
        if (elapsed > timeout) {
            RCLCPP_WARN(node->get_logger(), "reOrientation timeout.");
            break;
        }

        auto artag_pose = node->getCurrentPoseArTag();

        if (artag_pose.pose.position.x != 0.0 || artag_pose.pose.position.y != 0.0) {
            last_detected_time = node->now();

            double theta = getHeading(artag_pose.pose.orientation);
            error_p = static_cast<float>(atan2(sin(theta), cos(theta)));

            if (have_last_error) {
                error_d = (error_p - last_error) * RATE;
            } else {
                error_d = 0;
                have_last_error = true;
            }
            last_error = error_p;

            float cmd_yaw = (Kp * error_p) + (Kd * error_d);
            cmd_yaw = std::max(-0.6f, std::min(0.6f, cmd_yaw));

            velocity_msg.twist.linear.x = 0.0;
            velocity_msg.twist.linear.y = 0.0;
            velocity_msg.twist.linear.z = 0.0;
            velocity_msg.twist.angular.z = cmd_yaw;

            node->publishLocalVelocity(velocity_msg);

            float yaw_err_deg = std::abs(error_p * 180.0 / M_PI);
            float drone_roll = std::abs(getRoll(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_roll;
            float drone_pitch = std::abs(getPitch(node->getCurrentLocalPose().pose.orientation) * 180.0 / M_PI) - hover_pitch;

            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                "reOrientation: err=%.2f deg, roll=%.1f, pitch=%.1f", yaw_err_deg, drone_roll, drone_pitch);

            if (yaw_err_deg <= yaw_acc && drone_roll <= 3.0 && drone_pitch <= 3.0) {
                if (!centered) {
                    centered = true;
                    last_center_time = node->now();
                }
                if ((node->now() - last_center_time).seconds() >= min_align_time) {
                    RCLCPP_INFO(node->get_logger(), "++++++++++++ Heading reOriented! ++++++++++++");
                    status = true;
                    break;
                }
            } else {
                centered = false;
            }

        } else {
            centered = false;
            have_last_error = false;
            velocity_msg.twist.angular.z = 0.0;
            node->publishLocalVelocity(velocity_msg);

            if ((node->now() - last_detected_time).seconds() > 5.0) {
                RCLCPP_WARN(node->get_logger(), "ArTag lost during reOrientation.");
                break;
            }
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    velocity_msg.twist.angular.z = 0.0;
    node->publishLocalVelocity(velocity_msg);
}

void fix_alt(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float req_alt, float tolerance, float timeout) {
    geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
    float current_alt = curr_pose.pose.position.z;
    float alt_diff = req_alt - current_alt;

    RCLCPP_INFO(node->get_logger(), "=== fix_alt === current: %.3f m | requested: %.3f m | diff: %.3f m",
                current_alt, req_alt, alt_diff);

    if (std::fabs(alt_diff) < tolerance) {
        RCLCPP_INFO(node->get_logger(), "fix_alt: already at requested altitude (diff=%.3f m)", alt_diff);
        posee = node->getCurrentLocalPose();
        return;
    }

    // Hold XY, only change Z to req_alt
    geometry_msgs::msg::PoseStamped target_pose = curr_pose;
    target_pose.pose.position.z = req_alt;

    rclcpp::Rate fast_rate(20.0);
    auto start_time = node->now();
    const rclcpp::Duration timeout_dur = rclcpp::Duration::from_seconds(timeout);
    bool reached = false;

    while (rclcpp::ok()) {
        float current_z = node->getCurrentLocalPose().pose.position.z;
        float remaining = std::fabs(req_alt - current_z);

        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
            "fix_alt: z=%.3f m | target=%.3f m | diff=%.3f m", current_z, req_alt, req_alt - current_z);

        if (remaining <= tolerance) {
            reached = true;
            break;
        }

        if ((node->now() - start_time) > timeout_dur) {
            RCLCPP_WARN(node->get_logger(), "fix_alt: timeout after %.1f s", timeout);
            break;
        }

        target_pose.header.stamp = node->now();
        target_pose.header.frame_id = "map";
        node->publishLocalPosition(target_pose);

        rclcpp::spin_some(node);
        fast_rate.sleep();
    }

    if (reached) {
        RCLCPP_INFO(node->get_logger(), "fix_alt: reached %.3f m", node->getCurrentLocalPose().pose.position.z);
    }

    posee = node->getCurrentLocalPose();
}
