
void LocalMove(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, float up_z, float yaw_angle, float tolerance, float waypoint_speed) {
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    float current_yaw = getHeading(current_pose.pose.orientation);
    RCLCPP_INFO(node->get_logger(), "CURRENT HEADING: %.2f degrees", current_yaw * 180.0 / M_PI);
    float global_x = forward_x * cos(current_yaw) - left_y * sin(current_yaw);
    float global_y = forward_x * sin(current_yaw) + left_y * cos(current_yaw);
    target_pose.pose.position.x = current_pose.pose.position.x + global_x;
    target_pose.pose.position.y = current_pose.pose.position.y + global_y;
    target_pose.pose.position.z = current_pose.pose.position.z + up_z;
    const double heading_eps = 1e-4;
    const bool has_translation = std::abs(global_x) > heading_eps || std::abs(global_y) > heading_eps;
    if (has_translation) {
        const double target_yaw = std::atan2(global_y, global_x);
        setHeading(target_pose.pose.orientation, target_yaw);
        // RCLCPP_INFO(node->get_logger(), "AUTO HEADING: %.2f degrees", target_yaw * 180.0 / M_PI);
    } else if (yaw_angle != 0.0) {
        setHeading(target_pose.pose.orientation, yaw_angle * M_PI / 180.0);
    } else {
        target_pose.pose.orientation = current_pose.pose.orientation;
    }
    target_pose.header.stamp = node->now();
    target_pose.header.frame_id = "map";
    RCLCPP_INFO(node->get_logger(), "MOVING FROM (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "BODY FRAME: forward=%.2f, left=%.2f -> Global frame: x=%.2f, y=%.2f", forward_x, left_y, global_x, global_y);
    const double fast_rate_hz = node->isPx4() ? 20.0 : 10.0;
    rclcpp::Rate fast_rate(fast_rate_hz);
    rclcpp::Rate* loop_rate = node->isPx4() ? &fast_rate : &rate;
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        float current_distance = sqrt(pow(target_pose.pose.position.x - current_pose.pose.position.x, 2) + pow(target_pose.pose.position.y - current_pose.pose.position.y, 2) + pow(target_pose.pose.position.z - current_pose.pose.position.z, 2));

        if (current_distance < tolerance) {
            RCLCPP_INFO(node->get_logger(), "SAMPAIII...: %.3f m", current_distance);
            break;
        }
        geometry_msgs::msg::PoseStamped cmd_pose = target_pose;
        if (waypoint_speed > 0.0f) {
            const double dt = std::max(1e-3, 1.0 / fast_rate_hz);
            const double step_distance = waypoint_speed * dt;
            if (current_distance > step_distance) {
                const double dx = target_pose.pose.position.x - current_pose.pose.position.x;
                const double dy = target_pose.pose.position.y - current_pose.pose.position.y;
                const double dz = target_pose.pose.position.z - current_pose.pose.position.z;
                const double inv_dist = 1.0 / std::max(current_distance, 1e-6f);
                cmd_pose.pose.position.x = current_pose.pose.position.x + dx * inv_dist * step_distance;
                cmd_pose.pose.position.y = current_pose.pose.position.y + dy * inv_dist * step_distance;
                if (std::abs(up_z) < 1e-3) {
                    cmd_pose.pose.position.z = target_pose.pose.position.z;
                } else {
                    cmd_pose.pose.position.z = current_pose.pose.position.z + dz * inv_dist * step_distance;
                }
            }
        }
        cmd_pose.header.stamp = node->now();
        node->publishLocalPosition(cmd_pose);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "DIST TO TARGET: %.3f m", current_distance);
        rclcpp::spin_some(node);
        loop_rate->sleep(); 
    }
    posee = node->getCurrentLocalPose();
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
