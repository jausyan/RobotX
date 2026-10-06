#include "control_.hpp"
#include "math_.hpp"
#include "geo_.hpp"
#include "drone_controller_.hpp"
#include <iostream>
#include <string>
#define RATE 10.0

float takeoff_altitude;
float hold_time;
double survey_lat;
double survey_lon;
float survey_alt;
float survey_time;
float usv_alt;
float approach_alt;
float drop_alt;
double task2_lat;
double task2_lon;
double task3_lat;
double task3_lon;
std::string order_topic;
std::string usv_gps_topic;
float command_timeout;
float run_start_timeout;
int confirmation;
std::string target_topic_prefix;
float speed_xy;
float acc;
float maxAccel;
float X;
float Y;
float min_center_time;
float max_center_pitch;
float max_center_roll;
float hover_pitch;
float hover_roll;
float centering_tolerance_red;
std::string recovery_method;
int channel_red;
int channel_green;
int channel_blue;
int servo_buka;
int signal_repeat;
bool centered = false;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(RATE);
    auto node = std::make_shared<DroneController>();

    node->declare_parameter<float>("mission.takeoff_altitude", 5.0);
    node->declare_parameter<float>("mission.hold_time", 1.0);
    node->declare_parameter<double>("mission.survey_lat", 0.0);
    node->declare_parameter<double>("mission.survey_lon", 0.0);
    node->declare_parameter<float>("mission.survey_alt", 18.0);
    node->declare_parameter<float>("mission.survey_time", 60.0);
    node->declare_parameter<float>("mission.usv_alt", 10.0);
    node->declare_parameter<float>("mission.approach_alt", 10.0);
    node->declare_parameter<float>("mission.drop_alt", 3.0);
    node->declare_parameter<double>("mission.task2_lat", 0.0);
    node->declare_parameter<double>("mission.task2_lon", 0.0);
    node->declare_parameter<double>("mission.task3_lat", 0.0);
    node->declare_parameter<double>("mission.task3_lon", 0.0);
    node->declare_parameter<std::string>("communication.order_topic", "/mission/order");
    node->declare_parameter<std::string>("communication.usv_gps_topic", "/USV/global_position/global");
    node->declare_parameter<float>("communication.command_timeout", 600.0);
    node->declare_parameter<float>("communication.run_start_timeout", 3600.0);
    node->declare_parameter<std::string>("vision.target_topic_prefix", "/vision_geo/target/");
    node->declare_parameter<float>("centering_red.speed_xy", 0.75);
    node->declare_parameter<float>("centering_red.acc", 0.05);
    node->declare_parameter<float>("centering_red.maxAccel", 0.20);
    node->declare_parameter<float>("centering_red.X", 0.0);
    node->declare_parameter<float>("centering_red.Y", 0.0);
    node->declare_parameter<float>("centering_red.min_center_time", 0.2);
    node->declare_parameter<float>("centering_red.max_center_pitch", 2.5);
    node->declare_parameter<float>("centering_red.max_center_roll", 2.5);
    node->declare_parameter<float>("centering_red.hover_pitch", 0.0);
    node->declare_parameter<float>("centering_red.hover_roll", 0.0);
    node->declare_parameter<float>("centering_red.centering_tolerance", 0.15);
    node->declare_parameter<std::string>("centering_red.recovery_method", "local_pose");
    node->declare_parameter<int>("servo.channel_red", 9);
    node->declare_parameter<int>("servo.channel_green", 10);
    node->declare_parameter<int>("servo.channel_blue", 11);
    node->declare_parameter<int>("servo.servo_buka", 1900);
    node->declare_parameter<int>("servo.signal_repeat", 3);

    node->get_parameter("mission.takeoff_altitude", takeoff_altitude);
    node->get_parameter("mission.hold_time", hold_time);
    node->get_parameter("mission.survey_lat", survey_lat);
    node->get_parameter("mission.survey_lon", survey_lon);
    node->get_parameter("mission.survey_alt", survey_alt);
    node->get_parameter("mission.survey_time", survey_time);
    node->get_parameter("mission.usv_alt", usv_alt);
    node->get_parameter("mission.approach_alt", approach_alt);
    node->get_parameter("mission.drop_alt", drop_alt);
    node->get_parameter("mission.task2_lat", task2_lat);
    node->get_parameter("mission.task2_lon", task2_lon);
    node->get_parameter("mission.task3_lat", task3_lat);
    node->get_parameter("mission.task3_lon", task3_lon);
    node->get_parameter("communication.order_topic", order_topic);
    node->get_parameter("communication.usv_gps_topic", usv_gps_topic);
    node->get_parameter("communication.command_timeout", command_timeout);
    node->get_parameter("communication.run_start_timeout", run_start_timeout);
    node->get_parameter("vision.target_topic_prefix", target_topic_prefix);
    node->get_parameter("centering_red.speed_xy", speed_xy);
    node->get_parameter("centering_red.acc", acc);
    node->get_parameter("centering_red.maxAccel", maxAccel);
    node->get_parameter("centering_red.X", X);
    node->get_parameter("centering_red.Y", Y);
    node->get_parameter("centering_red.min_center_time", min_center_time);
    node->get_parameter("centering_red.max_center_pitch", max_center_pitch);
    node->get_parameter("centering_red.max_center_roll", max_center_roll);
    node->get_parameter("centering_red.hover_pitch", hover_pitch);
    node->get_parameter("centering_red.hover_roll", hover_roll);
    node->get_parameter("centering_red.centering_tolerance", centering_tolerance_red);
    node->get_parameter("centering_red.recovery_method", recovery_method);
    node->get_parameter("servo.channel_red", channel_red);
    node->get_parameter("servo.channel_green", channel_green);
    node->get_parameter("servo.channel_blue", channel_blue);
    node->get_parameter("servo.servo_buka", servo_buka);
    node->get_parameter("servo.signal_repeat", signal_repeat);

    RCLCPP_INFO(node->get_logger(), "=== BISMILLAH ROBOTX UAV MISSION ===");
    RCLCPP_INFO(node->get_logger(), "TAKEOFF ALT : %.2f m", takeoff_altitude);
    RCLCPP_INFO(node->get_logger(), "SURVEY      : lat=%.7f lon=%.7f alt=%.1f m (%.0f s)", survey_lat, survey_lon, survey_alt, survey_time);
    RCLCPP_INFO(node->get_logger(), "ORDER TOPIC : %s", order_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "USV TOPIC   : %s", usv_gps_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "DROP        : approach %.1f m, drop %.1f m", approach_alt, drop_alt);

    geometry_msgs::msg::PoseStamped posee;
    std::string command;
    MissionOrder order;
    std::string target;
    int channel;
    initFrame(node, posee);

    while (rclcpp::ok() && node->getCurrentLocalPose().pose.position.z == 0.0) {
        RCLCPP_INFO(node->get_logger(), "Wait local pose data...");
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // ============================== PRE-RUN =====================================
    // team rule: autonomous mode, then hold until the GCS sends CMD_RUN_START
    setTask(node, TASK_NONE);
    setMode(node, rate, "GUIDED");
    RCLCPP_INFO(node->get_logger(), "PRE-RUN CONFIRMATION 1 / 0? (1 : WAIT FOR RUN START, 0 : ABORT)");
    std::cin >> confirmation;
    if (confirmation != 1) {
        RCLCPP_WARN(node->get_logger(), "RUN CANCELLED..!!");
        rclcpp::shutdown();
        return 0;
    }
    RCLCPP_INFO(node->get_logger(), "PRE-RUN: ON THE GROUND, WAITING FOR RUN START...");
    if (!waitCommand(node, "RUN-START", order_topic, run_start_timeout, false)) {
        RCLCPP_ERROR(node->get_logger(), "=========== NO RUN START, ABORT (drone not armed) ===========");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "TAKING OFF.... %.2f meters...", takeoff_altitude);
    takeoff(node, rate, posee, takeoff_altitude, false);
    holdPosition(node, rate, posee, hold_time);

    // ============================== TASK 1 ======================================
    TASK_1:
    setTask(node, TASK_SAFE_PASSAGE);
    RCLCPP_INFO(node->get_logger(), "TASK 1: FLY TO SURVEY POINT");
    clearMission(node);
    pushMission(node, {create_waypoint(survey_lat, survey_lon, survey_alt)});
    setMode(node, rate, "AUTO");
    waitForWP(node, rate, 1);
    setMode(node, rate, "GUIDED");
    holdPosition(node, rate, posee, 1.0);
    fix_alt(node, rate, posee, survey_alt, 0.3, 60.0);
    holdPosition(node, rate, posee, 3.0);
    RCLCPP_INFO(node->get_logger(), "TASK 1: SURVEY ALT REACHED, STARTING VISION...");
    pubCommand(node, "UAV-GO", order_topic);
    waitCommand(node, "MISSION-DONE", "/mission/order", 300.0, true);
    setTask(node, TASK_NONE);
    RCLCPP_INFO(node->get_logger(), "TASK 1: MAPPING DONE, GOING TO USV POSE...");
    // goToVehicle(node, rate, posee, takeoff_altitude, usv_gps_topic);
    centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, "/vision_geo/target/buoy_red");
    fix_alt(node, rate, posee, drop_alt);
    centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, "/vision_geo/target/buoy_red");
    fix_alt(node, rate, posee, 3.0);
    if (centered) {
        channel = order.tin == "green" ? channel_green : order.tin == "blue" ? channel_blue : channel_red;
        //controlServoRepeated(node, channel, servo_tutup, signal_repeat);
        holdPosition(node, rate, posee, hold_time);
    } else {
        RCLCPP_ERROR(node->get_logger(), "TASK 2: CENTERING FAILED - SKIP DROP..!!");
    }

    // ============================== TASK 2 ======================================
    // TASK_2:
    // RCLCPP_INFO(node->get_logger(), "TASK 2: WAITING FOR ORDER...");
    // waitCommand(node, "UAV-GO", order_topic, command_timeout, true, &command);
    // order = parseCommand(command);
    // target = "tin_" + order.tin;
    // RCLCPP_INFO(node->get_logger(), "TASK 2: TIN %s → %s", order.tin.c_str(), target.c_str());
    // holdPosition(node, rate, posee, hold_time);

    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target);
    // fix_alt(node, rate, posee, drop_alt);
    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target);
    // if (centered) {
    //     channel = order.tin == "green" ? channel_green : order.tin == "blue" ? channel_blue : channel_red;
    //     controlServoRepeated(node, channel, servo_tutup, signal_repeat);
    //     holdPosition(node, rate, posee, hold_time);
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "TASK 2: CENTERING FAILED - SKIP DROP..!!");
    // }
    // fix_alt(node, rate, posee, takeoff_altitude);

    // target_circle = "circle_" + order.circle;
    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target_circle);
    // fix_alt(node, rate, posee, drop_alt);
    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target_circle);
    // if (centered) {
    //     channel = order.tin == "green" ? channel_green : order.tin == "blue" ? channel_blue : channel_red;
    //     controlServoRepeated(node, channel, servo_buka, signal_repeat);
    //     holdPosition(node, rate, posee, hold_time);
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "TASK 2: CENTERING FAILED - SKIP DROP..!!");
    // }

    // goToVehicle(node, rate, posee, takeoff_altitude, usv_gps_topic);

    // // ============================== TASK 3 ======================================
    // TASK_3:
    // RCLCPP_INFO(node->get_logger(), "TASK 3: WAITING FOR ORDER...");
    // waitCommand(node, "UAV-GO", order_topic, command_timeout, true, &command);
    // order = parseCommand(command);
    // target = "tin_" + order.tin;
    // RCLCPP_INFO(node->get_logger(), "TASK 3: TIN %s → %s", order.tin.c_str(), target.c_str());
    // holdPosition(node, rate, posee, hold_time);

    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target);
    // fix_alt(node, rate, posee, drop_alt);
    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target);
    // if (centered) {
    //     channel = order.tin == "green" ? channel_green : order.tin == "blue" ? channel_blue : channel_red;
    //     controlServoRepeated(node, channel, servo_tutup, signal_repeat);
    //     holdPosition(node, rate, posee, hold_time);
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "TASK 2: CENTERING FAILED - SKIP DROP..!!");
    // }
    // fix_alt(node, rate, posee, takeoff_altitude);

    // target_circle = "circle_" + order.circle;
    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target_circle);
    // fix_alt(node, rate, posee, drop_alt);
    // centering_red(node, rate, speed_xy, centered, acc, maxAccel, X, Y, min_center_time, max_center_pitch, max_center_roll, hover_pitch, hover_roll, recovery_method, centering_tolerance_red, target_topic_prefix + target_circle);
    // if (centered) {
    //     channel = order.tin == "green" ? channel_green : order.tin == "blue" ? channel_blue : channel_red;
    //     controlServoRepeated(node, channel, servo_buka, signal_repeat);
    //     holdPosition(node, rate, posee, hold_time);
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "TASK 2: CENTERING FAILED - SKIP DROP..!!");
    // }

    setTask(node, TASK_NONE);
    RCLCPP_INFO(node->get_logger(), "ALL TASKS DONE, RTL...");
    setMode(node, rate, "rtl");
    RCLCPP_INFO(node->get_logger(), "=========== ALHAMDULILLAH MISSION COMPLETE ===========");
    rclcpp::shutdown();
    return 0;
}
