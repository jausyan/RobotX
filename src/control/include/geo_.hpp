#pragma once

#include <cmath>
#include <tuple>
#include <vector>
#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <stdexcept>
#include <limits>

constexpr double EARTH_RADIUS = 6378137.0; // meters

struct LatLonAlt {
    double lat;
    double lon;
    double alt;
};

struct XYZ {
    double x;
    double y;
    double z;
};

// Conversion functions
LatLonAlt convert_xy_to_lat_long(double x, double y, double z = 0.0, LatLonAlt zero = {-35.0, 150.0, 600.0}, double zero_hdg = 0.0);

XYZ convert_lat_long_to_xy(double lat, double lon, double alt, LatLonAlt zero = {-35.0, 150.0, 600.0});

// MAVROS waypoint creation
mavros_msgs::msg::Waypoint create_waypoint(double lat, double lon, double alt,
    double acc_radius = 2.0, double pass_radius = 0.0,
    double yaw = NAN, bool is_current = false, bool autocontinue = true);

mavros_msgs::msg::Waypoint create_waypoint_from_xyz(double x, double y, double z = 0.0,
    LatLonAlt zero = {-35.0, 150.0, 600.0}, double zero_hdg = 0.0,
    double acc_radius = 2.0, double pass_radius = 0.0,
    double yaw = NAN, bool is_current = false, bool autocontinue = true);

// New: GlobalPositionTarget creation from XYZ
mavros_msgs::msg::GlobalPositionTarget create_globalpos_from_xyz(double x, double y, double z = 0.0,
    LatLonAlt zero = {-35.0, 150.0, 600.0}, double zero_hdg = 0.0);

// MAVROS takeoff command
mavros_msgs::msg::Waypoint get_takeoff_command(double alt,
    double pitch = 0.0, double yaw = NAN,
    bool is_current = false, bool autocontinue = true);

// Quaternion helpers
std::vector<double> quaternion_mult(const std::vector<double>& q, const std::vector<double>& r);
geometry_msgs::msg::Quaternion quaternion_mult_(const geometry_msgs::msg::Quaternion& q, const geometry_msgs::msg::Quaternion& r);
std::vector<double> point_rotation_by_quaternion(const std::vector<double>& point, const std::vector<double>& q);
geometry_msgs::msg::Point point_rotation_by_quaternion(const geometry_msgs::msg::Point &point, const geometry_msgs::msg::Quaternion &q);

// Distance between waypoints
double get_wp_distance(const mavros_msgs::msg::Waypoint& wp1, const mavros_msgs::msg::Waypoint& wp2);

// Offset waypoints to a new reference location
std::vector<mavros_msgs::msg::Waypoint> offset_waypoints(const std::vector<mavros_msgs::msg::Waypoint>& waypoints, sensor_msgs::msg::NavSatFix old_ref, sensor_msgs::msg::NavSatFix new_ref);
void offset_waypoints(std::vector<double>& latitudes, std::vector<double>& longitudes, sensor_msgs::msg::NavSatFix old_ref, sensor_msgs::msg::NavSatFix new_ref);