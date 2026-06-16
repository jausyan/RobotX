#include "geo_.hpp"

LatLonAlt convert_xy_to_lat_long(double x, double y, double z,
    LatLonAlt zero, double zero_hdg)
{
    double norm = std::sqrt(x*x + y*y);
    double angle = std::atan2(-1*y, x);
    double rot_x = norm * std::cos(zero_hdg * M_PI / 180.0 + angle);
    double rot_y = norm * std::sin(zero_hdg * M_PI / 180.0 + angle + 3.141592);

    double lat = zero.lat + std::asin(rot_x / EARTH_RADIUS) * (180.0 / M_PI);
    double lon = zero.lon + std::asin(rot_y / EARTH_RADIUS) * (180.0 / M_PI);
    double alt = zero.alt + z;

    return {lat, lon, alt};
}

XYZ convert_lat_long_to_xy(double lat, double lon, double alt, LatLonAlt zero)
{
    double x = (lat - zero.lat) * (M_PI / 180.0) * EARTH_RADIUS;
    double y = (lon - zero.lon) * (M_PI / 180.0) * EARTH_RADIUS;
    return {x, y, alt - zero.alt};
}

mavros_msgs::msg::Waypoint create_waypoint(double lat, double lon, double alt,
    double acc_radius, double pass_radius, double yaw,
    bool is_current, bool autocontinue)
{
    mavros_msgs::msg::Waypoint wp;
    wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command = 16; // MAV_CMD_NAV_WAYPOINT
    wp.is_current = is_current;
    wp.autocontinue = autocontinue;
    wp.param1 = 0.0;
    wp.param2 = acc_radius;
    wp.param3 = pass_radius;
    wp.param4 = yaw;
    wp.x_lat = lat;
    wp.y_long = lon;
    wp.z_alt = alt;
    return wp;
}

mavros_msgs::msg::Waypoint create_waypoint_from_xyz(double x, double y, double z,
    LatLonAlt zero, double zero_hdg,
    double acc_radius, double pass_radius,
    double yaw, bool is_current, bool autocontinue)
{
    auto [lat, lon, alt] = convert_xy_to_lat_long(x, y, z, zero, zero_hdg);
    return create_waypoint(lat, lon, alt, acc_radius, pass_radius, yaw, is_current, autocontinue);
}

mavros_msgs::msg::GlobalPositionTarget create_globalpos_from_xyz(double x, double y, double rel_alt, LatLonAlt zero, double zero_hdg)
{
    auto [lat, lon, alt] = convert_xy_to_lat_long(x, y, 0.0, zero, zero_hdg);

    mavros_msgs::msg::GlobalPositionTarget gpt;
    gpt.header.frame_id = "map";
    gpt.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    gpt.type_mask = mavros_msgs::msg::GlobalPositionTarget::IGNORE_VX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW_RATE;
    gpt.latitude = lat;
    gpt.longitude = lon;
    gpt.altitude = rel_alt;

    return gpt;
}

mavros_msgs::msg::Waypoint get_takeoff_command(double alt, double pitch, double yaw,
    bool is_current, bool autocontinue)
{
    mavros_msgs::msg::Waypoint wp;
    wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command = 22; // MAV_CMD_NAV_TAKEOFF
    wp.is_current = is_current;
    wp.autocontinue = autocontinue;
    wp.param1 = pitch;
    wp.param4 = yaw;
    wp.z_alt = alt;
    return wp;
}

std::vector<double> quaternion_mult(const std::vector<double>& q, const std::vector<double>& r)
{
    return {
        r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3],
        r[0]*q[1] + r[1]*q[0] - r[2]*q[3] + r[3]*q[2],
        r[0]*q[2] + r[1]*q[3] + r[2]*q[0] - r[3]*q[1],
        r[0]*q[3] - r[1]*q[2] + r[2]*q[1] + r[3]*q[0]
    };
}

geometry_msgs::msg::Quaternion quaternion_mult_(const geometry_msgs::msg::Quaternion& q, const geometry_msgs::msg::Quaternion& r)
{
    geometry_msgs::msg::Quaternion result;
    result.w = r.w*q.w - r.x*q.x - r.y*q.y - r.z*q.z;
    result.x = r.w*q.x + r.x*q.w - r.y*q.z + r.z*q.y;
    result.y = r.w*q.y + r.x*q.z + r.y*q.w - r.z*q.x;
    result.z = r.w*q.z - r.x*q.y + r.y*q.x + r.z*q.w;
    return result;
}

std::vector<double> point_rotation_by_quaternion(const std::vector<double>& point, const std::vector<double>& q)
{
    std::vector<double> r = {0, point[0], point[1], point[2]};
    std::vector<double> q_conj = {q[0], -q[1], -q[2], -q[3]};
    auto step1 = quaternion_mult(q, r);
    auto step2 = quaternion_mult(step1, q_conj);
    return {step2[1], step2[2], step2[3]};
}

geometry_msgs::msg::Point point_rotation_by_quaternion(const geometry_msgs::msg::Point &point, const geometry_msgs::msg::Quaternion &q) {
    geometry_msgs::msg::Quaternion r;
    r.w = 0;
    r.x = point.x;
    r.y = point.y;
    r.z = point.z;

    geometry_msgs::msg::Quaternion q_conj;
    q_conj.w = q.w;
    q_conj.x = -q.x;
    q_conj.y = -q.y;
    q_conj.z = -q.z;

    geometry_msgs::msg::Quaternion step1 = quaternion_mult_(q, r);
    geometry_msgs::msg::Quaternion step2 = quaternion_mult_(step1, q_conj);

    geometry_msgs::msg::Point rotated_point;
    rotated_point.x = step2.x;
    rotated_point.y = step2.y;
    rotated_point.z = step2.z;

    return rotated_point;
}

double get_wp_distance(const mavros_msgs::msg::Waypoint& wp1, const mavros_msgs::msg::Waypoint& wp2)
{
    double lat1 = wp1.x_lat;
    double lon1 = wp1.y_long;
    double lat2 = wp2.x_lat;
    double lon2 = wp2.y_long;

    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;

    double a = std::pow(std::sin(dlat / 2), 2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::pow(std::sin(dlon / 2), 2);

    double c = 2 * std::asin(std::sqrt(a));
    return EARTH_RADIUS * c;
}

std::vector<mavros_msgs::msg::Waypoint> offset_waypoints(const std::vector<mavros_msgs::msg::Waypoint>& waypoints, sensor_msgs::msg::NavSatFix old_ref, sensor_msgs::msg::NavSatFix new_ref)
{
    std::vector<mavros_msgs::msg::Waypoint> new_waypoints;
    double dlat = new_ref.latitude - old_ref.latitude;
    double dlon = new_ref.longitude - old_ref.longitude;

    for (const auto& wp : waypoints) {
        mavros_msgs::msg::Waypoint new_wp = create_waypoint(wp.x_lat + dlat, wp.y_long + dlon, wp.z_alt);
        new_waypoints.push_back(new_wp);
    }

    return new_waypoints;
}

#include <iostream>
#include <iomanip>

void offset_waypoints(std::vector<double>& latitudes, std::vector<double>& longitudes, sensor_msgs::msg::NavSatFix old_ref, sensor_msgs::msg::NavSatFix new_ref)
{
    double dlat = new_ref.latitude - old_ref.latitude;
    double dlon = new_ref.longitude - old_ref.longitude;

    for (int i = 0; i < latitudes.size(); ++i) {
        std::cout<<"Real " << i << ": (" << latitudes[i] << ", " << longitudes[i] << ") dlat, dlon: ("<<  dlat << ", " << dlon << ")\n";
        latitudes[i] = latitudes[i] + dlat;
        longitudes[i] = longitudes[i] + dlon;

        std::cout<<"Offset waypoint " << i << ": (" << std::fixed << std::setprecision(7) << latitudes[i] << ", " << std::fixed << std::setprecision(7)  << longitudes[i] << ")\n";
    }
}