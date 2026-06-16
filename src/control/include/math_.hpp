#ifndef MATH__HPP
#define MATH__HPP

#include <iostream>
#include <string>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define M_PI 3.14159265358979323846
#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

double getHeading(const geometry_msgs::msg::Quaternion& q);

double getPitch(const geometry_msgs::msg::Quaternion& q);

double getRoll(const geometry_msgs::msg::Quaternion& q);

void quatToEuler(geometry_msgs::msg::Point& p, const geometry_msgs::msg::Quaternion& q);

void setHeading(geometry_msgs::msg::Quaternion& q, const double h_cmd);

void setPosition(geometry_msgs::msg::Point& p, const double x,const double y,const double z);

void setPosition(geometry_msgs::msg::PoseStamped& p, const double x,const double y,const double z);

geometry_msgs::msg::Point reflect(const geometry_msgs::msg::Point& p);

geometry_msgs::msg::Point rotatePoint(const geometry_msgs::msg::Point&p, const float theta);

geometry_msgs::msg::Point rotatePoint(const float x, const float y, const float theta);

geometry_msgs::msg::Point rotatePoint(const geometry_msgs::msg::PoseStamped& p, const float theta);

geometry_msgs::msg::Point rotatePoint(const double x , const double y, const float theta);

geometry_msgs::msg::Point operator-(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& q);

geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& q);

geometry_msgs::msg::Point operator-(const geometry_msgs::msg::Point& p);

int sgn(double val);

double clamp(double max, double val);

int sgn(const float val);

bool isNear(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& p_tgt,const double eps);

bool isNear(const geometry_msgs::msg::Point& p,const double x1,const double y1,const double eps);

bool isNear(const geometry_msgs::msg::Point& p, float& x1, float& y1, float& eps);

double dist(const geometry_msgs::msg::Point& p,const double x1,const double y1);

double dist(const geometry_msgs::msg::Point& p, float& x1, float& y1);

double dist(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& p_tgt);

double dist(const double x1,const double y1, const double x2,const double y2);

bool isNear(const geometry_msgs::msg::Vector3& p,const double x1,const double y1,const double eps);

bool isNear(const double x1,const double y1,const double x2,const double y2,const double eps);

float limit(float max, float tgt);

float limitDelta(float maxDelta, float tgt, float last);

// geometry_msgs::msg::TwistStamped operator*(geometry_msgs::msg::Point& tgt,float factor){
//     geometry_msgs::msg::TwistStamped prd;
//     prd.twist.linear.x = tgt.x * factor;
//     prd.twist.linear.y = tgt.y * factor;
//     prd.twist.linear.z = tgt.z * factor;
//     return prd;
// }

geometry_msgs::msg::Point operator*(geometry_msgs::msg::Point& tgt,float factor);

geometry_msgs::msg::TwistStamped cast(geometry_msgs::msg::Point& tgt);


geometry_msgs::msg::TwistStamped limitDelta(double maxDelta, geometry_msgs::msg::TwistStamped& tgt, geometry_msgs::msg::TwistStamped& last);

geometry_msgs::msg::TwistStamped limit(double max, geometry_msgs::msg::TwistStamped& tgt);

geometry_msgs::msg::TwistStamped zero(geometry_msgs::msg::TwistStamped& tgt);

double latLonToMeter(double lat1, double lon1, double lat2, double lon2);

geometry_msgs::msg::PoseStamped centroid(std::vector<geometry_msgs::msg::PoseStamped>& arr);

float dataRange(std::vector<float> data);

#endif // MATH__HPP