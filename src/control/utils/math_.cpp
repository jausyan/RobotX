#include "math_.hpp"

double getHeading(const geometry_msgs::msg::Quaternion& q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return atan2(siny_cosp, cosy_cosp);
}

double getPitch(const geometry_msgs::msg::Quaternion& q){
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    return 2*atan2(sinp, cosp) - M_PI/2;
}

double getRoll(const geometry_msgs::msg::Quaternion& q){
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    return atan2(sinr_cosp, cosr_cosp);
}

void quatToEuler(geometry_msgs::msg::Point& p, const geometry_msgs::msg::Quaternion& q){
    p.x = getRoll(q); p.y = getPitch(q); p.z = getHeading(q);
}

void setHeading(geometry_msgs::msg::Quaternion& q, const double h_cmd){
    q.x = 0;
    q.y = 0;
    q.z = sin(h_cmd/2);
    q.w = cos(h_cmd/2);
}

void setPosition(geometry_msgs::msg::Point& p, const double x,const double y,const double z){
    p.x = x;
    p.y = y;
    p.z = z;
}

void setPosition(geometry_msgs::msg::PoseStamped& p, const double x,const double y,const double z){
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
}

geometry_msgs::msg::Point reflect(const geometry_msgs::msg::Point& p) {
    geometry_msgs::msg::Point p_ref;
    p_ref.x = p.y;
    p_ref.y = p.x;
    return p_ref;
}

geometry_msgs::msg::Point rotatePoint(const geometry_msgs::msg::Point&p, const float theta){
    geometry_msgs::msg::Point p_rot;
    p_rot.x=p.x*cos(theta)-p.y*sin(theta);
    p_rot.y=p.x*sin(theta)+p.y*cos(theta);
    return p_rot;
}

geometry_msgs::msg::Point rotatePoint(const float x, const float y, const float theta){
    geometry_msgs::msg::Point p_rot;
    p_rot.x=x*cos(theta)-y*sin(theta);
    p_rot.y=x*sin(theta)+y*cos(theta);
    return p_rot;
}

geometry_msgs::msg::Point rotatePoint(const geometry_msgs::msg::PoseStamped& p, const float theta){
    geometry_msgs::msg::Point p_rot;
    p_rot.x= p.pose.position.x*cos(theta)-p.pose.position.y*sin(theta);
    p_rot.y= p.pose.position.x*sin(theta)+p.pose.position.y*cos(theta);
    return p_rot;
}

geometry_msgs::msg::Point rotatePoint(const double x , const double y, const float theta){
    geometry_msgs::msg::Point p_rot;
    p_rot.x= x*cos(theta)-y*sin(theta);
    p_rot.y= x*sin(theta)+y*cos(theta);
    return p_rot;
}

geometry_msgs::msg::Point operator-(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& q) {
    geometry_msgs::msg::Point p_off;
    p_off.x = p.x-q.x;
    p_off.y = p.y-q.y;
    p_off.z = p.z-q.z;
    return p_off;
}

geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& q) {
    geometry_msgs::msg::Point p_off;
    p_off.x = p.x+q.x;
    p_off.y = p.y+q.y;
    p_off.z = p.z+q.z;
    return p_off;
}

geometry_msgs::msg::Point operator-(const geometry_msgs::msg::Point& p) {
    geometry_msgs::msg::Point p_off;
    p_off.x = -p.x;
    p_off.y = -p.y;
    p_off.z = -p.z;
    return p_off;
}

int sgn(double val){
    return ((val > 0) - (val < 0));
}

double clamp(double max, double val){
    if (abs(val)>max) return sgn(val)*max;
    return val;

}

int sgn(const float val){
    return ((val>0) - (val<0));
}

bool isNear(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& p_tgt,const double eps){
    return sqrt((p_tgt.x-p.x)*(p_tgt.x-p.x)+(p_tgt.y-p.y)*(p_tgt.y-p.y))<eps;
}

bool isNear(const geometry_msgs::msg::Point& p,const double x1,const double y1,const double eps){
    return sqrt((x1-p.x)*(x1-p.x)+(y1-p.y)*(y1-p.y))<eps;
}

bool isNear(const geometry_msgs::msg::Point& p, float& x1, float& y1, float& eps){
    return sqrt((x1-p.x)*(x1-p.x)+(y1-p.y)*(y1-p.y))<eps;
}

double dist(const geometry_msgs::msg::Point& p,const double x1,const double y1){
    return sqrt((x1-p.x)*(x1-p.x)+(y1-p.y)*(y1-p.y));
}

double dist(const geometry_msgs::msg::Point& p, float& x1, float& y1){
    return sqrt((x1-p.x)*(x1-p.x)+(y1-p.y)*(y1-p.y));
}

double dist(const geometry_msgs::msg::Point& p,const geometry_msgs::msg::Point& p_tgt){
    return sqrt((p_tgt.x-p.x)*(p_tgt.x-p.x)+(p_tgt.y-p.y)*(p_tgt.y-p.y));
}

double dist(const double x1,const double y1, const double x2,const double y2){
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

bool isNear(const geometry_msgs::msg::Vector3& p,const double x1,const double y1,const double eps){
    return sqrt((x1-p.x)*(x1-p.x)+(y1-p.y)*(y1-p.y))<eps;
}

bool isNear(const double x1,const double y1,const double x2,const double y2,const double eps){
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))<eps;
}

float limit(float max, float tgt){
    // if (abs(tgt)>max) return sgn(tgt)*max;
    if(tgt > max) {return max;}
    if(tgt < -1*max) {return -1*max;}
    return tgt;
}

float limitDelta(float maxDelta, float tgt, float last){
    return last+limit(maxDelta,tgt-last);
}

// geometry_msgs::msg::TwistStamped operator*(geometry_msgs::msg::Point& tgt,float factor){
//     geometry_msgs::msg::TwistStamped prd;
//     prd.twist.linear.x = tgt.x * factor;
//     prd.twist.linear.y = tgt.y * factor;
//     prd.twist.linear.z = tgt.z * factor;
//     return prd;
// }

geometry_msgs::msg::Point operator*(geometry_msgs::msg::Point& tgt,float factor){
    geometry_msgs::msg::Point prd;
    prd.x = tgt.x * factor;
    prd.y = tgt.y * factor;
    prd.z = tgt.z * factor;
    return prd;
}

geometry_msgs::msg::TwistStamped cast(geometry_msgs::msg::Point& tgt){
    geometry_msgs::msg::TwistStamped prd;
    prd.twist.linear.x = tgt.x;
    prd.twist.linear.y = tgt.y;
    prd.twist.linear.z = tgt.z;
    return prd;
}


geometry_msgs::msg::TwistStamped limitDelta(double maxDelta, geometry_msgs::msg::TwistStamped& tgt, geometry_msgs::msg::TwistStamped& last){
    geometry_msgs::msg::TwistStamped ret;
    ret.twist.linear.x = limitDelta(maxDelta,tgt.twist.linear.x,last.twist.linear.x);
    ret.twist.linear.y = limitDelta(maxDelta,tgt.twist.linear.y,last.twist.linear.y);
    ret.twist.linear.z = limitDelta(maxDelta,tgt.twist.linear.z,last.twist.linear.z);
    return ret;
}

geometry_msgs::msg::TwistStamped limit(double max, geometry_msgs::msg::TwistStamped& tgt){
    geometry_msgs::msg::TwistStamped ret;
    ret.twist.linear.x = limit(max,tgt.twist.linear.x);
    ret.twist.linear.y = limit(max,tgt.twist.linear.y);
    ret.twist.linear.z = limit(max,tgt.twist.linear.z);
    return ret;
}

geometry_msgs::msg::TwistStamped zero(geometry_msgs::msg::TwistStamped& tgt){
    geometry_msgs::msg::TwistStamped ret;
    ret.twist.linear.x = 0;
    ret.twist.linear.y = 0;
    ret.twist.linear.z = 0;
    return ret;
}

double latLonToMeter(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6378.137; // Radius of Earth in KM
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = R * c;
    return d * 1000; // meters
}

geometry_msgs::msg::PoseStamped centroid(std::vector<geometry_msgs::msg::PoseStamped>& arr) {
    int n = arr.size();
    geometry_msgs::msg::PoseStamped centroid;
    if(n <= 0){return centroid;}
    for(int i = 0; i < n; i++)
    {
        centroid.pose.position.x += arr[i].pose.position.x;
        centroid.pose.position.y += arr[i].pose.position.y;
        centroid.pose.position.z += arr[i].pose.position.z;
    }
    centroid.pose.position.x /= n;
    centroid.pose.position.y /= n;
    centroid.pose.position.z /= n;
    return centroid;
}

float dataRange(std::vector<float> data) {
    float min = 9999.9999;
    float max = -9999.9999;
    for (int i = 0; i < data.size(); i++)
    {
        if(data[i] < min) {min = data[i];}
        else if(data[i] > max) {max = data[i];}
    }
    return max - min;
}