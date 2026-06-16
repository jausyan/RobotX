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

#include "drone_controller_.hpp"

DroneController::DroneController() : Node("drone_controller") {
    this->local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("rian/setpoint_position/local", 10);
    this->local_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("rian/setpoint_velocity/cmd_vel", 10);
    this->local_vel_pub_unstamped = this->create_publisher<geometry_msgs::msg::Twist>("rian/setpoint_velocity/cmd_vel_unstamped", 10);
    this->open_cam_pub = this->create_publisher<std_msgs::msg::Bool>("open_camera1", 10);
    this->open_cam2_pub = this->create_publisher<std_msgs::msg::Bool>("open_camera2", 10);
    this->setpoint_raw_global_pub = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>("rian/setpoint_raw/global", 10);
    this->setpoint_raw_local_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>("rian/setpoint_raw/local", 10);

    this->takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>("rian/cmd/takeoff");
    this->land_client = this->create_client<mavros_msgs::srv::CommandTOL>("rian/cmd/land");
    this->arming_client = this->create_client<mavros_msgs::srv::CommandBool>("rian/cmd/arming");
    this->set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("rian/set_mode");
    this->servo_client = this->create_client<mavros_msgs::srv::CommandLong>("rian/cmd/command");
    this->close_cam_client = this->create_client<std_srvs::srv::Trigger>("/close_cam");
    this->open_cam_client = this->create_client<std_srvs::srv::Trigger>("/open_cam");
    this->set_param_client = this->create_client<mavros_msgs::srv::ParamSetV2>("rian/param/set");
    this->push_mission_client = this->create_client<mavros_msgs::srv::WaypointPush>("rian/mission/push");
    this->clear_mission_client = this->create_client<mavros_msgs::srv::WaypointClear>("rian/mission/clear");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    this->state_sub = this->create_subscription<mavros_msgs::msg::State>("rian/state", qos, std::bind(&DroneController::state_cb, this, std::placeholders::_1));
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("rian/local_position/pose", qos, std::bind(&DroneController::pose_cb, this, std::placeholders::_1));
    this->gate_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vision/front", qos, std::bind(&DroneController::gate_pose_cb, this, std::placeholders::_1));
    this->artag_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vision/down/artag", qos, std::bind(&DroneController::artag_pose_cb, this, std::placeholders::_1));
    this->payload_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vision/down/payload", qos, std::bind(&DroneController::payload_pose_cb, this, std::placeholders::_1));
    this->ember_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vision/down/ember", qos, std::bind(&DroneController::ember_pose_cb, this, std::placeholders::_1));
    this->bunder_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vision/down/outdoor", qos, std::bind(&DroneController::bunder_pose_cb, this, std::placeholders::_1));
    this->vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("rian/local_position/velocity_body", qos, std::bind(&DroneController::vel_cb, this, std::placeholders::_1));
    this->compass_hdg_sub = this->create_subscription<std_msgs::msg::Float64>("rian/global_position/compass_hdg", qos, std::bind(&DroneController::compass_hdg_cb, this, std::placeholders::_1));
    this->gps_state_sub = this->create_subscription<mavros_msgs::msg::GPSRAW>("rian/gpsstatus/gps1/raw", qos, std::bind(&DroneController::gps_state_cb, this, std::placeholders::_1));
    this->accel_sub = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>("rian/local_position/accel", qos, std::bind(&DroneController::accel_cb, this, std::placeholders::_1));
    this->gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("rian/global_position/global", qos, std::bind(&DroneController::gps_cb, this, std::placeholders::_1));
    this->alt_sub = this->create_subscription<sensor_msgs::msg::Range>("rian/rangefinder/rangefinder", qos, std::bind(&DroneController::alt_cb, this, std::placeholders::_1));
    this->rel_alt_sub = this->create_subscription<std_msgs::msg::Float64>("rian/global_position/rel_alt", qos, std::bind(&DroneController::rel_alt_cb, this, std::placeholders::_1));
    this->wp_reached_sub = this->create_subscription<mavros_msgs::msg::WaypointReached>("rian/mission/reached", qos, std::bind(&DroneController::wp_reached_cb, this, std::placeholders::_1));
    this->laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", qos, std::bind(&DroneController::laser_cb, this, std::placeholders::_1));
}

// Get fucntions to retrieve current state and pose
mavros_msgs::msg::State DroneController::getCurrentState()                        {return this->curr_state;}
mavros_msgs::msg::GPSRAW DroneController::getCurrentGPSState()                    {return this->gps_state;}
geometry_msgs::msg::PoseStamped DroneController::getCurrentLocalPose()            {return this->curr_pose;}
geometry_msgs::msg::PoseStamped DroneController::getCurrentGatePose()             {return this->gate_pose;}
geometry_msgs::msg::PoseStamped DroneController::getCurrentPoseArTag()            {return this->artag_pose;}
geometry_msgs::msg::PoseStamped DroneController::getCurrentPosePayload()          {return this->payload_pose;}
geometry_msgs::msg::PoseStamped DroneController::getCurrentPoseEmber()            {return this->ember_pose;}
geometry_msgs::msg::PoseStamped DroneController::getCurrentPoseBunder()           {return this->bunder_pose;}
geometry_msgs::msg::PoseStamped DroneController::getLastNonZeroPoseArTag()        {return this->nonzero_last_artag_pose;}
geometry_msgs::msg::PoseStamped DroneController::getLastNonZeroPosePayload()      {return this->nonzero_last_payload_pose;}
geometry_msgs::msg::PoseStamped DroneController::getLastNonZeroPoseEmber()        {return this->nonzero_last_ember_pose;}
geometry_msgs::msg::PoseStamped DroneController::getLastNonZeroPoseBunder()       {return this->nonzero_last_bunder_pose;}
geometry_msgs::msg::PoseStamped DroneController::getDroneNonZeroPoseArTag()       {return this->nonzero_drone_artag_pose;}
geometry_msgs::msg::PoseStamped DroneController::getDroneNonZeroPosePayload()     {return this->nonzero_drone_payload_pose;}
geometry_msgs::msg::PoseStamped DroneController::getDroneNonZeroPoseEmber()       {return this->nonzero_drone_ember_pose;}
geometry_msgs::msg::PoseStamped DroneController::getDroneNonZeroPoseBunder()      {return this->nonzero_drone_bunder_pose;}
geometry_msgs::msg::Point DroneController::getCurrentEuler()                      {return this->curr_euler;}
geometry_msgs::msg::TwistStamped DroneController::getCurrentVelocity()            {return this->curr_vel;}
geometry_msgs::msg::PoseStamped DroneController::getLastArTagPose()               {return this->last_artag_pose;}
geometry_msgs::msg::PoseStamped DroneController::getLastPayloadPose()             {return this->last_payload_pose;}
geometry_msgs::msg::PoseStamped DroneController::getLastEmberPose()               {return this->last_ember_pose;}
geometry_msgs::msg::PoseStamped DroneController::getLastBunderPose()              {return this->last_bunder_pose;}
geometry_msgs::msg::AccelWithCovarianceStamped DroneController::getCurrentAccel() {return this->curr_accel;}
sensor_msgs::msg::NavSatFix DroneController::getCurrentGPSPosition()              {return this->curr_gps_position;}
geometry_msgs::msg::PoseStamped DroneController::getInitDronePose()               {return this->init_drone_pose;}
std_msgs::msg::Float64 DroneController::getCurrentCompassHdg()                    {return this->compass_hdg;}
std_msgs::msg::Float64 DroneController::getCurrentRelAlt()                        {return this->curr_rel_alt;}
sensor_msgs::msg::Range DroneController::getCurrentRangefinder()                  {return this->curr_rangefinder;}
sensor_msgs::msg::LaserScan DroneController::getCurrentLaserScan()                {return this->curr_laserscan;}

float DroneController::getObstacleDistance(float angle, float angle_tolerance) {
    float min_distance = std::numeric_limits<float>::infinity();
    float min_angle = angle - angle_tolerance/2;
    float angle_inc = 2*M_PI / this->curr_laserscan.ranges.size();
    for (int i = (int)((angle - angle_tolerance/2) / angle_inc); i < (int)(angle_tolerance / angle_inc); ++i) {
        // this is very prone to noise, consider thresholding variance to detect invalid readings
        if (this->curr_laserscan.ranges[this->curr_laserscan.ranges.size() % i] < min_distance) {
            min_distance = this->curr_laserscan.ranges[i];
            min_angle = i*angle_inc;
        }
    }

    // Project the distance by drone's pitch and roll
    // float pitch = getHeading(this->curr_pose.pose.orientation);
    // float roll = getHeading(this->curr_pose.pose.orientation);
    // min_distance = min_distance * std::cos(pitch) * std::cos(roll);// * std::cos(angle - min_angle);

    return min_distance;
}

int DroneController::current_wp_() {return wp_reached;}
int DroneController::reset_wp_()   {wp_reached = 0; return 0;}

// Set functions to update current state and pose
void DroneController::setLastArTagPose(const geometry_msgs::msg::PoseStamped &pose)          {this->last_artag_pose = pose;}
void DroneController::setLastPayloadPose(const geometry_msgs::msg::PoseStamped &pose)        {this->last_payload_pose = pose;}
void DroneController::setLastEmberPose(const geometry_msgs::msg::PoseStamped &pose)          {this->last_ember_pose = pose;}
void DroneController::setLastBunderPose(const geometry_msgs::msg::PoseStamped &pose)         {this->last_bunder_pose = pose;}
void DroneController::setInitDronePose(const geometry_msgs::msg::PoseStamped &pose)          {this->init_drone_pose = pose;}
void DroneController::resetNonzeroArTagPose()
{
    this->nonzero_last_artag_pose.pose.position.x = 0.0;
    this->nonzero_last_artag_pose.pose.position.y = 0.0;
    this->nonzero_last_artag_pose.pose.position.z = 0.0;

    this->nonzero_drone_artag_pose.pose.position.x = 0.0;
    this->nonzero_drone_artag_pose.pose.position.y = 0.0;
    this->nonzero_drone_artag_pose.pose.position.z = 0.0;
}
void DroneController::resetNonzeroPayloadPose()
{
    this->nonzero_last_payload_pose.pose.position.x = 0.0;
    this->nonzero_last_payload_pose.pose.position.y = 0.0;
    this->nonzero_last_payload_pose.pose.position.z = 0.0;

    this->nonzero_drone_payload_pose.pose.position.x = 0.0;
    this->nonzero_drone_payload_pose.pose.position.y = 0.0;
    this->nonzero_drone_payload_pose.pose.position.z = 0.0;
}
void DroneController::resetNonzeroEmberPose()
{
    this->nonzero_last_ember_pose.pose.position.x = 0.0;
    this->nonzero_last_ember_pose.pose.position.y = 0.0;
    this->nonzero_last_ember_pose.pose.position.z = 0.0;

    this->nonzero_drone_ember_pose.pose.position.x = 0.0;
    this->nonzero_drone_ember_pose.pose.position.y = 0.0;
    this->nonzero_drone_ember_pose.pose.position.z = 0.0;
}
void DroneController::resetNonzeroBunderPose()
{
    this->nonzero_last_bunder_pose.pose.position.x = 0.0;
    this->nonzero_last_bunder_pose.pose.position.y = 0.0;
    this->nonzero_last_bunder_pose.pose.position.z = 0.0;

    this->nonzero_drone_bunder_pose.pose.position.x = 0.0;
    this->nonzero_drone_bunder_pose.pose.position.y = 0.0;
    this->nonzero_drone_bunder_pose.pose.position.z = 0.0;
}

// Functions to wait for services to be available
void DroneController::waitForTakeoffService()      {waitForService(this->takeoff_client);}
void DroneController::waitForLandService()         {waitForService(this->land_client);}
void DroneController::waitForArmingService()       {waitForService(this->arming_client);}
void DroneController::waitForSetModeService()      {waitForService(this->set_mode_client);}
void DroneController::waitForServoService()        {waitForService(this->servo_client);}
void DroneController::waitForCloseCamService()     {waitForService(this->close_cam_client);}
void DroneController::waitForOpenCamService()      {waitForService(this->open_cam_client);}
void DroneController::waitForSetParamService()     {waitForService(this->set_param_client);}
void DroneController::waitForPushMissionService()  {waitForService(this->push_mission_client);}
void DroneController::waitForClearMissionService() {waitForService(this->clear_mission_client);}

// Functions to check if services are ready
bool DroneController::isTakeoffServiceReady()      {return this->takeoff_client->service_is_ready();}
bool DroneController::isLandServiceReady()         {return this->land_client->service_is_ready();}
bool DroneController::isArmingServiceReady()       {return this->arming_client->service_is_ready();}
bool DroneController::isSetModeServiceReady()      {return this->set_mode_client->service_is_ready();}
bool DroneController::isServoServiceReady()        {return this->servo_client->service_is_ready();}
bool DroneController::isCloseCamServiceReady()     {return this->close_cam_client->service_is_ready();}
bool DroneController::isOpenCamServiceReady()      {return this->open_cam_client->service_is_ready();}
bool DroneController::isSetParamServiceReady()     {return this->set_param_client->service_is_ready();}
bool DroneController::isPushMissionServiceReady()  {return this->push_mission_client->service_is_ready();}
bool DroneController::isClearMissionServiceReady() {return this->clear_mission_client->service_is_ready();}

// Functions to call services, returns a future that can be waited on
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture DroneController::takeoff_(const std::shared_ptr<mavros_msgs::srv::CommandTOL::Request>& request)             {waitForTakeoffService(); return this->takeoff_client->async_send_request(request);}
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture DroneController::land_(const std::shared_ptr<mavros_msgs::srv::CommandTOL::Request>& request)                {waitForLandService(); return this->land_client->async_send_request(request);}
rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture DroneController::arm_(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request>& request)               {waitForArmingService(); return this->arming_client->async_send_request(request);}
rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture DroneController::setMode_(const std::shared_ptr<mavros_msgs::srv::SetMode::Request>& request)                   {waitForSetModeService(); return this->set_mode_client->async_send_request(request);}
rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture DroneController::servo_(const std::shared_ptr<mavros_msgs::srv::CommandLong::Request>& request)             {waitForServoService(); return this->servo_client->async_send_request(request);}
rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture DroneController::closeCamera_(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request)                     {waitForCloseCamService(); return this->close_cam_client->async_send_request(request);}
rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture DroneController::openCamera_(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request)                      {waitForOpenCamService(); return this->open_cam_client->async_send_request(request);}
rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedFuture DroneController::setParam_(const std::shared_ptr<mavros_msgs::srv::ParamSetV2::Request>& request)            {waitForSetParamService(); return this->set_param_client->async_send_request(request);}
rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture DroneController::pushMission_(const std::shared_ptr<mavros_msgs::srv::WaypointPush::Request>& request)     {waitForSetParamService(); return this->push_mission_client->async_send_request(request);}
rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedFuture DroneController::clearMission_(const std::shared_ptr<mavros_msgs::srv::WaypointClear::Request>& request)  {waitForSetParamService(); return this->clear_mission_client->async_send_request(request);}

// Functions to publish messages
void DroneController::publishLocalPosition(const geometry_msgs::msg::PoseStamped &pose) {this->local_pos_pub->publish(pose);}
void DroneController::publishLocalVelocity(const geometry_msgs::msg::TwistStamped &vel) {this->local_vel_pub->publish(vel);}
void DroneController::publishLocalVelocityUnstamped(const geometry_msgs::msg::Twist &vel) {this->local_vel_pub_unstamped->publish(vel);}
void DroneController::publishOpenCamera(const std_msgs::msg::Bool &open) {this->open_cam_pub->publish(open);}
void DroneController::publishOpenCamera2(const std_msgs::msg::Bool &open) {this->open_cam2_pub->publish(open);}
void DroneController::publishSetpointRawGlobal(const mavros_msgs::msg::GlobalPositionTarget &setpoint) {this->setpoint_raw_global_pub->publish(setpoint);}
void DroneController::publishSetpointRawLocal(const mavros_msgs::msg::PositionTarget &setpoint)        {this->setpoint_raw_local_pub->publish(setpoint);}

// Callback functions to receive messages
void DroneController::state_cb(const mavros_msgs::msg::State::ConstSharedPtr msg)                   {curr_state = *msg;}
void DroneController::gps_state_cb(const mavros_msgs::msg::GPSRAW::SharedPtr msg)                   {gps_state = *msg;}
void DroneController::pose_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)            {curr_pose = *msg; quatToEuler(curr_euler, curr_pose.pose.orientation);}
void DroneController::gate_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)            {gate_pose = *msg;}
void DroneController::artag_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)           {artag_pose = *msg; if(msg->pose.position.x != 0.0 && msg->pose.position.y != 0.0 && msg->pose.position.z != 0.0) {nonzero_last_artag_pose = *msg; nonzero_drone_artag_pose = this->curr_pose;}}
void DroneController::payload_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)         {payload_pose = *msg; if(msg->pose.position.x != 0.0 && msg->pose.position.y != 0.0 && msg->pose.position.z != 0.0) {nonzero_last_payload_pose = *msg; nonzero_drone_payload_pose = this->curr_pose;}}
void DroneController::ember_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)           {ember_pose = *msg; if(msg->pose.position.x != 0.0 && msg->pose.position.y != 0.0 && msg->pose.position.z != 0.0) {nonzero_last_ember_pose = *msg; nonzero_drone_ember_pose = this->curr_pose;}}
void DroneController::bunder_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)          {bunder_pose = *msg; if(msg->pose.position.x != 0.0 && msg->pose.position.y != 0.0 && msg->pose.position.z != 0.0) {nonzero_last_bunder_pose = *msg; nonzero_drone_bunder_pose = this->curr_pose;}}
void DroneController::vel_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)            {curr_vel = *msg;}
void DroneController::accel_cb(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg) {curr_accel = *msg;}
void DroneController::gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)                      {curr_gps_position = *msg;}
void DroneController::compass_hdg_cb(const std_msgs::msg::Float64::SharedPtr msg)                   {compass_hdg = *msg;}
void DroneController::rel_alt_cb(const std_msgs::msg::Float64::SharedPtr msg)                       {curr_rel_alt = *msg;}
void DroneController::alt_cb(const sensor_msgs::msg::Range::SharedPtr msg)                          {curr_rangefinder = *msg;}
void DroneController::wp_reached_cb(const mavros_msgs::msg::WaypointReached::SharedPtr msg)         {wp_reached = msg->wp_seq;}
void DroneController::laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)                    {curr_laserscan = *msg;}

