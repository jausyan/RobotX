#include "vision_geo_.hpp"

// ── Constructor ────────────────────────────────────────────────────────────────

VisionGeoNode::VisionGeoNode()
: Node("vision_geo_node"),
  fps_window_start_(std::chrono::steady_clock::now())
{
  // camera input
  input_mode_        = declare_parameter<std::string>("input_mode", "device");
  input_source_      = declare_parameter<std::string>("input_source", "/dev/video0");
  use_camera_info_   = declare_parameter<bool>("use_camera_info", false);
  camera_info_topic_ = declare_parameter<std::string>(
    "camera_info_topic", "/iris_with_camera_2/camera/camera_info");

  // YOLO
  model_path_      = declare_parameter<std::string>("model_path", "best.onnx");
  input_width_     = declare_parameter<int>("input_width", 320);
  input_height_    = declare_parameter<int>("input_height", 320);
  conf_threshold_  = declare_parameter<double>("conf_threshold", 0.4);
  nms_threshold_   = declare_parameter<double>("nms_threshold", 0.45);
  class_names_     = declare_parameter<std::vector<std::string>>("class_names",
    std::vector<std::string>{"object"});

  // MAVROS
  gps_topic_       = declare_parameter<std::string>("gps_topic",
    "/mavros/global_position/global");
  pose_topic_      = declare_parameter<std::string>("pose_topic",
    "/mavros/local_position/pose");
  rel_alt_topic_   = declare_parameter<std::string>("rel_alt_topic",
    "/mavros/global_position/rel_alt");
  use_fixed_pose_  = declare_parameter<bool>("use_fixed_pose", false);
  fixed_lat_       = declare_parameter<double>("fixed_lat", 0.0);
  fixed_lon_       = declare_parameter<double>("fixed_lon", 0.0);
  fixed_altitude_m_ = declare_parameter<double>("fixed_altitude_m", 15.0);
  max_pose_age_ms_ = declare_parameter<int>("max_pose_age_ms", 500);

  // camera mount rotation (camera frame → body FLU frame)
  std::vector<double> rpy = declare_parameter<std::vector<double>>(
    "cam_to_body_rpy", std::vector<double>{0.0, 0.0, 0.0});

  // output
  output_topic_    = declare_parameter<std::string>("output_topic",
    "/vision_geo/detections");
  marker_topic_    = declare_parameter<std::string>("marker_topic",
    "/vision_geo/markers");
  marker_frame_id_ = declare_parameter<std::string>("marker_frame_id", "map");
  marker_lifetime_s_     = declare_parameter<double>("marker_lifetime_s", 0.0);
  marker_size_           = declare_parameter<double>("marker_size", 0.5);
  map_merge_radius_m_    = declare_parameter<double>("map_merge_radius_m", 2.0);
  min_obs_to_commit_     = declare_parameter<int>("min_obs_to_commit", 5);
  track_exit_frames_     = declare_parameter<int>("track_exit_frames", 5);
  pixel_match_radius_px_ = declare_parameter<double>("pixel_match_radius_px", 100.0);

  // display
  show_window_ = declare_parameter<bool>("show_window", true);
  window_name_ = declare_parameter<std::string>("window_name", "vision_geo");

  // GStreamer stream (same params as vision_hailo)
  enable_stream_       = declare_parameter<bool>("enable_stream", false);
  stream_host_         = declare_parameter<std::string>("stream_host", "192.168.0.127");
  stream_port_         = declare_parameter<int>("stream_port", 5000);
  stream_width_        = declare_parameter<int>("stream_width", 640);
  stream_height_       = declare_parameter<int>("stream_height", 480);
  stream_fps_          = declare_parameter<int>("stream_fps", 30);
  stream_bitrate_kbps_ = declare_parameter<int>("stream_bitrate_kbps", 500);

  // mission order (read by main() in src/vision_geo.cpp) + per-class topics
  declare_parameter<std::string>("order_topic", "/mission/order");
  declare_parameter<bool>("wait_for_order", true);
  declare_parameter<double>("order_timeout", 3600.0);
  object_size_m_       = declare_parameter<std::vector<double>>("object_size_m",
    std::vector<double>{0.5});
  target_topic_prefix_ = declare_parameter<std::string>("target_topic_prefix",
    "/vision_geo/target/");
  map_topic_prefix_    = declare_parameter<std::string>("map_topic_prefix",
    "/vision_geo/map/");
  pose_frame_id_       = declare_parameter<std::string>("pose_frame_id", "camera");
  if (object_size_m_.empty()) object_size_m_.push_back(0.5);
  // classes that get GPS projection + map (Task 1); the rest only get a target pose
  const auto geo_classes = declare_parameter<std::vector<std::string>>("geo_classes", class_names_);
  for (const auto & name : class_names_) {
    is_geo_class_.push_back(
      std::find(geo_classes.begin(), geo_classes.end(), name) != geo_classes.end());
  }
  // buoy light state (flashing vs solid)
  state_window_s_  = declare_parameter<double>("state_window_s", 6.0);
  flash_gap_min_s_ = declare_parameter<double>("flash_gap_min_s", 0.6);
  flash_gap_max_s_ = declare_parameter<double>("flash_gap_max_s", 1.6);
  solid_min_s_     = declare_parameter<double>("solid_min_s", 3.0);

  // ── camera intrinsics ──────────────────────────────────────────────────────
  if (use_camera_info_) {
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        cameraInfoCallback(msg);
      });
    RCLCPP_INFO(get_logger(), "Waiting for camera_info on: %s",
      camera_info_topic_.c_str());
  } else {
    setupCameraIntrinsics();
  }

  setupCamToBodyRotation(rpy[0], rpy[1], rpy[2]);

  // ── YOLO model ─────────────────────────────────────────────────────────────
  cv::setNumThreads(4);
  cv::setUseOptimized(true);
  try {
    net_ = cv::dnn::readNetFromONNX(model_path_);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    RCLCPP_INFO(get_logger(), "ONNX model loaded: %s", model_path_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to load ONNX model (%s): %s",
      model_path_.c_str(), e.what());
    throw;
  }

  // ── MAVROS subscriptions ───────────────────────────────────────────────────
  // always subscribe to pose_topic for altitude gate (position.z = local AGL)
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      poseCallback(msg);
    });

  if (use_fixed_pose_) {
    RCLCPP_INFO(get_logger(),
      "Fixed pose mode: lat=%.6f lon=%.6f alt=%.1fm | altitude gate from: %s",
      fixed_lat_, fixed_lon_, fixed_altitude_m_, pose_topic_.c_str());
    home_lat_ = fixed_lat_;
    home_lon_ = fixed_lon_;
    home_set_ = true;
  } else {
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gpsCallback(msg);
      });
    rel_alt_sub_ = create_subscription<std_msgs::msg::Float64>(
      rel_alt_topic_, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        relAltCallback(msg);
      });
    RCLCPP_INFO(get_logger(), "Subscribing MAVROS: %s | %s | %s",
      gps_topic_.c_str(), pose_topic_.c_str(), rel_alt_topic_.c_str());
  }

  // ── image input ───────────────────────────────────────────────────────────
  if (input_mode_ == "topic") {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_source_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        imageCallback(msg);
      });
    RCLCPP_INFO(get_logger(), "Subscribing image topic: %s", input_source_.c_str());
  } else {
    openInputSource();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&VisionGeoNode::processFrame, this));
  }

  // ── publishers ────────────────────────────────────────────────────────────
  detections_pub_ = create_publisher<vision_msgs::msg::DetectedObjectArray>(
    output_topic_, rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  for (size_t i = 0; i < class_names_.size(); ++i) {
    target_pubs_.push_back(create_publisher<geometry_msgs::msg::PoseStamped>(
      target_topic_prefix_ + class_names_[i], rclcpp::QoS(10)));
    RCLCPP_INFO(get_logger(), "class %zu '%s' (%.2f m)%s → %s%s", i,
      class_names_[i].c_str(),
      i < object_size_m_.size() ? object_size_m_[i] : object_size_m_.back(),
      is_geo_class_[i] ? " [geo]" : "",
      target_topic_prefix_.c_str(), class_names_[i].c_str());
  }
  for (const std::string state : {"red", "green", "entry", "exit", "off"}) {
    state_pubs_[state] = create_publisher<vision_msgs::msg::DetectedObjectArray>(
      map_topic_prefix_ + "buoy_" + state, rclcpp::QoS(10));
    RCLCPP_INFO(get_logger(), "buoy map → %sbuoy_%s", map_topic_prefix_.c_str(), state.c_str());
  }

  // ── window ────────────────────────────────────────────────────────────────
  if (show_window_) {
    try {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
      RCLCPP_INFO(get_logger(), "Visualization window: %s", window_name_.c_str());
    } catch (const cv::Exception & e) {
      show_window_ = false;
      RCLCPP_WARN(get_logger(), "Failed to create window: %s", e.what());
    }
  }

  RCLCPP_INFO(get_logger(),
    "vision_geo ready — publishing detections → %s | markers → %s",
    output_topic_.c_str(), marker_topic_.c_str());
}

// ── Destructor ─────────────────────────────────────────────────────────────────

VisionGeoNode::~VisionGeoNode()
{
  if (stream_writer_.isOpened()) {
    stream_writer_.release();
  }
  if (show_window_) {
    cv::destroyWindow(window_name_);
  }
}

// ── Setup helpers ──────────────────────────────────────────────────────────────

void VisionGeoNode::setupCameraIntrinsics()
{
  std::vector<double> cam_data = declare_parameter<std::vector<double>>(
    "camera_matrix",
    {504.99132615, 0.0, 318.4012728,
     0.0, 506.13168687, 243.99035432,
     0.0, 0.0, 1.0});
  std::vector<double> dist_data = declare_parameter<std::vector<double>>(
    "distortion_coeffs",
    {0.0, 0.0, 0.0, 0.0, 0.0});

  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  if (cam_data.size() == 9) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        camera_matrix_.at<double>(r, c) = cam_data[static_cast<size_t>(r * 3 + c)];
      }
    }
  } else {
    RCLCPP_WARN(get_logger(), "camera_matrix must have 9 values. Using identity.");
  }

  dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
  for (size_t i = 0; i < std::min(dist_data.size(), size_t(5)); ++i) {
    dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_data[i];
  }

  K_inv_ = camera_matrix_.inv();
}

void VisionGeoNode::setupCamToBodyRotation(
  double roll_deg, double pitch_deg, double yaw_deg)
{
  R_cam2body_ = rpy2mat(roll_deg, pitch_deg, yaw_deg);
  RCLCPP_INFO(get_logger(),
    "cam_to_body_rpy: roll=%.1f° pitch=%.1f° yaw=%.1f°",
    roll_deg, pitch_deg, yaw_deg);
}

bool VisionGeoNode::startsWith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

void VisionGeoNode::openInputSource()
{
  if (startsWith(input_source_, "/dev/video")) {
    const std::string idx = input_source_.substr(std::string("/dev/video").size());
    if (!idx.empty()) {
      try {
        if (capture_.open(std::stoi(idx), cv::CAP_V4L2)) {
          using_camera_device_ = true;
        }
      } catch (const std::exception &) {}
    }
  }
  if (!capture_.isOpened()) {
    capture_.open(input_source_);
    using_camera_device_ = startsWith(input_source_, "/dev/video");
  }
  if (!capture_.isOpened()) {
    throw std::runtime_error("failed to open input: " + input_source_);
  }
  RCLCPP_INFO(get_logger(), "Input: %s", input_source_.c_str());
}

// ── Callbacks ──────────────────────────────────────────────────────────────────

void VisionGeoNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (camera_info_received_) return;
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = msg->k[0];
  camera_matrix_.at<double>(0, 2) = msg->k[2];
  camera_matrix_.at<double>(1, 1) = msg->k[4];
  camera_matrix_.at<double>(1, 2) = msg->k[5];
  dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
  for (size_t i = 0; i < std::min(msg->d.size(), size_t(5)); ++i) {
    dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
  }
  K_inv_ = camera_matrix_.inv();
  camera_info_received_ = true;
  RCLCPP_INFO(get_logger(),
    "Camera intrinsics from topic: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
    msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
}

void VisionGeoNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (use_camera_info_ && !camera_info_received_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for camera_info...");
    return;
  }
  cv::Mat frame;
  const std::string & enc = msg->encoding;
  if (enc == "bgr8") {
    cv::Mat view(msg->height, msg->width, CV_8UC3,
      const_cast<uint8_t *>(msg->data.data()), msg->step);
    frame = view.clone();
  } else if (enc == "rgb8") {
    cv::Mat view(msg->height, msg->width, CV_8UC3,
      const_cast<uint8_t *>(msg->data.data()), msg->step);
    cv::cvtColor(view, frame, cv::COLOR_RGB2BGR);
  } else if (enc == "mono8") {
    cv::Mat view(msg->height, msg->width, CV_8UC1,
      const_cast<uint8_t *>(msg->data.data()), msg->step);
    cv::cvtColor(view, frame, cv::COLOR_GRAY2BGR);
  } else {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
      "Unsupported encoding: %s", enc.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = frame;
    new_frame_ = true;
  }
  processFrame();
}

void VisionGeoNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  mavros_state_.lat = msg->latitude;
  mavros_state_.lon = msg->longitude;
  mavros_state_.stamp = now();
  mavros_state_.gps_valid = true;
  if (!home_set_) {
    home_lat_ = msg->latitude;
    home_lon_ = msg->longitude;
    home_set_ = true;
    RCLCPP_INFO(get_logger(), "Home position set: lat=%.6f lon=%.6f",
      home_lat_, home_lon_);
  }
}

void VisionGeoNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  mavros_state_.qx = msg->pose.orientation.x;
  mavros_state_.qy = msg->pose.orientation.y;
  mavros_state_.qz = msg->pose.orientation.z;
  mavros_state_.qw = msg->pose.orientation.w;
  mavros_state_.x_local     = msg->pose.position.x;
  mavros_state_.y_local     = msg->pose.position.y;
  mavros_state_.alt_local   = msg->pose.position.z;
  mavros_state_.pose_valid  = true;
  mavros_state_.pose_z_valid = true;
}

void VisionGeoNode::relAltCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  mavros_state_.alt_agl = static_cast<double>(msg->data);
}

// ── Math helpers ───────────────────────────────────────────────────────────────

cv::Mat VisionGeoNode::quaternionToMat(double x, double y, double z, double w)
{
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
  R.at<double>(0, 0) = 1.0 - 2.0 * (y*y + z*z);
  R.at<double>(0, 1) = 2.0 * (x*y - w*z);
  R.at<double>(0, 2) = 2.0 * (x*z + w*y);
  R.at<double>(1, 0) = 2.0 * (x*y + w*z);
  R.at<double>(1, 1) = 1.0 - 2.0 * (x*x + z*z);
  R.at<double>(1, 2) = 2.0 * (y*z - w*x);
  R.at<double>(2, 0) = 2.0 * (x*z - w*y);
  R.at<double>(2, 1) = 2.0 * (y*z + w*x);
  R.at<double>(2, 2) = 1.0 - 2.0 * (x*x + y*y);
  return R;
}

cv::Mat VisionGeoNode::rpy2mat(double roll_deg, double pitch_deg, double yaw_deg)
{
  const double r = roll_deg  * M_PI / 180.0;
  const double p = pitch_deg * M_PI / 180.0;
  const double y = yaw_deg   * M_PI / 180.0;

  cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
    1,      0,       0,
    0,  cos(r), -sin(r),
    0,  sin(r),  cos(r));
  cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
    cos(p),  0, sin(p),
    0,       1,      0,
    -sin(p), 0, cos(p));
  cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
    cos(y), -sin(y), 0,
    sin(y),  cos(y), 0,
    0,       0,      1);

  return Rz * Ry * Rx;
}

// ── Map-absolute projection ──────────────────────────────────────────────────
//
// Returns the buoy position in the map (ENU, meters from home), NOT relative to
// the drone. The whole fix lives here:
//
//     buoy_map = drone_map_position + ground_offset_from_camera
//
// The drone's own translation across the world (from local_position/pose) is
// ADDED, so a stationary buoy keeps a fixed map position while the drone flies —
// it no longer "follows" the camera. GPS lat/lon is derived from the map ENU.
//
// Frame convention — empirically verified on this ArduPilot/MAVROS hardware:
//   x_local → North,  y_local → West (= -East),  z_local → Up
//   drone_north = x_local,  drone_east = -y_local
VisionGeoNode::GeoPoint VisionGeoNode::projectPixelToGPS(
  float u, float v, const MavrosState & state) const
{
  GeoPoint result;

  // altitude: always use pose.position.z when available
  const double alt = state.pose_z_valid ? state.alt_local
    : (use_fixed_pose_ ? fixed_altitude_m_ : state.alt_agl);
  if (alt < 0.5) {
    return result;  // too low to project reliably
  }

  // 1. pixel → camera ray
  cv::Mat pixel = (cv::Mat_<double>(3, 1) << static_cast<double>(u),
                                              static_cast<double>(v),
                                              1.0);
  cv::Mat ray_cam = K_inv_ * pixel;

  // 2. camera → body frame
  cv::Mat ray_body = R_cam2body_ * ray_cam;

  // 3. body → native world frame using drone attitude quaternion.
  //    Use the real orientation whenever pose is valid (works in both modes);
  //    this rotates the ground offset by the drone heading as it surveys.
  cv::Mat R_body2world;
  if (state.pose_valid) {
    R_body2world = quaternionToMat(state.qx, state.qy, state.qz, state.qw);
  } else {
    R_body2world = cv::Mat::eye(3, 3, CV_64F);  // assume level until pose arrives
  }
  cv::Mat ray_world = R_body2world * ray_body;

  // 4. ground intersection (native z = Up, ray must point down)
  const double ray_z = ray_world.at<double>(2);
  if (ray_z >= -0.01) {
    return result;  // ray points up/horizontal — no ground hit
  }
  const double t = alt / (-ray_z);
  // The quaternion produces ray_world in standard ENU: axis0 = East, axis1 = North.
  // MAVROS local_position/pose is also ENU: x = East, y = North, z = Up.
  const double offset_east  = t * ray_world.at<double>(0);
  const double offset_north = t * ray_world.at<double>(1);

  // 5. drone position in the map frame (absolute ENU meters from home).
  //    Empirically verified for this ArduPilot/MAVROS setup:
  //      x_local = North  (forward, drone_north = x_local)
  //      y_local = West   (= -East, so drone_east = -y_local)
  double drone_east  = 0.0;
  double drone_north = 0.0;
  if (state.pose_z_valid) {
    drone_north =  state.x_local;   // x = North
    drone_east  = -state.y_local;   // y = West → negate to get East
  }

  // 6. absolute buoy position in the map frame
  result.north_m = drone_north + offset_north;
  result.east_m  = drone_east  + offset_east;

  // 7. derive GPS from map ENU + home anchor (flat-earth, valid < 500m)
  result.lat = home_lat_ + result.north_m / 111320.0;
  result.lon = home_lon_ + result.east_m  /
    (111320.0 * std::cos(home_lat_ * M_PI / 180.0));
  result.alt   = 0.0;
  result.valid = true;
  return result;
}

// ── Core loop ──────────────────────────────────────────────────────────────────

void VisionGeoNode::processFrame()
{
  cv::Mat frame;

  if (input_mode_ == "topic") {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!new_frame_ || latest_frame_.empty()) return;
    frame = latest_frame_.clone();
    new_frame_ = false;
  } else {
    if (!capture_.read(frame) || frame.empty()) {
      if (using_camera_device_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "Failed to read frame from %s", input_source_.c_str());
        return;
      }
      capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!capture_.read(frame) || frame.empty()) return;
    }
  }

  // snapshot MAVROS state
  MavrosState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = mavros_state_;
  }

  // check pose freshness (only when using real MAVROS)
  if (!use_fixed_pose_) {
    if (!state.gps_valid || !state.pose_valid) {
      std::string missing;
      if (!state.gps_valid)  missing += " GPS on " + gps_topic_;
      if (!state.pose_valid) missing += " pose on " + pose_topic_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "FPS=%.2f | Waiting for MAVROS%s — check the topic names / MAVROS namespace",
        fps_, missing.c_str());
    } else {
      const double age_ms =
        (now() - state.stamp).nanoseconds() / 1e6;
      if (age_ms > max_pose_age_ms_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "FPS=%.2f | MAVROS pose stale (%.0f ms)", fps_, age_ms);
      }
    }
  }

  // camera_info not received yet → no intrinsics, projection would throw
  if (K_inv_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for camera intrinsics (camera_info on %s)...", camera_info_topic_.c_str());
    return;
  }

  // inference
  std::vector<Detection> detections = infer(frame);
  updateFps();

  // GPS projection only for geo classes (Task 1 buoys); others stay invalid
  std::vector<GeoPoint> geo_points(detections.size());
  std::vector<Detection> geo_dets;
  std::vector<GeoPoint> geo_dets_points;
  for (size_t i = 0; i < detections.size(); ++i) {
    const int cid = detections[i].class_id;
    if (cid < 0 || cid >= static_cast<int>(is_geo_class_.size()) || !is_geo_class_[cid]) continue;
    const float u = detections[i].box.x + detections[i].box.width  * 0.5f;
    const float v = detections[i].box.y + detections[i].box.height * 0.5f;
    geo_points[i] = projectPixelToGPS(u, v, state);
    geo_dets.push_back(detections[i]);
    geo_dets_points.push_back(geo_points[i]);
  }

  const rclcpp::Time stamp = now();
  publishDetections(detections, geo_points, stamp);
  publishTargets(detections, stamp);
  updateMarkers(geo_dets, geo_dets_points, stamp);
  publishMarkers();
  publishBuoyMap(stamp);
  streamFrame(frame, detections, geo_points);
  displayFrame(frame, detections, geo_points);

  if (!detections.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "FPS=%.2f | pose.z=%.1fm | %zu object(s) detected",
      fps_, state.alt_local, detections.size());
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "FPS=%.2f | pose.z=%.1fm | No objects detected", fps_, state.alt_local);
  }
}

// ── Inference ──────────────────────────────────────────────────────────────────

std::vector<VisionGeoNode::Detection> VisionGeoNode::infer(const cv::Mat & frame)
{
  std::vector<Detection> detections;

  cv::Mat blob;
  cv::dnn::blobFromImage(
    frame, blob, 1.0 / 255.0, cv::Size(input_width_, input_height_),
    cv::Scalar(), true, false);

  cv::Mat out;
  try {
    net_.setInput(blob);
    out = net_.forward();
  } catch (const cv::Exception & primary) {
    try {
      std::vector<cv::Mat> outputs;
      net_.forward(outputs, net_.getUnconnectedOutLayersNames());
      if (outputs.empty()) return detections;
      out = outputs[0];
    } catch (const cv::Exception & fallback) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "DNN forward failed: %s | %s", primary.what(), fallback.what());
      return detections;
    }
  }

  if (out.dims != 3) return detections;

  const int dim1 = out.size[1];
  const int dim2 = out.size[2];
  const bool channels_first = (dim1 < dim2);
  const int num_candidates = channels_first ? dim2 : dim1;
  const int num_features   = channels_first ? dim1 : dim2;
  if (num_features < 5) return detections;

  const int num_classes = num_features - 4;
  const float x_scale = static_cast<float>(frame.cols) / static_cast<float>(input_width_);
  const float y_scale = static_cast<float>(frame.rows) / static_cast<float>(input_height_);

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for (int i = 0; i < num_candidates; ++i) {
    float cx, cy, w, h;
    if (channels_first) {
      cx = out.at<float>(0, 0, i); cy = out.at<float>(0, 1, i);
      w  = out.at<float>(0, 2, i); h  = out.at<float>(0, 3, i);
    } else {
      cx = out.at<float>(0, i, 0); cy = out.at<float>(0, i, 1);
      w  = out.at<float>(0, i, 2); h  = out.at<float>(0, i, 3);
    }

    int best_class = -1;
    float best_score = 0.0f;
    for (int c = 0; c < num_classes; ++c) {
      float score = channels_first ?
        out.at<float>(0, c + 4, i) : out.at<float>(0, i, c + 4);
      if (score > best_score) { best_score = score; best_class = c; }
    }
    if (best_score < static_cast<float>(conf_threshold_)) continue;

    int left  = static_cast<int>((cx - 0.5f * w) * x_scale);
    int top   = static_cast<int>((cy - 0.5f * h) * y_scale);
    int width = static_cast<int>(w * x_scale);
    int height = static_cast<int>(h * y_scale);
    cv::Rect box(left, top, width, height);
    box &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (box.width <= 0 || box.height <= 0) continue;

    class_ids.push_back(best_class);
    confidences.push_back(best_score);
    boxes.push_back(box);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences,
    static_cast<float>(conf_threshold_),
    static_cast<float>(nms_threshold_),
    indices);

  detections.reserve(indices.size());
  for (int idx : indices) {
    detections.push_back(Detection{class_ids[idx], confidences[idx], boxes[idx]});
  }
  return detections;
}

// ── Publishing ─────────────────────────────────────────────────────────────────

void VisionGeoNode::publishDetections(
  const std::vector<Detection> & dets,
  const std::vector<GeoPoint> & geo,
  const rclcpp::Time & stamp)
{
  auto arr = std::make_unique<vision_msgs::msg::DetectedObjectArray>();
  arr->header.stamp    = stamp;
  arr->header.frame_id = marker_frame_id_;

  for (size_t i = 0; i < dets.size(); ++i) {
    vision_msgs::msg::DetectedObject obj;
    obj.header.stamp    = stamp;
    obj.header.frame_id = marker_frame_id_;
    obj.class_id    = dets[i].class_id;
    obj.confidence  = dets[i].confidence;
    obj.pixel_u     = dets[i].box.x + dets[i].box.width  * 0.5f;
    obj.pixel_v     = dets[i].box.y + dets[i].box.height * 0.5f;

    const int cid = dets[i].class_id;
    obj.label = (cid >= 0 && cid < static_cast<int>(class_names_.size()))
      ? class_names_[static_cast<size_t>(cid)]
      : ("class_" + std::to_string(cid));

    if (i < geo.size() && geo[i].valid) {
      obj.latitude       = geo[i].lat;
      obj.longitude      = geo[i].lon;
      obj.altitude_m     = geo[i].alt;
      obj.north_offset_m = static_cast<float>(geo[i].north_m);
      obj.east_offset_m  = static_cast<float>(geo[i].east_m);
    }
    arr->objects.push_back(obj);
  }
  detections_pub_->publish(std::move(arr));
}

// Same method as vision_video: bounding-box corners ↔ square of the real object size.
bool VisionGeoNode::estimatePose(const Detection & det, cv::Mat & rvec, cv::Mat & tvec) const
{
  const size_t cid = static_cast<size_t>(det.class_id);
  const double size = cid < object_size_m_.size() ? object_size_m_[cid] : object_size_m_.back();
  const float h = static_cast<float>(size * 0.5);
  const std::vector<cv::Point3f> object_points = {
    {-h,  h, 0.0f},
    { h,  h, 0.0f},
    { h, -h, 0.0f},
    {-h, -h, 0.0f}
  };
  const std::vector<cv::Point2f> image_points = {
    {static_cast<float>(det.box.x),                   static_cast<float>(det.box.y)},
    {static_cast<float>(det.box.x + det.box.width),   static_cast<float>(det.box.y)},
    {static_cast<float>(det.box.x + det.box.width),   static_cast<float>(det.box.y + det.box.height)},
    {static_cast<float>(det.box.x),                   static_cast<float>(det.box.y + det.box.height)}
  };
  if (camera_matrix_.empty()) return false;  // camera_info not received yet
  try {
    return cv::solvePnP(
      object_points, image_points, camera_matrix_, dist_coeffs_,
      rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
  } catch (const cv::Exception &) {
    return false;
  }
}

// One PoseStamped per class every frame (camera frame, meters, like vision_video):
//   x = right, y = down, z = forward (distance) — all zeros when not detected.
// With 2+ detections of a class, the closest one (smallest distance) is published.
// centering_red() reads this directly.
void VisionGeoNode::publishTargets(
  const std::vector<Detection> & dets, const rclcpp::Time & stamp)
{
  for (size_t c = 0; c < target_pubs_.size(); ++c) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = pose_frame_id_;
    msg.pose.orientation.w = 1.0;

    // closest detection of this class
    cv::Mat rvec, tvec;
    double best_dist = -1.0;
    for (const auto & d : dets) {
      if (d.class_id != static_cast<int>(c)) continue;
      cv::Mat r, t;
      if (!estimatePose(d, r, t)) continue;
      const double dist = cv::norm(t);
      if (best_dist < 0.0 || dist < best_dist) {
        best_dist = dist;
        rvec = r;
        tvec = t;
      }
    }

    if (best_dist >= 0.0) {
      msg.pose.position.x = tvec.at<double>(0);
      msg.pose.position.y = tvec.at<double>(1);
      msg.pose.position.z = tvec.at<double>(2);

      // rvec (axis-angle) → quaternion
      const double angle = cv::norm(rvec);
      if (angle > 1e-9) {
        const double s = std::sin(angle * 0.5) / angle;
        msg.pose.orientation.x = rvec.at<double>(0) * s;
        msg.pose.orientation.y = rvec.at<double>(1) * s;
        msg.pose.orientation.z = rvec.at<double>(2) * s;
        msg.pose.orientation.w = std::cos(angle * 0.5);
      }
    }
    target_pubs_[c]->publish(msg);
  }
}

// Confirmed buoys split by light state, e.g. /vision_geo/map/buoy_entry
void VisionGeoNode::publishBuoyMap(const rclcpp::Time & stamp)
{
  if (!home_set_) return;

  for (const auto & [state, pub] : state_pubs_) {
    vision_msgs::msg::DetectedObjectArray arr;
    arr.header.stamp    = stamp;
    arr.header.frame_id = marker_frame_id_;

    for (const auto & pt : buoy_map_) {
      if (pt.state != state) continue;
      vision_msgs::msg::DetectedObject obj;
      obj.header         = arr.header;
      obj.class_id       = pt.buoy_id;
      obj.label          = "buoy_" + pt.state;
      obj.confidence     = pt.best_confidence;
      obj.latitude       = pt.lat;
      obj.longitude      = pt.lon;
      obj.north_offset_m = static_cast<float>(pt.north_m);
      obj.east_offset_m  = static_cast<float>(pt.east_m);
      arr.objects.push_back(obj);
    }
    pub->publish(arr);
  }
}

void VisionGeoNode::updateMarkers(
  const std::vector<Detection> & dets,
  const std::vector<GeoPoint> & geo,
  const rclcpp::Time & stamp)
{
  if (!home_set_) return;

  // ── step 1: match detections → active tracks (pixel space) ───────────────
  std::vector<bool> det_matched(dets.size(), false);
  std::vector<bool> trk_matched(active_tracks_.size(), false);

  for (size_t ti = 0; ti < active_tracks_.size(); ++ti) {
    float best_dist = static_cast<float>(pixel_match_radius_px_);
    int   best_di   = -1;

    for (size_t di = 0; di < dets.size(); ++di) {
      if (det_matched[di]) continue;   // any buoy class: a flashing buoy changes class

      const float dcx = dets[di].box.x + dets[di].box.width  * 0.5f;
      const float dcy = dets[di].box.y + dets[di].box.height * 0.5f;
      const float dx  = dcx - active_tracks_[ti].cx;
      const float dy  = dcy - active_tracks_[ti].cy;
      const float dist = std::sqrt(dx * dx + dy * dy);

      if (dist < best_dist) { best_dist = dist; best_di = static_cast<int>(di); }
    }

    if (best_di >= 0) {
      trk_matched[ti] = true;
      det_matched[best_di] = true;
      auto & t = active_tracks_[ti];
      t.cx = dets[best_di].box.x + dets[best_di].box.width  * 0.5f;
      t.cy = dets[best_di].box.y + dets[best_di].box.height * 0.5f;
      t.class_id      = dets[best_di].class_id;
      t.confidence    = dets[best_di].confidence;
      t.frames_missing = 0;

      // light observation: straight into the map point, or buffered until linked
      if (t.linked_buoy_id >= 0) {
        for (auto & pt : buoy_map_) {
          if (pt.buoy_id == t.linked_buoy_id) {
            addLightObservation(pt, t.class_id, stamp);
            break;
          }
        }
      } else {
        t.class_buf.emplace_back(stamp, t.class_id);
      }

      const bool has_geo =
        best_di < static_cast<int>(geo.size()) && geo[best_di].valid;
      if (has_geo) {
        const double e = geo[best_di].east_m;
        const double n = geo[best_di].north_m;
        if (t.linked_buoy_id >= 0) {
          // already confirmed — feed each new observation into its running mean
          for (auto & pt : buoy_map_) {
            if (pt.buoy_id == t.linked_buoy_id) {
              addObservationToBuoy(pt, e, n, t.confidence, stamp);
              break;
            }
          }
        } else {
          t.enu_buf.emplace_back(e, n);
        }
      }
    }
  }

  // ── step 2: confirm tracks that reached enough observations ───────────────
  //    A confirmed track links to a map point (new or existing within radius),
  //    so the buoy appears on the map immediately — no need to exit the frame.
  for (auto & t : active_tracks_) {
    if (t.linked_buoy_id >= 0) continue;
    if (static_cast<int>(t.enu_buf.size()) < min_obs_to_commit_) continue;

    double sum_e = 0.0, sum_n = 0.0;
    for (const auto & [e, n_] : t.enu_buf) { sum_e += e; sum_n += n_; }
    const double n       = static_cast<double>(t.enu_buf.size());
    const double mean_e  = sum_e / n;
    const double mean_n  = sum_n / n;

    // merge check: any buoy within radius → SAME buoy (class may differ while flashing)
    BuoyMapPoint * nearest = nullptr;
    double nearest_dist = map_merge_radius_m_;
    for (auto & pt : buoy_map_) {
      const double dn   = mean_n - pt.north_m;
      const double de   = mean_e - pt.east_m;
      const double dist = std::sqrt(dn * dn + de * de);
      if (dist < nearest_dist) { nearest_dist = dist; nearest = &pt; }
    }

    if (nearest) {
      // existing buoy re-sighted — fold this sighting's observations into its mean
      nearest->sighting_count++;
      for (const auto & [e, n_] : t.enu_buf)
        addObservationToBuoy(*nearest, e, n_, t.confidence, stamp);
      for (const auto & [ts, cid] : t.class_buf)
        addLightObservation(*nearest, cid, ts);
      t.linked_buoy_id = nearest->buoy_id;
      RCLCPP_INFO(get_logger(),
        "[map] buoy#%d '%s' re-sighted (sightings=%d, obs=%d, match=%.1fm)",
        nearest->buoy_id, nearest->state.c_str(),
        nearest->sighting_count, nearest->obs_count, nearest_dist);
    } else {
      // new buoy — seed a fresh map point with the buffered observations
      BuoyMapPoint pt;
      pt.buoy_id        = next_buoy_id_++;
      pt.class_id       = t.class_id;
      pt.label          = "buoy";
      pt.sighting_count = 1;
      for (const auto & [e, n_] : t.enu_buf)
        addObservationToBuoy(pt, e, n_, t.confidence, stamp);
      for (const auto & [ts, cid] : t.class_buf)
        addLightObservation(pt, cid, ts);
      t.linked_buoy_id = pt.buoy_id;
      buoy_map_.push_back(pt);
      RCLCPP_INFO(get_logger(),
        "[map] buoy#%d NEW E=%.1fm N=%.1fm lat=%.6f lon=%.6f (%d obs)",
        pt.buoy_id, pt.east_m, pt.north_m,
        pt.lat, pt.lon, pt.obs_count);
    }
    t.enu_buf.clear();  // observations now live in the map point
    t.class_buf.clear();
  }

  // ── step 3: age unmatched tracks; drop those missing too long ─────────────
  //    A dropped track's map point (if it was confirmed) stays permanent.
  std::vector<ActiveTrack> still_active;
  for (size_t ti = 0; ti < active_tracks_.size(); ++ti) {
    auto & t = active_tracks_[ti];
    if (trk_matched[ti]) {
      still_active.push_back(std::move(t));       // seen this frame
    } else {
      t.frames_missing++;
      if (t.frames_missing < track_exit_frames_)
        still_active.push_back(std::move(t));      // grace period (occlusion/flicker)
      // else: exited frame — discard track
    }
  }
  active_tracks_ = std::move(still_active);

  // ── step 4: unmatched detections → new active tracks ─────────────────────
  for (size_t di = 0; di < dets.size(); ++di) {
    if (det_matched[di]) continue;
    const int cid = dets[di].class_id;
    ActiveTrack t;
    t.class_id  = cid;
    t.label     = (cid >= 0 && cid < static_cast<int>(class_names_.size()))
      ? class_names_[static_cast<size_t>(cid)] : ("class_" + std::to_string(cid));
    t.confidence = dets[di].confidence;
    t.cx = dets[di].box.x + dets[di].box.width  * 0.5f;
    t.cy = dets[di].box.y + dets[di].box.height * 0.5f;
    if (di < geo.size() && geo[di].valid) {
      t.enu_buf.emplace_back(geo[di].east_m, geo[di].north_m);
    }
    t.class_buf.emplace_back(stamp, cid);
    active_tracks_.push_back(t);
  }

  // ── step 5: refresh every buoy's light state from its recent history ─────
  for (auto & pt : buoy_map_) {
    updateBuoyState(pt, stamp);
  }
}

std::string VisionGeoNode::lightColor(int class_id) const
{
  if (class_id < 0 || class_id >= static_cast<int>(class_names_.size())) return "off";
  const std::string & name = class_names_[static_cast<size_t>(class_id)];
  for (const std::string color : {"red", "green", "blue"}) {
    if (name.find(color) != std::string::npos) return color;
  }
  return "off";
}

void VisionGeoNode::addLightObservation(
  BuoyMapPoint & pt, int class_id, const rclcpp::Time & stamp)
{
  pt.light_hist.emplace_back(stamp, lightColor(class_id));
}

// Light pattern from the last state_window_s of observations:
//   lit red/green              → "red" / "green"
//   lit blue + flashing        → "entry"  (off frames between lit ones, or an on/off-sized gap)
//   lit blue, solid long enough → "exit"
//   only off, long enough      → "off"
// Not enough evidence → keep the previous state (also when the buoy is out of view).
void VisionGeoNode::updateBuoyState(BuoyMapPoint & pt, const rclcpp::Time & stamp)
{
  const rclcpp::Time window_start = stamp - rclcpp::Duration::from_seconds(state_window_s_);
  while (!pt.light_hist.empty() && pt.light_hist.front().first < window_start) {
    pt.light_hist.pop_front();
  }
  if (pt.light_hist.empty()) return;

  std::map<std::string, int> lit_count;
  rclcpp::Time first_lit = stamp, last_lit = window_start, prev_lit = window_start;
  bool have_lit = false, flashing = false;
  for (const auto & [ts, color] : pt.light_hist) {
    if (color == "off") continue;
    lit_count[color]++;
    if (have_lit) {
      const double gap = (ts - prev_lit).seconds();
      if (gap >= flash_gap_min_s_ && gap <= flash_gap_max_s_) flashing = true;
    } else {
      first_lit = ts;
    }
    prev_lit = last_lit = ts;
    have_lit = true;
  }
  if (have_lit) {
    for (const auto & [ts, color] : pt.light_hist) {
      if (color == "off" && ts > first_lit && ts < last_lit) { flashing = true; break; }
    }
  }

  std::string new_state = pt.state;
  if (!have_lit) {
    const double off_span = (pt.light_hist.back().first - pt.light_hist.front().first).seconds();
    if (off_span >= flash_gap_max_s_) new_state = "off";
  } else {
    std::string color;
    int best = 0;
    for (const auto & [c, n] : lit_count) {
      if (n > best) { best = n; color = c; }
    }
    if (color != "blue") {
      new_state = color;
    } else if (flashing) {
      new_state = "entry";
    } else if ((last_lit - first_lit).seconds() >= solid_min_s_) {
      new_state = "exit";
    }
  }

  if (new_state != pt.state) {
    RCLCPP_INFO(get_logger(), "[map] buoy#%d state %s → %s (lat=%.6f lon=%.6f)",
      pt.buoy_id, pt.state.c_str(), new_state.c_str(), pt.lat, pt.lon);
    pt.state = new_state;
  }
}

std::tuple<float, float, float> VisionGeoNode::stateColor(const std::string & state)
{
  if (state == "red")   return {0.96f, 0.26f, 0.21f};
  if (state == "green") return {0.30f, 0.69f, 0.31f};
  if (state == "entry") return {0.13f, 0.59f, 0.95f};  // flashing blue
  if (state == "exit")  return {0.00f, 0.20f, 0.70f};  // solid blue (darker)
  if (state == "off")   return {0.25f, 0.25f, 0.25f};
  return {1.0f, 1.0f, 1.0f};                           // unknown
}

void VisionGeoNode::addObservationToBuoy(
  BuoyMapPoint & pt, double east_m, double north_m,
  float confidence, const rclcpp::Time & stamp)
{
  pt.sum_east  += east_m;
  pt.sum_north += north_m;
  pt.obs_count += 1;
  // running mean — converges in place (safe now that projection is map-absolute)
  pt.east_m  = pt.sum_east  / pt.obs_count;
  pt.north_m = pt.sum_north / pt.obs_count;
  pt.lat = home_lat_ + pt.north_m / 111320.0;
  pt.lon = home_lon_ + pt.east_m  /
    (111320.0 * std::cos(home_lat_ * M_PI / 180.0));
  if (confidence > pt.best_confidence) pt.best_confidence = confidence;
  pt.last_seen = stamp;
}

void VisionGeoNode::publishMarkers()
{
  if (!home_set_) return;

  visualization_msgs::msg::MarkerArray arr;
  const auto t_now = now();

  // DELETEALL every frame — we republish entire map so IDs stay stable
  visualization_msgs::msg::Marker del;
  del.action          = visualization_msgs::msg::Marker::DELETEALL;
  del.header.frame_id = marker_frame_id_;
  del.header.stamp    = t_now;
  arr.markers.push_back(del);

  int id = 0;
  const auto permanent = rclcpp::Duration(0, 0);

  // ── confirmed buoy map ────────────────────────────────────────────────────
  for (const auto & pt : buoy_map_) {
    auto [cr, cg, cb] = stateColor(pt.state);

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id    = marker_frame_id_;
    sphere.header.stamp       = t_now;
    sphere.ns                 = "buoys";
    sphere.id                 = id++;
    sphere.type               = visualization_msgs::msg::Marker::SPHERE;
    sphere.action             = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x    = pt.east_m;
    sphere.pose.position.y    = pt.north_m;
    sphere.pose.position.z    = 0.0;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = marker_size_;
    sphere.scale.y = marker_size_;
    sphere.scale.z = marker_size_ * 0.3;
    sphere.color.r = cr; sphere.color.g = cg; sphere.color.b = cb;
    sphere.color.a = 0.9f;
    sphere.lifetime = permanent;
    arr.markers.push_back(sphere);

    visualization_msgs::msg::Marker text;
    text.header           = sphere.header;
    text.ns               = "labels";
    text.id               = id++;
    text.type             = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action           = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x  = pt.east_m;
    text.pose.position.y  = pt.north_m;
    text.pose.position.z  = marker_size_ * 0.8;
    text.pose.orientation.w = 1.0;
    text.scale.z          = marker_size_ * 0.55;
    text.color.r = 1.0f; text.color.g = 1.0f;
    text.color.b = 1.0f; text.color.a = 1.0f;
    text.text = cv::format("#%d %s\n[%d obs / %d pass]",
      pt.buoy_id, pt.state.c_str(), pt.obs_count, pt.sighting_count);
    text.lifetime = permanent;
    arr.markers.push_back(text);
  }

  // ── active tracks (currently in frame, yellow ring, short lifetime) ───────
  const auto tracking_lt = rclcpp::Duration::from_seconds(0.5);
  for (const auto & t : active_tracks_) {
    // running mean of buffered map positions — smooth live estimate
    if (t.enu_buf.empty()) continue;
    double se = 0.0, sn = 0.0;
    for (const auto & [e, n_] : t.enu_buf) { se += e; sn += n_; }
    const double aeast  = se / static_cast<double>(t.enu_buf.size());
    const double anorth = sn / static_cast<double>(t.enu_buf.size());

    visualization_msgs::msg::Marker ring;
    ring.header.frame_id    = marker_frame_id_;
    ring.header.stamp       = t_now;
    ring.ns                 = "tracking";
    ring.id                 = id++;
    ring.type               = visualization_msgs::msg::Marker::CYLINDER;
    ring.action             = visualization_msgs::msg::Marker::ADD;
    ring.pose.position.x    = aeast;
    ring.pose.position.y    = anorth;
    ring.pose.position.z    = 0.0;
    ring.pose.orientation.w = 1.0;
    ring.scale.x = marker_size_ * 1.3;
    ring.scale.y = marker_size_ * 1.3;
    ring.scale.z = 0.05;
    ring.color.r = 1.0f; ring.color.g = 0.9f;
    ring.color.b = 0.0f; ring.color.a = 0.5f;
    ring.lifetime = tracking_lt;
    arr.markers.push_back(ring);

    visualization_msgs::msg::Marker tlabel;
    tlabel.header           = ring.header;
    tlabel.ns               = "tracking";
    tlabel.id               = id++;
    tlabel.type             = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    tlabel.action           = visualization_msgs::msg::Marker::ADD;
    tlabel.pose.position.x  = aeast;
    tlabel.pose.position.y  = anorth;
    tlabel.pose.position.z  = marker_size_ * 0.9;
    tlabel.pose.orientation.w = 1.0;
    tlabel.scale.z          = marker_size_ * 0.45;
    tlabel.color.r = 1.0f; tlabel.color.g = 1.0f;
    tlabel.color.b = 0.0f; tlabel.color.a = 1.0f;
    tlabel.text = t.label + cv::format(" [%zu obs]", t.enu_buf.size());
    tlabel.lifetime = tracking_lt;
    arr.markers.push_back(tlabel);
  }

  // ── drone position marker ─────────────────────────────────────────────────
  {
    // use local_position/pose (x=East, y=North, z=Up from home) for drone TF.
    // this is always the actual drone position regardless of use_fixed_pose_ mode.
    double drone_east  = 0.0;
    double drone_north = 0.0;
    double drone_alt   = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (mavros_state_.pose_z_valid) {
        drone_north =  mavros_state_.x_local;   // x = North
        drone_east  = -mavros_state_.y_local;   // y = West → negate to get East
        drone_alt   = mavros_state_.alt_local;
      } else {
        // fallback before first pose message
        drone_alt = use_fixed_pose_ ? fixed_altitude_m_ : mavros_state_.alt_agl;
      }
    }

    const auto t_now = now();

    // flat disc at ground projection of drone (camera footprint indicator)
    visualization_msgs::msg::Marker footprint;
    footprint.header.frame_id = marker_frame_id_;
    footprint.header.stamp    = t_now;
    footprint.ns              = "drone";
    footprint.id              = 0;
    footprint.type            = visualization_msgs::msg::Marker::CYLINDER;
    footprint.action          = visualization_msgs::msg::Marker::ADD;
    footprint.pose.position.x = drone_east;
    footprint.pose.position.y = drone_north;
    footprint.pose.position.z = 0.0;
    footprint.pose.orientation.w = 1.0;
    footprint.scale.x = 1.2;   // diameter (m)
    footprint.scale.y = 1.2;
    footprint.scale.z = 0.05;  // very flat
    footprint.color.r = 0.0f;
    footprint.color.g = 1.0f;
    footprint.color.b = 1.0f;  // cyan
    footprint.color.a = 0.7f;
    footprint.lifetime = rclcpp::Duration::from_seconds(1.0);
    arr.markers.push_back(footprint);

    // vertical line from ground up to drone altitude
    visualization_msgs::msg::Marker line;
    line.header = footprint.header;
    line.ns     = "drone";
    line.id     = 1;
    line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 0.0f; line.color.g = 1.0f;
    line.color.b = 1.0f; line.color.a = 0.5f;
    line.lifetime = rclcpp::Duration::from_seconds(1.0);
    geometry_msgs::msg::Point p_ground, p_drone;
    p_ground.x = drone_east; p_ground.y = drone_north; p_ground.z = 0.0;
    p_drone.x  = drone_east; p_drone.y  = drone_north; p_drone.z  = drone_alt;
    line.points.push_back(p_ground);
    line.points.push_back(p_drone);
    arr.markers.push_back(line);

    // sphere at drone altitude position
    visualization_msgs::msg::Marker drone_sphere;
    drone_sphere.header = footprint.header;
    drone_sphere.ns     = "drone";
    drone_sphere.id     = 2;
    drone_sphere.type   = visualization_msgs::msg::Marker::SPHERE;
    drone_sphere.action = visualization_msgs::msg::Marker::ADD;
    drone_sphere.pose.position.x = drone_east;
    drone_sphere.pose.position.y = drone_north;
    drone_sphere.pose.position.z = drone_alt;
    drone_sphere.pose.orientation.w = 1.0;
    drone_sphere.scale.x = 0.6;
    drone_sphere.scale.y = 0.6;
    drone_sphere.scale.z = 0.6;
    drone_sphere.color.r = 0.0f;
    drone_sphere.color.g = 1.0f;
    drone_sphere.color.b = 1.0f;
    drone_sphere.color.a = 1.0f;
    drone_sphere.lifetime = rclcpp::Duration::from_seconds(1.0);
    arr.markers.push_back(drone_sphere);

    // text label: GPS + altitude
    visualization_msgs::msg::Marker drone_text;
    drone_text.header = footprint.header;
    drone_text.ns     = "drone";
    drone_text.id     = 3;
    drone_text.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    drone_text.action = visualization_msgs::msg::Marker::ADD;
    drone_text.pose.position.x = drone_east + 0.8;
    drone_text.pose.position.y = drone_north + 0.8;
    drone_text.pose.position.z = drone_alt + 0.8;
    drone_text.pose.orientation.w = 1.0;
    drone_text.scale.z = 0.6;
    drone_text.color.r = 0.0f; drone_text.color.g = 1.0f;
    drone_text.color.b = 1.0f; drone_text.color.a = 1.0f;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const double disp_lat = use_fixed_pose_ ? fixed_lat_ : mavros_state_.lat;
      const double disp_lon = use_fixed_pose_ ? fixed_lon_ : mavros_state_.lon;
      drone_text.text = cv::format(
        "DRONE\nE=%.1fm N=%.1fm\nalt=%.1fm\nlat=%.5f\nlon=%.5f",
        drone_east, drone_north, drone_alt, disp_lat, disp_lon);
    }
    drone_text.lifetime = rclcpp::Duration::from_seconds(1.0);
    arr.markers.push_back(drone_text);

    // ── publish map → drone TF so RViz can follow the drone ─────────────────
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = t_now;
    tf.header.frame_id = marker_frame_id_;   // "map"
    tf.child_frame_id  = "drone";
    tf.transform.translation.x = drone_east;
    tf.transform.translation.y = drone_north;
    tf.transform.translation.z = drone_alt;
    tf.transform.rotation.w    = 1.0;        // no yaw rotation — north stays up
    tf_broadcaster_->sendTransform(tf);
  }

  marker_pub_->publish(arr);
}

std::tuple<float, float, float> VisionGeoNode::classColor(int class_id)
{
  static const float palette[][3] = {
    {0.96f, 0.26f, 0.21f},  // red
    {0.13f, 0.59f, 0.95f},  // blue
    {0.30f, 0.69f, 0.31f},  // green
    {1.00f, 0.76f, 0.03f},  // yellow
    {0.61f, 0.15f, 0.69f},  // purple
    {1.00f, 0.34f, 0.13f},  // orange
    {0.00f, 0.74f, 0.83f},  // cyan
    {0.91f, 0.12f, 0.39f},  // pink
  };
  const int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
  const auto & c = palette[((class_id % n) + n) % n];
  return {c[0], c[1], c[2]};
}

// ── Display ────────────────────────────────────────────────────────────────────

void VisionGeoNode::updateFps()
{
  ++frames_in_window_;
  const auto t = std::chrono::steady_clock::now();
  const double elapsed =
    std::chrono::duration<double>(t - fps_window_start_).count();
  if (elapsed >= 1.0) {
    fps_ = static_cast<double>(frames_in_window_) / elapsed;
    fps_window_start_ = t;
    frames_in_window_ = 0;
  }
}

void VisionGeoNode::drawCrosshair(cv::Mat & frame) const
{
  const cv::Scalar green(0, 255, 0);
  cv::line(frame, {0, frame.rows / 2}, {frame.cols, frame.rows / 2}, green, 1);
  cv::line(frame, {frame.cols / 2, 0}, {frame.cols / 2, frame.rows}, green, 1);
}

void VisionGeoNode::drawDetections(
  cv::Mat & frame,
  const std::vector<Detection> & dets,
  const std::vector<GeoPoint> & geo) const
{
  for (size_t i = 0; i < dets.size(); ++i) {
    const auto & det = dets[i];
    const int cid = det.class_id;
    const std::string name =
      (cid >= 0 && cid < static_cast<int>(class_names_.size()))
      ? class_names_[static_cast<size_t>(cid)]
      : ("class_" + std::to_string(cid));

    auto [cr, cg, cb] = classColor(cid);
    const cv::Scalar color(
      static_cast<int>(cb * 255),
      static_cast<int>(cg * 255),
      static_cast<int>(cr * 255));

    cv::rectangle(frame, det.box, color, 2);

    std::string label;
    if (i < geo.size() && geo[i].valid) {
      label = cv::format("%s %.2f | lat=%.5f lon=%.5f",
        name.c_str(), static_cast<double>(det.confidence),
        geo[i].lat, geo[i].lon);
    } else {
      label = cv::format("%s %.2f | no GPS", name.c_str(),
        static_cast<double>(det.confidence));
    }

    int baseline = 0;
    const cv::Size ts =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    const int top = std::max(det.box.y, ts.height + 4);
    cv::rectangle(frame,
      cv::Point(det.box.x, top - ts.height - 4),
      cv::Point(det.box.x + ts.width, top + baseline - 4),
      color, cv::FILLED);
    cv::putText(frame, label, cv::Point(det.box.x, top - 2),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
  }

  // HUD
  const std::string hud = cv::format("FPS=%.1f | objs=%zu", fps_, dets.size());
  cv::putText(frame, hud, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
    cv::Scalar(0, 0, 0), 3);
  cv::putText(frame, hud, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
    cv::Scalar(0, 255, 0), 1);
}

void VisionGeoNode::displayFrame(
  cv::Mat & frame,
  const std::vector<Detection> & dets,
  const std::vector<GeoPoint> & geo)
{
  if (!show_window_) return;
  try {
    drawCrosshair(frame);
    drawDetections(frame, dets, geo);
    cv::imshow(window_name_, frame);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      RCLCPP_INFO(get_logger(), "Quit key pressed.");
      rclcpp::shutdown();
    }
  } catch (const cv::Exception & e) {
    show_window_ = false;
    RCLCPP_WARN(get_logger(), "Display failed, disabling window: %s", e.what());
  }
}

bool VisionGeoNode::ensureStreamWriter()
{
  if (!enable_stream_) {
    return false;
  }
  if (stream_writer_initialized_) {
    return stream_writer_.isOpened();
  }

  stream_writer_initialized_ = true;
  std::ostringstream pipeline;
  pipeline
    << "appsrc is-live=true do-timestamp=true format=time ! "
    << "video/x-raw,format=BGR,width=" << stream_width_
    << ",height=" << stream_height_
    << ",framerate=" << stream_fps_ << "/1 ! "
    << "videoconvert ! "
    << "x264enc tune=zerolatency bitrate=" << stream_bitrate_kbps_
    << " speed-preset=superfast key-int-max=" << stream_fps_
    << " byte-stream=true ! "
    << "rtph264pay config-interval=1 pt=96 ! "
    << "udpsink host=" << stream_host_
    << " port=" << stream_port_
    << " sync=false async=false";

  if (!stream_writer_.open(
      pipeline.str(), cv::CAP_GSTREAMER, 0,
      static_cast<double>(stream_fps_),
      cv::Size(stream_width_, stream_height_), true))
  {
    RCLCPP_ERROR(get_logger(),
      "Failed to open GStreamer stream pipeline. Disabling stream. Pipeline: %s",
      pipeline.str().c_str());
    enable_stream_ = false;
    return false;
  }

  RCLCPP_INFO(get_logger(), "Streaming enabled to udp://%s:%d (%dx%d @ %d fps)",
    stream_host_.c_str(), stream_port_, stream_width_, stream_height_, stream_fps_);
  return true;
}

void VisionGeoNode::streamFrame(
  const cv::Mat & frame,
  const std::vector<Detection> & dets,
  const std::vector<GeoPoint> & geo)
{
  if (!ensureStreamWriter()) {
    return;
  }
  // draw at the source size (boxes are in source pixels), then scale to the stream size
  cv::Mat annotated = frame.clone();
  drawCrosshair(annotated);
  drawDetections(annotated, dets, geo);
  cv::Mat out;
  if (annotated.cols != stream_width_ || annotated.rows != stream_height_) {
    cv::resize(annotated, out, cv::Size(stream_width_, stream_height_));
  } else {
    out = annotated;
  }
  stream_writer_.write(out);
}

// ── Mission command ────────────────────────────────────────────────────────────

bool waitCommand(const rclcpp::Node::SharedPtr & node, const std::string & expected_command,
                 const std::string & topic, double timeout)
{
  auto toUpperTrim = [](std::string text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    const auto last  = text.find_last_not_of(" \t\r\n");
    text = (first == std::string::npos) ? "" : text.substr(first, last - first + 1);
    std::transform(text.begin(), text.end(), text.begin(), ::toupper);
    return text;
  };

  std::string received_command;
  bool has_command = false;
  auto sub = node->create_subscription<std_msgs::msg::String>(topic, 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      received_command = msg->data;
      has_command = true;
    });

  RCLCPP_INFO(node->get_logger(), "Waiting for '%s' on %s (timeout: %.0fs)...",
    expected_command.c_str(), topic.c_str(), timeout);

  const std::string exp_cmd = toUpperTrim(expected_command);
  const auto start_time = node->now();
  rclcpp::Rate loop_rate(10.0);

  while (rclcpp::ok()) {
    if (has_command) {
      const std::string recv_cmd = toUpperTrim(received_command);
      if (recv_cmd.rfind(exp_cmd, 0) == 0) {
        RCLCPP_INFO(node->get_logger(), "Received command: %s", recv_cmd.c_str());
        return true;
      }
      RCLCPP_WARN(node->get_logger(), "Received unexpected command: %s (expected: %s)",
        recv_cmd.c_str(), exp_cmd.c_str());
      has_command = false;
    }

    if ((node->now() - start_time).seconds() > timeout) {
      RCLCPP_WARN(node->get_logger(), "Timeout waiting for '%s' after %.0fs",
        expected_command.c_str(), timeout);
      return false;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return false;
}
