#include "vision_video_.hpp"

#include <ctime>
#include <filesystem>
#include <sstream>

// ── Constructor ────────────────────────────────────────────────────────────────

VisionVideoNode::VisionVideoNode()
: Node("vision_video_node"),
  fps_window_start_(std::chrono::steady_clock::now())
{
  model_path_   = declare_parameter<std::string>("model_path", "best.onnx");
  input_mode_   = declare_parameter<std::string>("input_mode", "device");
  input_source_ = declare_parameter<std::string>("input_source", "/dev/video0");
  input_width_  = declare_parameter<int>("input_width", 320);
  input_height_ = declare_parameter<int>("input_height", 320);
  letterbox_ = declare_parameter<bool>("letterbox", true);
  camera_width_ = declare_parameter<int>("camera_width", 0);
  camera_height_ = declare_parameter<int>("camera_height", 0);
  camera_fps_ = declare_parameter<int>("camera_fps", 0);
  camera_fourcc_ = declare_parameter<std::string>("camera_fourcc", "");
  use_camera_info_   = declare_parameter<bool>("use_camera_info", false);
  camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/iris_with_camera_2/camera/camera_info");
  conf_threshold_ = declare_parameter<double>("conf_threshold", 0.4);
  nms_threshold_ = declare_parameter<double>("nms_threshold", 0.45);
  show_window_ = declare_parameter<bool>("show_window", true);
  record_video_ = declare_parameter<bool>("record_video", false);
  record_dir_ = declare_parameter<std::string>("record_dir", "video");
  window_name_ = declare_parameter<std::string>("window_name", "vision_video_detection");
  enable_stream_ = declare_parameter<bool>("enable_stream", false);
  stream_host_ = declare_parameter<std::string>("stream_host", "10.7.101.148");
  stream_port_ = declare_parameter<int>("stream_port", 5000);
  stream_width_ = declare_parameter<int>("stream_width", 640);
  stream_height_ = declare_parameter<int>("stream_height", 480);
  stream_fps_ = declare_parameter<int>("stream_fps", 30);
  stream_bitrate_kbps_ = declare_parameter<int>("stream_bitrate_kbps", 500);
  stream_undistort_ = declare_parameter<bool>("stream_undistort", false);
  stream_undistort_alpha_ = declare_parameter<double>("stream_undistort_alpha", 1.0);
  use_openvino_ = declare_parameter<bool>("use_openvino", false);
  openvino_device_ = declare_parameter<std::string>("openvino_device", "CPU");
  openvino_model_path_ = declare_parameter<std::string>("openvino_model_path", "");
  class_names_ = declare_parameter<std::vector<std::string>>("class_names", std::vector<std::string>{});
  object_width_m_ = declare_parameter<double>("object_width_m", 0.3);
  object_height_m_ = declare_parameter<double>("object_height_m", 0.3);
  pose_frame_id_ = declare_parameter<std::string>("pose_frame_id", "camera");
  offset_x_m_ = declare_parameter<double>("offset_x_m", 0.0);
  offset_y_m_ = declare_parameter<double>("offset_y_m", 0.0);
  offset_z_m_ = declare_parameter<double>("offset_z_m", 0.0);
  crosshair_offset_x_px_ = declare_parameter<int>("crosshair_offset_x_px", 0);
  crosshair_offset_y_px_ = declare_parameter<int>("crosshair_offset_y_px", 0);

  if (use_camera_info_) {
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });
    RCLCPP_INFO(get_logger(), "Waiting for camera_info on: %s", camera_info_topic_.c_str());
  } else {
    setupCameraIntrinsics();
  }
  setupObjectPoints();

  cv::setNumThreads(16);
  cv::setUseOptimized(true);

  if (use_openvino_) {
    initializeOpenVINO();
  } else {
    try {
      net_ = cv::dnn::readNetFromONNX(model_path_);
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      RCLCPP_INFO(get_logger(), "ONNX MODEL LOADED: %s", model_path_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "FAILED TO LOAD ONNX MODEL (%s): %s", model_path_.c_str(), e.what());
      throw;
    }
  }

  if (input_mode_ == "topic") {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_source_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });
    RCLCPP_INFO(get_logger(), "Subscribing to image topic: %s", input_source_.c_str());
  } else {
    openInputSource();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&VisionVideoNode::processFrame, this));
  }

  if (show_window_) {
    try {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
      RCLCPP_INFO(get_logger(), "Visualization window enabled: %s", window_name_.c_str());
    } catch (const cv::Exception & e) {
      show_window_ = false;
      RCLCPP_ERROR(get_logger(),
        "Failed to create visualization window (%s). Disabling display. Error: %s",
        window_name_.c_str(), e.what());
    }
  }

  legacy_pose_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/payload_pose", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vision/down/payload", 10);
  RCLCPP_INFO(get_logger(), "Publishing pose to /vision/down/payload and /payload_pose");
}

// ── Destructor ─────────────────────────────────────────────────────────────────

VisionVideoNode::~VisionVideoNode()
{
#if defined(ENABLE_OPENVINO_RUNTIME)
  if (openvino_infer_request_ != nullptr) {
    ov_infer_request_free(openvino_infer_request_);
    openvino_infer_request_ = nullptr;
  }
  if (openvino_compiled_model_ != nullptr) {
    ov_compiled_model_free(openvino_compiled_model_);
    openvino_compiled_model_ = nullptr;
  }
  if (openvino_core_ != nullptr) {
    ov_core_free(openvino_core_);
    openvino_core_ = nullptr;
  }
#endif
  if (stream_writer_.isOpened()) {
    stream_writer_.release();
  }
  if (record_writer_.isOpened()) {
    record_writer_.release();
  }
  if (show_window_) {
    cv::destroyWindow(window_name_);
  }
}

// ── Setup helpers ──────────────────────────────────────────────────────────────

bool VisionVideoNode::startsWith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

void VisionVideoNode::setupCameraIntrinsics()
{
  std::vector<double> cam_data = declare_parameter<std::vector<double>>(
    "camera_matrix",
    {504.99132615, 0.0, 318.4012728,
     0.0, 506.13168687, 243.99035432,
     0.0, 0.0, 1.0});
  std::vector<double> dist_data = declare_parameter<std::vector<double>>(
    "distortion_coeffs",
    {0.07771398, -0.18711237, -0.00182709, 0.00026326, -0.06212367});

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
  if (dist_data.size() >= 5) {
    for (size_t i = 0; i < 5; ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_data[i];
    }
  } else {
    RCLCPP_WARN(get_logger(), "distortion_coeffs must have at least 5 values. Using zeros.");
  }
}

void VisionVideoNode::setupObjectPoints()
{
  const float hw = static_cast<float>(object_width_m_ * 0.5);
  const float hh = static_cast<float>(object_height_m_ * 0.5);
  object_points_ = {
    {-hw,  hh, 0.0f},
    { hw,  hh, 0.0f},
    { hw, -hh, 0.0f},
    {-hw, -hh, 0.0f}
  };
}

void VisionVideoNode::openInputSource()
{
  if (startsWith(input_source_, "/dev/video")) {
    const std::string index_text = input_source_.substr(std::string("/dev/video").size());
    if (!index_text.empty()) {
      try {
        const int camera_index = std::stoi(index_text);
        if (capture_.open(camera_index, cv::CAP_V4L2)) {
          using_camera_device_ = true;
        }
      } catch (const std::exception &) {
      }
    }
  }

  if (!capture_.isOpened()) {
    capture_.open(input_source_);
    using_camera_device_ = startsWith(input_source_, "/dev/video");
  }

  if (!capture_.isOpened()) {
    throw std::runtime_error("failed to open input source: " + input_source_);
  }

  // camera mode: FOURCC first (MJPG is needed for 720p/1080p at 30 fps), then size and fps
  if (using_camera_device_) {
    if (camera_fourcc_.size() == 4) {
      capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(
        camera_fourcc_[0], camera_fourcc_[1], camera_fourcc_[2], camera_fourcc_[3]));
    }
    if (camera_width_ > 0 && camera_height_ > 0) {
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
    }
    if (camera_fps_ > 0) {
      capture_.set(cv::CAP_PROP_FPS, camera_fps_);
    }
    const int code = static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));
    const std::string fourcc{static_cast<char>(code & 255), static_cast<char>((code >> 8) & 255),
                             static_cast<char>((code >> 16) & 255), static_cast<char>((code >> 24) & 255)};
    RCLCPP_INFO(get_logger(), "Camera mode: %.0fx%.0f %s @ %.0f fps (asked %dx%d %s @ %d)",
      capture_.get(cv::CAP_PROP_FRAME_WIDTH), capture_.get(cv::CAP_PROP_FRAME_HEIGHT), fourcc.c_str(),
      capture_.get(cv::CAP_PROP_FPS), camera_width_, camera_height_,
      camera_fourcc_.empty() ? "default" : camera_fourcc_.c_str(), camera_fps_);
  }

  const char * mode = using_camera_device_ ? "camera device" : "video file";
  RCLCPP_INFO(get_logger(), "Using %s input: %s", mode, input_source_.c_str());
}

// ── OpenVINO ───────────────────────────────────────────────────────────────────

void VisionVideoNode::initializeOpenVINO()
{
#if defined(ENABLE_OPENVINO_RUNTIME)
  try {
    const std::string ov_model_path = openvino_model_path_.empty() ? model_path_ : openvino_model_path_;
    ov_status_e status = ov_core_create(&openvino_core_);
    if (status != OK || openvino_core_ == nullptr) {
      throw std::runtime_error("ov_core_create failed: " + getOpenVINOError(status));
    }

    status = ov_core_compile_model_from_file(
      openvino_core_, ov_model_path.c_str(), openvino_device_.c_str(),
      0, &openvino_compiled_model_);
    if (status != OK || openvino_compiled_model_ == nullptr) {
      throw std::runtime_error("ov_core_compile_model_from_file failed: " + getOpenVINOError(status));
    }

    status = ov_compiled_model_create_infer_request(openvino_compiled_model_, &openvino_infer_request_);
    if (status != OK || openvino_infer_request_ == nullptr) {
      throw std::runtime_error("ov_compiled_model_create_infer_request failed: " + getOpenVINOError(status));
    }

    RCLCPP_INFO(get_logger(), "OpenVINO initialized (C API): model=%s device=%s",
      ov_model_path.c_str(), openvino_device_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize OpenVINO runtime: %s", e.what());
    throw;
  }
#else
  throw std::runtime_error(
    "use_openvino=true but this binary was built without OpenVINO. "
    "Rebuild with -DENABLE_OPENVINO=ON.");
#endif
}

#if defined(ENABLE_OPENVINO_RUNTIME)
std::string VisionVideoNode::getOpenVINOError(ov_status_e status) const
{
  std::ostringstream oss;
  oss << ov_get_error_info(status);
  const char * detail = ov_get_last_err_msg();
  if (detail != nullptr && detail[0] != '\0') {
    oss << " | " << detail;
  }
  return oss.str();
}
#endif

// ── Callbacks ──────────────────────────────────────────────────────────────────

void VisionVideoNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (camera_info_received_) {
    return;
  }
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = msg->k[0];
  camera_matrix_.at<double>(0, 2) = msg->k[2];
  camera_matrix_.at<double>(1, 1) = msg->k[4];
  camera_matrix_.at<double>(1, 2) = msg->k[5];

  dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
  for (size_t i = 0; i < std::min(msg->d.size(), size_t(5)); ++i) {
    dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
  }
  camera_info_received_ = true;
  RCLCPP_INFO(get_logger(),
    "Camera intrinsics loaded from topic: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
    msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
}

void VisionVideoNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (use_camera_info_ && !camera_info_received_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for camera_info before processing frames...");
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
      "Unsupported image encoding: %s", enc.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = frame;
    new_frame_ = true;
  }
  processFrame();
}

// ── Pose estimation ────────────────────────────────────────────────────────────

bool VisionVideoNode::estimatePose(const Detection & det, cv::Mat & rvec, cv::Mat & tvec) const
{
  const std::vector<cv::Point2f> image_points = {
    {static_cast<float>(det.box.x),                         static_cast<float>(det.box.y)},
    {static_cast<float>(det.box.x + det.box.width),         static_cast<float>(det.box.y)},
    {static_cast<float>(det.box.x + det.box.width),         static_cast<float>(det.box.y + det.box.height)},
    {static_cast<float>(det.box.x),                         static_cast<float>(det.box.y + det.box.height)}
  };
  return cv::solvePnP(
    object_points_, image_points, camera_matrix_, dist_coeffs_,
    rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
}

void VisionVideoNode::publishLostPose()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = pose_frame_id_;
  msg.pose.orientation.w = 1.0;
  pose_pub_->publish(msg);
}

void VisionVideoNode::publishPose(const cv::Mat & rvec, const cv::Mat & tvec)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = pose_frame_id_;
  const cv::Point2d shift = crosshairShift(tvec.at<double>(2));
  msg.pose.position.x = tvec.at<double>(0) - shift.x + offset_x_m_;
  msg.pose.position.y = tvec.at<double>(1) - shift.y + offset_y_m_;
  msg.pose.position.z = tvec.at<double>(2) + offset_z_m_;

  cv::Mat R;
  cv::Rodrigues(rvec, R);
  const double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    msg.pose.orientation.w = 0.25 / s;
    msg.pose.orientation.x = (R.at<double>(2,1) - R.at<double>(1,2)) * s;
    msg.pose.orientation.y = (R.at<double>(0,2) - R.at<double>(2,0)) * s;
    msg.pose.orientation.z = (R.at<double>(1,0) - R.at<double>(0,1)) * s;
  } else if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
    const double s = 2.0 * std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2));
    msg.pose.orientation.w = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
    msg.pose.orientation.x = 0.25 * s;
    msg.pose.orientation.y = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
    msg.pose.orientation.z = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
  } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
    const double s = 2.0 * std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2));
    msg.pose.orientation.w = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
    msg.pose.orientation.x = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
    msg.pose.orientation.y = 0.25 * s;
    msg.pose.orientation.z = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1));
    msg.pose.orientation.w = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
    msg.pose.orientation.x = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
    msg.pose.orientation.y = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
    msg.pose.orientation.z = 0.25 * s;
  }

  pose_pub_->publish(msg);
}

void VisionVideoNode::publishLegacyPose(const cv::Mat & frame, const Detection & det)
{
  const float frame_center_x = frame.cols / 2.0f;
  const float frame_center_y = frame.rows / 2.0f;
  const float box_center_x = det.box.x + det.box.width / 2.0f;
  const float box_center_y = det.box.y + det.box.height / 2.0f;
  const float dx = box_center_x - frame_center_x;
  const float dy = box_center_y - frame_center_y;
  auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  msg->data.resize(3);
  msg->data[0] = box_center_x;
  msg->data[1] = box_center_y;
  msg->data[2] = std::sqrt(dx * dx + dy * dy);
  legacy_pose_pub_->publish(std::move(msg));
}

// ── Raw recording (record_video) ──────────────────────────────────────────────
// Saves the frame exactly as grabbed (no boxes, no crosshair) — for datasets / review.
void VisionVideoNode::recordFrame(const cv::Mat & frame)
{
  if (!record_video_ || frame.empty()) return;
  if (!record_writer_.isOpened()) {
    if (record_failed_) return;
    std::error_code ec;
    std::filesystem::create_directories(record_dir_, ec);
    char stamp[32];
    const std::time_t t = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
    const std::string path =
      (std::filesystem::absolute(record_dir_) / ("vision_video_" + std::string(stamp) + ".mp4")).string();
    const double fps = camera_fps_ > 0 ? camera_fps_ : 30.0;
    // H.264 .mp4 through GStreamer (our OpenCV has no FFmpeg); the mp4 header is rewritten
    // every second (room for 2 h), so the file stays playable even if the node is killed
    std::ostringstream pipeline;
    pipeline
      << "appsrc ! videoconvert ! video/x-raw,format=I420 ! "
      << "x264enc speed-preset=superfast pass=quant quantizer=18 key-int-max=" << static_cast<int>(fps) << " ! "
      << "h264parse ! mp4mux reserved-max-duration=7200000000000 reserved-moov-update-period=1000000000 ! "
      << "filesink location=\"" << path << "\"";
    if (!record_writer_.open(pipeline.str(), cv::CAP_GSTREAMER, 0, fps, frame.size(), true)) {
      RCLCPP_ERROR(get_logger(), "record_video: cannot open %s — recording off", path.c_str());
      record_failed_ = true;
      return;
    }
    RCLCPP_INFO(get_logger(), "record_video: raw frames -> %s (%dx%d @ %.0f fps)",
      path.c_str(), frame.cols, frame.rows, fps);
  }
  record_writer_.write(frame);
}

// ── Display & stream ───────────────────────────────────────────────────────────

void VisionVideoNode::updateFps()
{
  ++frames_in_window_;
  const auto now = std::chrono::steady_clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - fps_window_start_).count();
  if (elapsed >= 1.0) {
    fps_ = static_cast<double>(frames_in_window_) / elapsed;
    fps_window_start_ = now;
    frames_in_window_ = 0;
  }
}

std::string VisionVideoNode::className(int class_id) const
{
  if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
    return class_names_[static_cast<size_t>(class_id)];
  }
  return "class=" + std::to_string(class_id);
}

void VisionVideoNode::drawDetections(cv::Mat & frame, const std::vector<Detection> & detections) const
{
  for (const auto & det : detections) {
    cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
    const std::string label =
      cv::format("%s %.2f", className(det.class_id).c_str(), static_cast<double>(det.confidence));
    int baseline = 0;
    const cv::Size text_size =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    const int top = std::max(det.box.y, text_size.height + 4);
    cv::rectangle(
      frame,
      cv::Point(det.box.x, top - text_size.height - 4),
      cv::Point(det.box.x + text_size.width, top + baseline - 4),
      cv::Scalar(0, 255, 0), cv::FILLED);
    cv::putText(
      frame, label, cv::Point(det.box.x, top - 6),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }
}

void VisionVideoNode::drawCrosshair(cv::Mat & frame) const
{
  int center_x = std::clamp(frame.cols / 2 + crosshair_offset_x_px_, 0, frame.cols - 1);
  int center_y = std::clamp(frame.rows / 2 + crosshair_offset_y_px_, 0, frame.rows - 1);
  cv::Scalar green(0, 255, 0);
  cv::line(frame, cv::Point(0, center_y), cv::Point(frame.cols, center_y), green, 1);
  cv::line(frame, cv::Point(center_x, 0), cv::Point(center_x, frame.rows), green, 1);
}

// Crosshair offset in metres at depth z: the target pose is measured from the drawn crosshair
// (not the image centre), so centering puts the target on the crosshair. 0 px offset = no change.
cv::Point2d VisionVideoNode::crosshairShift(double z) const
{
  if (frame_size_.empty() || (crosshair_offset_x_px_ == 0 && crosshair_offset_y_px_ == 0)) return {0.0, 0.0};
  const cv::Point2f c(frame_size_.width / 2.0f, frame_size_.height / 2.0f);
  const std::vector<cv::Point2f> px = {c, {c.x + crosshair_offset_x_px_, c.y + crosshair_offset_y_px_}};
  std::vector<cv::Point2f> n;
  cv::undistortPoints(px, n, camera_matrix_, dist_coeffs_);
  return {(n[1].x - n[0].x) * z, (n[1].y - n[0].y) * z};
}

void VisionVideoNode::displayFrame(cv::Mat & frame, const std::vector<Detection> & detections)
{
  if (!show_window_) {
    return;
  }
  try {
    drawCrosshair(frame);
    drawDetections(frame, detections);
    cv::imshow(window_name_, frame);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      RCLCPP_INFO(get_logger(), "Window exit key pressed, shutting down node");
      rclcpp::shutdown();
    }
  } catch (const cv::Exception & e) {
    show_window_ = false;
    RCLCPP_ERROR(get_logger(),
      "Visualization failed at runtime, disabling display. Error: %s", e.what());
  }
}

bool VisionVideoNode::ensureStreamWriter()
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

void VisionVideoNode::streamFrame(const cv::Mat & frame, const std::vector<Detection> & detections)
{
  if (!ensureStreamWriter()) {
    return;
  }
  // send at most stream_fps frames per second (the stream is only for monitoring)
  const auto t = std::chrono::steady_clock::now();
  if (t - last_stream_write_ < std::chrono::duration<double>(1.0 / std::max(1, stream_fps_))) {
    return;
  }
  last_stream_write_ = t;
  // draw at the source size (boxes are in source pixels), then scale to the stream size
  cv::Mat annotated = frame.clone();
  drawDetections(annotated, detections);            // boxes first: they bend with the picture and stay on their objects
  if (stream_undistort_) {
    undistortForStream(annotated);
  }
  drawCrosshair(annotated);                 // after: stays straight
  cv::Mat out;
  if (annotated.cols != stream_width_ || annotated.rows != stream_height_) {
    cv::resize(annotated, out, cv::Size(stream_width_, stream_height_));
  } else {
    out = annotated;
  }
  stream_writer_.write(out);
}

// ── Core loop ──────────────────────────────────────────────────────────────────

void VisionVideoNode::processFrame()
{
  cv::Mat frame;
  if (input_mode_ == "topic") {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!new_frame_ || latest_frame_.empty()) {
      return;
    }
    frame = latest_frame_.clone();
    new_frame_ = false;
  } else {
    if (!capture_.read(frame) || frame.empty()) {
      if (using_camera_device_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "Failed to read frame from camera: %s", input_source_.c_str());
        return;
      }
      capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!capture_.read(frame) || frame.empty()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "Failed to read frame from video file: %s", input_source_.c_str());
        return;
      }
    }
  }

  recordFrame(frame);   // raw, before anything is drawn
  frame_size_ = frame.size();
  std::vector<Detection> detections = infer(frame);
  updateFps();
  streamFrame(frame, detections);
  displayFrame(frame, detections);

  if (!detections.empty()) {
    const Detection & best = *std::max_element(
      detections.begin(), detections.end(),
      [](const Detection & a, const Detection & b) { return a.confidence < b.confidence; });

    publishLegacyPose(frame, best);

    cv::Mat rvec, tvec;
    if (estimatePose(best, rvec, tvec)) {
      publishPose(rvec, tvec);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 250,
        "FPS=%.2f | %zu obj, best[%s conf=%.2f] pose(x=%.3f y=%.3f z=%.3f) m",
        fps_, detections.size(), className(best.class_id).c_str(), best.confidence,
        tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    } else {
      publishLostPose();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "FPS=%.2f | %zu obj detected but solvePnP failed", fps_, detections.size());
    }
  } else {
    publishLostPose();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "FPS=%.2f | No object detected", fps_);
  }
}

// ── Inference ──────────────────────────────────────────────────────────────────

std::vector<VisionVideoNode::Detection> VisionVideoNode::infer(const cv::Mat & frame)
{
  std::vector<Detection> detections;

  // letterbox: keep the aspect ratio (a 16:9 frame is not squashed into the model input)
  cv::Mat model_in = frame;
  lb_scale_ = 1.0f; lb_pad_x_ = 0; lb_pad_y_ = 0;
  if (letterbox_) {
    lb_scale_ = std::min(static_cast<float>(input_width_) / frame.cols,
                         static_cast<float>(input_height_) / frame.rows);
    const int new_w = static_cast<int>(std::round(frame.cols * lb_scale_));
    const int new_h = static_cast<int>(std::round(frame.rows * lb_scale_));
    lb_pad_x_ = (input_width_ - new_w) / 2;
    lb_pad_y_ = (input_height_ - new_h) / 2;
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(new_w, new_h));
    model_in = cv::Mat(input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(model_in(cv::Rect(lb_pad_x_, lb_pad_y_, new_w, new_h)));
  }
  cv::Mat blob;
  cv::dnn::blobFromImage(
    model_in, blob, 1.0 / 255.0, cv::Size(input_width_, input_height_),
    cv::Scalar(), true, false);
  cv::Mat out;

  if (use_openvino_) {
#if defined(ENABLE_OPENVINO_RUNTIME)
    ov_tensor_t * input_tensor = nullptr;
    ov_tensor_t * output_tensor = nullptr;
    ov_shape_t output_shape{};
    bool has_output_shape = false;
    auto cleanup = [&]() {
      if (has_output_shape) {
        ov_shape_free(&output_shape);
      }
      if (output_tensor != nullptr) {
        ov_tensor_free(output_tensor);
      }
      if (input_tensor != nullptr) {
        ov_tensor_free(input_tensor);
      }
    };

    try {
      ov_status_e status = ov_infer_request_get_input_tensor(openvino_infer_request_, &input_tensor);
      if (status != OK || input_tensor == nullptr) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO input tensor fetch failed: %s", getOpenVINOError(status).c_str());
        cleanup();
        return detections;
      }

      size_t input_count = 0;
      status = ov_tensor_get_size(input_tensor, &input_count);
      if (status != OK) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO input tensor size read failed: %s", getOpenVINOError(status).c_str());
        cleanup();
        return detections;
      }

      const size_t blob_count = blob.total();
      if (blob_count != input_count) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO input size mismatch. Blob=%zu Tensor=%zu", blob_count, input_count);
        cleanup();
        return detections;
      }

      void * input_ptr = nullptr;
      status = ov_tensor_data(input_tensor, &input_ptr);
      if (status != OK || input_ptr == nullptr) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO input tensor data access failed: %s", getOpenVINOError(status).c_str());
        cleanup();
        return detections;
      }

      std::memcpy(
        static_cast<float *>(input_ptr), blob.ptr<float>(),
        blob_count * sizeof(float));
      status = ov_infer_request_infer(openvino_infer_request_);
      if (status != OK) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO infer failed: %s", getOpenVINOError(status).c_str());
        cleanup();
        return detections;
      }

      status = ov_infer_request_get_output_tensor(openvino_infer_request_, &output_tensor);
      if (status != OK || output_tensor == nullptr) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO output tensor fetch failed: %s", getOpenVINOError(status).c_str());
        cleanup();
        return detections;
      }

      status = ov_tensor_get_shape(output_tensor, &output_shape);
      if (status != OK) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO output shape read failed: %s", getOpenVINOError(status).c_str());
        cleanup();
        return detections;
      }
      has_output_shape = true;

      void * output_ptr = nullptr;
      status = ov_tensor_data(output_tensor, &output_ptr);
      if (status != OK || output_ptr == nullptr) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "OpenVINO output tensor data access failed: %s", getOpenVINOError(status).c_str());
        cleanup();
        return detections;
      }

      if (output_shape.rank == 3) {
        int dims[3] = {
          static_cast<int>(output_shape.dims[0]),
          static_cast<int>(output_shape.dims[1]),
          static_cast<int>(output_shape.dims[2])};
        cv::Mat out_view(3, dims, CV_32F, output_ptr);
        out = out_view.clone();
      } else if (output_shape.rank == 4 && output_shape.dims[0] == 1) {
        int dims[3] = {
          static_cast<int>(output_shape.dims[1]),
          static_cast<int>(output_shape.dims[2]),
          static_cast<int>(output_shape.dims[3])};
        cv::Mat out_view(3, dims, CV_32F, output_ptr);
        out = out_view.clone();
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "Unsupported OpenVINO output shape rank=%ld", output_shape.rank);
        cleanup();
        return detections;
      }
      cleanup();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "OpenVINO inference failed: %s", e.what());
      cleanup();
      return detections;
    }
#else
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "use_openvino=true but binary built without OpenVINO runtime");
    return detections;
#endif
  } else {
    try {
      net_.setInput(blob);
      out = net_.forward();
    } catch (const cv::Exception & primary_error) {
      try {
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        if (outputs.empty()) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "DNN forward returned no outputs");
          return detections;
        }
        out = outputs[0];
      } catch (const cv::Exception & fallback_error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "DNN forward failed (OpenCV may not support this ONNX graph). "
          "Primary: %s | Fallback: %s",
          primary_error.what(), fallback_error.what());
        return detections;
      }
    }
  }

  if (out.dims != 3) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Unexpected output dims: %d", out.dims);
    return detections;
  }

  const int dim1 = out.size[1];
  const int dim2 = out.size[2];
  const bool channels_first = (dim1 < dim2);
  const int num_candidates = channels_first ? dim2 : dim1;
  const int num_features = channels_first ? dim1 : dim2;

  if (num_features < 5) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Invalid YOLO features: %d", num_features);
    return detections;
  }

  const int num_classes = num_features - 4;
  // model input -> frame pixels (letterbox: remove the padding, undo the scale)
  const float x_scale = letterbox_ ? 1.0f / lb_scale_
                                   : static_cast<float>(frame.cols) / static_cast<float>(input_width_);
  const float y_scale = letterbox_ ? 1.0f / lb_scale_
                                   : static_cast<float>(frame.rows) / static_cast<float>(input_height_);
  const float x_pad = letterbox_ ? static_cast<float>(lb_pad_x_) : 0.0f;
  const float y_pad = letterbox_ ? static_cast<float>(lb_pad_y_) : 0.0f;

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for (int i = 0; i < num_candidates; ++i) {
    float cx, cy, w, h;
    if (channels_first) {
      cx = out.at<float>(0, 0, i);
      cy = out.at<float>(0, 1, i);
      w  = out.at<float>(0, 2, i);
      h  = out.at<float>(0, 3, i);
    } else {
      cx = out.at<float>(0, i, 0);
      cy = out.at<float>(0, i, 1);
      w  = out.at<float>(0, i, 2);
      h  = out.at<float>(0, i, 3);
    }

    int best_class = -1;
    float best_score = 0.0f;
    for (int c = 0; c < num_classes; ++c) {
      float score = channels_first ? out.at<float>(0, c + 4, i) : out.at<float>(0, i, c + 4);
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }

    if (best_score < static_cast<float>(conf_threshold_)) {
      continue;
    }

    int left   = static_cast<int>((cx - 0.5f * w - x_pad) * x_scale);
    int top    = static_cast<int>((cy - 0.5f * h - y_pad) * y_scale);
    int width  = static_cast<int>(w * x_scale);
    int height = static_cast<int>(h * y_scale);
    cv::Rect box(left, top, width, height);
    box &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (box.width <= 0 || box.height <= 0) {
      continue;
    }

    class_ids.push_back(best_class);
    confidences.push_back(best_score);
    boxes.push_back(box);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(
    boxes, confidences,
    static_cast<float>(conf_threshold_),
    static_cast<float>(nms_threshold_),
    indices);

  detections.reserve(indices.size());
  for (int idx : indices) {
    detections.push_back(Detection{class_ids[idx], confidences[idx], boxes[idx]});
  }
  return detections;
}

// Straighten the stream copy with the calibration (camera_matrix / distortion_coeffs).
// Only for viewing: detection, centering and GPS keep using the raw (distorted) image.
void VisionVideoNode::undistortForStream(cv::Mat & img)
{
  if (camera_matrix_.empty() || dist_coeffs_.empty() || cv::countNonZero(dist_coeffs_) == 0) {
    return;
  }
  if (stream_map1_.empty() || stream_map_size_ != img.size()) {
    const cv::Mat new_k = cv::getOptimalNewCameraMatrix(
      camera_matrix_, dist_coeffs_, img.size(), stream_undistort_alpha_, img.size());
    cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat(), new_k, img.size(),
      CV_16SC2, stream_map1_, stream_map2_);
    stream_map_size_ = img.size();
    RCLCPP_INFO(get_logger(), "Stream undistort enabled (%dx%d, alpha %.2f)",
      img.cols, img.rows, stream_undistort_alpha_);
  }
  cv::Mat out;
  cv::remap(img, out, stream_map1_, stream_map2_, cv::INTER_LINEAR);
  img = out;
}
