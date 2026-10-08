#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>

class VisionArTagNode : public rclcpp::Node
{
public:
  VisionArTagNode()
  : Node("vision_artag_node"),
    fps_window_start_(std::chrono::steady_clock::now())
  {
    input_mode_   = declare_parameter<std::string>("input_mode", "device");
    input_source_ = declare_parameter<std::string>("input_source", "/dev/video4");
    input_width_  = declare_parameter<int>("input_width", 640);
    input_height_ = declare_parameter<int>("input_height", 480);
    use_camera_info_  = declare_parameter<bool>("use_camera_info", false);
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic",
      "/iris_with_camera_2/camera/camera_info");
    show_window_ = declare_parameter<bool>("show_window", true);
    window_name_ = declare_parameter<std::string>("window_name", "vision_artag");
    enable_stream_ = declare_parameter<bool>("enable_stream", false);
    stream_host_ = declare_parameter<std::string>("stream_host", "10.7.101.148");
    stream_port_ = declare_parameter<int>("stream_port", 5000);
    stream_width_ = declare_parameter<int>("stream_width", 640);
    stream_height_ = declare_parameter<int>("stream_height", 480);
    stream_fps_ = declare_parameter<int>("stream_fps", 30);
    stream_bitrate_kbps_ = declare_parameter<int>("stream_bitrate_kbps", 500);
    target_tag_id_ = declare_parameter<int>("tag_id", -1);
    tag_size_m_ = declare_parameter<double>("tag_size_m", 0.2);
    pose_frame_id_ = declare_parameter<std::string>("pose_frame_id", "camera");
    offset_x_m_ = declare_parameter<double>("offset_x_m", 0.0);
    offset_y_m_ = declare_parameter<double>("offset_y_m", 0.0);
    offset_z_m_ = declare_parameter<double>("offset_z_m", 0.0);
    min_area_ratio_ = declare_parameter<double>("filter.min_area_ratio", 0.0006);
    max_area_ratio_ = declare_parameter<double>("filter.max_area_ratio", 0.8);
    max_squareness_ratio_ = declare_parameter<double>("filter.max_squareness_ratio", 1.4);
    min_border_margin_px_ = declare_parameter<int>("filter.min_border_px", 6);
    stable_frames_required_ = declare_parameter<int>("filter.stable_frames", 1);
    max_center_jump_px_ = declare_parameter<double>("filter.max_center_jump_px", 100.0);

    cv::setNumThreads(16);
    cv::setUseOptimized(true);

    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->adaptiveThreshWinSizeMin =
      declare_parameter<int>("aruco.adaptive_thresh_win_size_min", 3);
    detector_params_->adaptiveThreshWinSizeMax =
      declare_parameter<int>("aruco.adaptive_thresh_win_size_max", 53);
    detector_params_->adaptiveThreshWinSizeStep =
      declare_parameter<int>("aruco.adaptive_thresh_win_size_step", 10);
    detector_params_->adaptiveThreshConstant =
      declare_parameter<double>("aruco.adaptive_thresh_constant", 7.0);
    detector_params_->minMarkerPerimeterRate =
      declare_parameter<double>("aruco.min_marker_perimeter_rate", 0.02);
    detector_params_->maxMarkerPerimeterRate =
      declare_parameter<double>("aruco.max_marker_perimeter_rate", 4.0);
    detector_params_->polygonalApproxAccuracyRate =
      declare_parameter<double>("aruco.polygonal_approx_accuracy_rate", 0.05);
    detector_params_->minCornerDistanceRate =
      declare_parameter<double>("aruco.min_corner_distance_rate", 0.05);
    detector_params_->minMarkerDistanceRate =
      declare_parameter<double>("aruco.min_marker_distance_rate", 0.05);
    detector_params_->minDistanceToBorder =
      declare_parameter<int>("aruco.min_distance_to_border", 3);
    detector_params_->detectInvertedMarker =
      declare_parameter<bool>("aruco.detect_inverted_marker", true);
    detector_params_->errorCorrectionRate =
      declare_parameter<double>("aruco.error_correction_rate", 0.6);
    const int corner_refine_method =
      declare_parameter<int>("aruco.corner_refine_method",
        static_cast<int>(cv::aruco::CORNER_REFINE_SUBPIX));
    detector_params_->cornerRefinementMethod = corner_refine_method;
    detector_params_->cornerRefinementWinSize =
      declare_parameter<int>("aruco.corner_refine_win_size", 5);
    detector_params_->cornerRefinementMaxIterations =
      declare_parameter<int>("aruco.corner_refine_max_iterations", 30);
    detector_params_->cornerRefinementMinAccuracy =
      declare_parameter<double>("aruco.corner_refine_min_accuracy", 0.1);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

    setupObjectPoints();

    if (use_camera_info_) {
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });
      RCLCPP_INFO(get_logger(), "Waiting for camera_info on: %s", camera_info_topic_.c_str());
    } else {
      setupCameraIntrinsics();
    }

    if (input_mode_ == "topic") {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_source_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });
      RCLCPP_INFO(get_logger(), "Subscribing to image topic: %s", input_source_.c_str());
    } else {
      openInputSource();
      timer_ = create_wall_timer(
        std::chrono::milliseconds(1), std::bind(&VisionArTagNode::processFrame, this));
    }

    if (show_window_) {
      try {
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        RCLCPP_INFO(get_logger(), "Visualization window enabled: %s", window_name_.c_str());
      } catch (const cv::Exception & e) {
        show_window_ = false;
        RCLCPP_ERROR(
          get_logger(),
          "Failed to create visualization window (%s). Disabling display. Error: %s",
          window_name_.c_str(), e.what());
      }
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vision/down/artag", 10);
    RCLCPP_INFO(get_logger(), "Publishing ArTag pose to /vision/down/artag");
  }

  ~VisionArTagNode() override
  {
    if (stream_writer_.isOpened()) {
      stream_writer_.release();
    }
    if (show_window_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  struct TagDetection
  {
    int id;
    std::array<cv::Point2f, 4> corners;
    cv::Point2f center;
    double center_dist = 0.0;
    double area = 0.0;
  };

  static bool startsWith(const std::string & value, const std::string & prefix)
  {
    return value.rfind(prefix, 0) == 0;
  }

  bool passesGeometryFilters(const TagDetection & det, const cv::Size & frame_size) const
  {
    const double frame_area = static_cast<double>(frame_size.area());
    if (frame_area <= 0.0) {
      return false;
    }

    const double area_ratio = det.area / frame_area;
    if (min_area_ratio_ > 0.0 && area_ratio < min_area_ratio_) {
      return false;
    }
    if (max_area_ratio_ > 0.0 && area_ratio > max_area_ratio_) {
      return false;
    }

    if (max_squareness_ratio_ > 1.0) {
      std::array<double, 4> sides = {
        std::hypot(det.corners[1].x - det.corners[0].x, det.corners[1].y - det.corners[0].y),
        std::hypot(det.corners[2].x - det.corners[1].x, det.corners[2].y - det.corners[1].y),
        std::hypot(det.corners[3].x - det.corners[2].x, det.corners[3].y - det.corners[2].y),
        std::hypot(det.corners[0].x - det.corners[3].x, det.corners[0].y - det.corners[3].y)
      };
      const auto minmax = std::minmax_element(sides.begin(), sides.end());
      if (*minmax.first <= 1e-3) {
        return false;
      }
      const double ratio = *minmax.second / *minmax.first;
      if (ratio > max_squareness_ratio_) {
        return false;
      }
    }

    if (min_border_margin_px_ > 0) {
      const double width = static_cast<double>(frame_size.width);
      const double height = static_cast<double>(frame_size.height);
      double min_border = std::numeric_limits<double>::max();
      for (const auto & corner : det.corners) {
        const double cx = static_cast<double>(corner.x);
        const double cy = static_cast<double>(corner.y);
        const double corner_border = std::min(
          std::min(cx, cy),
          std::min(width - cx, height - cy));
        min_border = std::min(min_border, corner_border);
      }
      if (min_border < static_cast<double>(min_border_margin_px_)) {
        return false;
      }
    }

    return true;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) {
      return;
    }
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->k[0];   // fx
    camera_matrix_.at<double>(0, 2) = msg->k[2];   // cx
    camera_matrix_.at<double>(1, 1) = msg->k[4];   // fy
    camera_matrix_.at<double>(1, 2) = msg->k[5];   // cy

    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    for (size_t i = 0; i < std::min(msg->d.size(), size_t(5)); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }
    camera_info_received_ = true;
    RCLCPP_INFO(get_logger(),
      "Camera intrinsics loaded from topic: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
      msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (use_camera_info_ && !camera_info_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for camera_info before processing frames...");
      return;
    }
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = cv_ptr->image;
      new_frame_ = true;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "cv_bridge exception: %s", e.what());
      return;
    }
    processFrame();
  }

  void setupCameraIntrinsics()
  {
    std::vector<double> camera_matrix_data =
      declare_parameter<std::vector<double>>(
        "camera_matrix",
        {
          504.99132615, 0.0, 318.4012728,
          0.0, 506.13168687, 243.99035432,
          0.0, 0.0, 1.0
        });
    std::vector<double> dist_coeffs_data =
      declare_parameter<std::vector<double>>(
        "distortion_coeffs",
        {0.07771398, -0.18711237, -0.00182709, 0.00026326, -0.06212367});

    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    if (camera_matrix_data.size() == 9) {
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          camera_matrix_.at<double>(r, c) = camera_matrix_data[static_cast<size_t>(r * 3 + c)];
        }
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "camera_matrix must have 9 values. Using identity matrix instead.");
    }

    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    if (dist_coeffs_data.size() >= 5) {
      for (size_t i = 0; i < 5; ++i) {
        dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_coeffs_data[i];
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "distortion_coeffs must have at least 5 values. Using zeros.");
    }
  }

  void setupObjectPoints()
  {
    if (tag_size_m_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "tag_size_m must be > 0. Using default 0.2m.");
      tag_size_m_ = 0.2;
    }
    const float half = static_cast<float>(tag_size_m_ * 0.5);
    object_points_ = {
      {-half,  half, 0.0f},
      { half,  half, 0.0f},
      { half, -half, 0.0f},
      {-half, -half, 0.0f}
    };
  }

  void openInputSource()
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

    if (using_camera_device_) {
      if (input_width_ > 0) {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, input_width_);
      }
      if (input_height_ > 0) {
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, input_height_);
      }
    }

    const char * mode = using_camera_device_ ? "camera device" : "video file";
    RCLCPP_INFO(get_logger(), "Using %s input: %s", mode, input_source_.c_str());
  }

  void updateFps()
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

  void drawCrosshair(cv::Mat & frame) const
  {
    const int center_x = frame.cols / 2;
    const int center_y = frame.rows / 2;
    const cv::Scalar green(0, 255, 0);
    const int thickness = 1;
    cv::line(frame, cv::Point(0, center_y), cv::Point(frame.cols, center_y), green, thickness);
    cv::line(frame, cv::Point(center_x, 0), cv::Point(center_x, frame.rows), green, thickness);
  }

  void drawDetections(cv::Mat & frame, const std::vector<TagDetection> & detections) const
  {
    for (const auto & det : detections) {
      std::array<cv::Point, 4> pts;
      for (size_t i = 0; i < det.corners.size(); ++i) {
        pts[i] = cv::Point(static_cast<int>(det.corners[i].x), static_cast<int>(det.corners[i].y));
      }
      const cv::Point * pts_ptr = pts.data();
      int num_points = 4;
      cv::polylines(frame, &pts_ptr, &num_points, 1, true, cv::Scalar(0, 255, 0), 2);
      cv::circle(frame, det.center, 3, cv::Scalar(0, 255, 255), cv::FILLED);

      const std::string label = "ARUCO:" + std::to_string(det.id);
      int baseline = 0;
      const cv::Size text_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      const int top = std::max(0, static_cast<int>(det.corners[0].y) - text_size.height - 6);
      cv::rectangle(
        frame,
        cv::Point(static_cast<int>(det.corners[0].x), top),
        cv::Point(static_cast<int>(det.corners[0].x) + text_size.width, top + text_size.height + 4),
        cv::Scalar(0, 255, 0), cv::FILLED);
      cv::putText(
        frame, label, cv::Point(static_cast<int>(det.corners[0].x), top + text_size.height),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
  }

  bool ensureStreamWriter()
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
      RCLCPP_ERROR(
        get_logger(),
        "Failed to open GStreamer stream pipeline. Disabling stream. Pipeline: %s",
        pipeline.str().c_str());
      enable_stream_ = false;
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Streaming enabled to udp://%s:%d (%dx%d @ %d fps)",
      stream_host_.c_str(), stream_port_, stream_width_, stream_height_, stream_fps_);
    return true;
  }

  void streamFrame(const cv::Mat & frame, const std::vector<TagDetection> & detections)
  {
    if (!ensureStreamWriter()) {
      return;
    }

    cv::Mat out;
    if (frame.cols != stream_width_ || frame.rows != stream_height_) {
      cv::resize(frame, out, cv::Size(stream_width_, stream_height_));
    } else {
      out = frame.clone();
    }
    drawCrosshair(out);
    drawDetections(out, detections);
    stream_writer_.write(out);
  }

  std::vector<TagDetection> detectTags(const cv::Mat & frame)
  {
    std::vector<TagDetection> detections;
    if (frame.empty()) {
      return detections;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = frame;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

    const cv::Point2f frame_center(frame.cols / 2.0f, frame.rows / 2.0f);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (corners[i].size() != 4) {
        continue;
      }
      if (target_tag_id_ >= 0 && ids[i] != target_tag_id_) {
        continue;
      }

      TagDetection det;
      det.id = ids[i];
      for (size_t j = 0; j < 4; ++j) {
        det.corners[j] = corners[i][j];
      }

      det.center = cv::Point2f(
        (det.corners[0].x + det.corners[1].x + det.corners[2].x + det.corners[3].x) / 4.0f,
        (det.corners[0].y + det.corners[1].y + det.corners[2].y + det.corners[3].y) / 4.0f);

      det.center_dist = std::hypot(det.center.x - frame_center.x, det.center.y - frame_center.y);
      det.area = std::fabs(cv::contourArea(corners[i]));
      if (!passesGeometryFilters(det, frame.size())) {
        continue;
      }
      detections.push_back(det);
    }

    return detections;
  }

  bool estimatePose(const TagDetection & det, cv::Mat & rvec, cv::Mat & tvec) const
  {
    std::vector<cv::Point2f> image_points;
    image_points.reserve(4);
    for (const auto & corner : det.corners) {
      image_points.push_back(corner);
    }

    bool ok = false;
    try {
      ok = cv::solvePnP(
        object_points_,
        image_points,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE);
    } catch (const cv::Exception &) {
      // degenerate corner geometry (e.g. near-edge-on tag) — skip this frame
    }
    return ok;
  }

  void publishLostPose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = pose_frame_id_;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 1.0;
    pose_pub_->publish(msg);
  }

  void publishPose(const cv::Mat & rvec, const cv::Mat & tvec)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = pose_frame_id_;
    msg.pose.position.x = tvec.at<double>(0) + offset_x_m_;
    msg.pose.position.y = tvec.at<double>(1) + offset_y_m_;
    msg.pose.position.z = tvec.at<double>(2) + offset_z_m_;

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      msg.pose.orientation.w = 0.25 / s;
      msg.pose.orientation.x = (R.at<double>(2,1) - R.at<double>(1,2)) * s;
      msg.pose.orientation.y = (R.at<double>(0,2) - R.at<double>(2,0)) * s;
      msg.pose.orientation.z = (R.at<double>(1,0) - R.at<double>(0,1)) * s;
    } else {
      if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2));
        msg.pose.orientation.w = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
        msg.pose.orientation.x = 0.25 * s;
        msg.pose.orientation.y = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
        msg.pose.orientation.z = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
      } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2));
        msg.pose.orientation.w = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
        msg.pose.orientation.x = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
        msg.pose.orientation.y = 0.25 * s;
        msg.pose.orientation.z = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
      } else {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1));
        msg.pose.orientation.w = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
        msg.pose.orientation.x = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
        msg.pose.orientation.y = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
        msg.pose.orientation.z = 0.25 * s;
      }
    }

    pose_pub_->publish(msg);
  }

  void processFrame()
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
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "Failed to read frame from camera: %s",
            input_source_.c_str());
          return;
        }
        capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
        if (!capture_.read(frame) || frame.empty()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "Failed to read frame from video file: %s",
            input_source_.c_str());
          return;
        }
      }
    }

    std::vector<TagDetection> detections = detectTags(frame);
    updateFps();
    streamFrame(frame, detections);

    if (show_window_) {
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
        RCLCPP_ERROR(
          get_logger(),
          "Visualization failed at runtime, disabling display. Error: %s",
          e.what());
      }
    }

    if (!detections.empty()) {
      const TagDetection * best = nullptr;
      for (const auto & det : detections) {
        if (best == nullptr) {
          best = &det;
          continue;
        }
        if (det.center_dist < best->center_dist - 1e-3) {
          best = &det;
          continue;
        }
        if (std::abs(det.center_dist - best->center_dist) <= 1e-3 && det.area > best->area) {
          best = &det;
        }
      }

      if (best != nullptr) {
        bool stable_match = false;
        if (have_last_best_) {
          const bool same_id = best->id == last_best_id_;
          const double jump = std::hypot(
            best->center.x - last_best_center_.x, best->center.y - last_best_center_.y);
          stable_match = same_id && (max_center_jump_px_ <= 0.0 || jump <= max_center_jump_px_);
        }

        if (!stable_match) {
          stable_count_ = 1;
        } else {
          stable_count_++;
        }
        last_best_id_ = best->id;
        last_best_center_ = best->center;
        have_last_best_ = true;

        const bool ready =
          stable_frames_required_ <= 1 || stable_count_ >= stable_frames_required_;
        if (ready) {
          cv::Mat rvec, tvec;
          if (estimatePose(*best, rvec, tvec)) {
            publishPose(rvec, tvec);
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 250,
              "FPS=%.2f | ArTag id=%d pose(x=%.3f y=%.3f z=%.3f) m",
              fps_, best->id, tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
          } else {
            publishLostPose();
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 500,
              "Pose solve failed for tag id=%d, publishing zero pose", best->id);
          }
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 250,
            "FPS=%.2f | Candidate tag id=%d waiting for stability (%d/%d)",
            fps_, best->id, stable_count_, stable_frames_required_);
        }
      }
    } else {
      stable_count_ = 0;
      have_last_best_ = false;
      publishLostPose();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "FPS=%.2f | No tag detected", fps_);
    }
  }

  std::string input_mode_;
  std::string input_source_;
  int input_width_ = 0;
  int input_height_ = 0;
  bool show_window_ = true;
  std::string window_name_;
  bool enable_stream_ = false;
  std::string stream_host_;
  int stream_port_ = 0;
  int stream_width_ = 0;
  int stream_height_ = 0;
  int stream_fps_ = 0;
  int stream_bitrate_kbps_ = 0;
  int target_tag_id_ = -1;
  double tag_size_m_ = 0.0;
  std::string pose_frame_id_;
  double offset_x_m_ = 0.0;
  double offset_y_m_ = 0.0;
  double offset_z_m_ = 0.0;
  double min_area_ratio_ = 0.0;
  double max_area_ratio_ = 0.0;
  double max_squareness_ratio_ = 0.0;
  int min_border_margin_px_ = 0;
  int stable_frames_required_ = 1;
  double max_center_jump_px_ = 0.0;
  bool use_camera_info_ = false;
  std::string camera_info_topic_;
  bool camera_info_received_ = false;

  cv::VideoCapture capture_;
  bool using_camera_device_ = false;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::vector<cv::Point3f> object_points_;

  std::chrono::steady_clock::time_point fps_window_start_;
  int frames_in_window_ = 0;
  double fps_ = 0.0;

  bool stream_writer_initialized_ = false;
  cv::VideoWriter stream_writer_;

  bool have_last_best_ = false;
  int last_best_id_ = -1;
  cv::Point2f last_best_center_{0.0f, 0.0f};
  int stable_count_ = 0;

  cv::Mat latest_frame_;
  bool new_frame_ = false;
  std::mutex frame_mutex_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionArTagNode>());
  rclcpp::shutdown();
  return 0;
}
