#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>

class VisionKRTINode : public rclcpp::Node
{
public:
  VisionKRTINode()
  : Node("vision_krti_node"),
    fps_window_start_(std::chrono::steady_clock::now())
  {
    input_mode_   = declare_parameter<std::string>("input_mode", "device");
    input_source_ = declare_parameter<std::string>("input_source", "/dev/video0");
    input_width_  = declare_parameter<int>("input_width", 640);
    input_height_ = declare_parameter<int>("input_height", 480);
    show_window_  = declare_parameter<bool>("show_window", true);
    window_name_ = declare_parameter<std::string>("window_name", "vision_krti_tag");
    enable_stream_ = declare_parameter<bool>("enable_stream", false);
    stream_host_ = declare_parameter<std::string>("stream_host", "10.7.101.148");
    stream_port_ = declare_parameter<int>("stream_port", 5000);
    stream_width_ = declare_parameter<int>("stream_width", 640);
    stream_height_ = declare_parameter<int>("stream_height", 480);
    stream_fps_ = declare_parameter<int>("stream_fps", 30);
    stream_bitrate_kbps_ = declare_parameter<int>("stream_bitrate_kbps", 500);
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
    dictionaries_ = buildDictionaries();

    if (input_mode_ == "topic") {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_source_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });
      RCLCPP_INFO(get_logger(), "Subscribing to image topic: %s", input_source_.c_str());
    } else {
      openInputSource();
      timer_ = create_wall_timer(
        std::chrono::milliseconds(1), std::bind(&VisionKRTINode::processFrame, this));
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

    pose_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/tag_pose", 10);
    RCLCPP_INFO(get_logger(), "Publishing tag pose data to /tag_pose");
  }

  ~VisionKRTINode() override
  {
    if (stream_writer_.isOpened()) {
      stream_writer_.release();
    }
    if (show_window_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  struct DictionaryInfo
  {
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::string name;
  };

  struct TagDetection
  {
    int id;
    std::string dict_name;
    std::array<cv::Point2f, 4> corners;
    cv::Point2f center;
    cv::Point2f offset;
    double center_dist = 0.0;
    double area = 0.0;
  };

  static bool startsWith(const std::string & value, const std::string & prefix)
  {
    return value.rfind(prefix, 0) == 0;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static std::vector<DictionaryInfo> buildDictionaries()
  {
    std::vector<DictionaryInfo> dicts;
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL), "ARUCO_ORIGINAL"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), "ARUCO_4X4_50"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100), "ARUCO_4X4_100"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250), "ARUCO_4X4_250"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000), "ARUCO_4X4_1000"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50), "ARUCO_5X5_50"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100), "ARUCO_5X5_100"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250), "ARUCO_5X5_250"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000), "ARUCO_5X5_1000"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50), "ARUCO_6X6_50"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100), "ARUCO_6X6_100"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), "ARUCO_6X6_250"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000), "ARUCO_6X6_1000"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50), "ARUCO_7X7_50"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100), "ARUCO_7X7_100"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250), "ARUCO_7X7_250"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000), "ARUCO_7X7_1000"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9), "APRILTAG_25h9"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10), "APRILTAG_36h10"});
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), "APRILTAG_36h11"});
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    dicts.push_back({cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_MIP_36h12), "ARUCO_MIP_36h12"});
#endif
    return dicts;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
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

      const std::string label = det.dict_name + ":" + std::to_string(det.id);
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

  void publishBestDetection(const TagDetection & det) const
  {
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    msg->data.resize(3);
    msg->data[0] = det.offset.x;
    msg->data[1] = det.offset.y;
    msg->data[2] = det.center_dist;
    pose_pub_->publish(std::move(msg));
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

    const cv::Point2f frame_center(frame.cols / 2.0f, frame.rows / 2.0f);
    for (const auto & dict : dictionaries_) {
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<std::vector<cv::Point2f>> rejected;
      cv::aruco::detectMarkers(gray, dict.dictionary, corners, ids, detector_params_, rejected);
      for (size_t i = 0; i < ids.size(); ++i) {
        if (corners[i].size() != 4) {
          continue;
        }
        TagDetection det;
        det.id = ids[i];
        det.dict_name = dict.name;
        for (size_t j = 0; j < 4; ++j) {
          det.corners[j] = corners[i][j];
        }

        det.center = cv::Point2f(
          (det.corners[0].x + det.corners[1].x + det.corners[2].x + det.corners[3].x) / 4.0f,
          (det.corners[0].y + det.corners[1].y + det.corners[2].y + det.corners[3].y) / 4.0f);

        det.offset = cv::Point2f(det.center.x - frame_center.x, det.center.y - frame_center.y);
        det.center_dist = std::hypot(det.offset.x, det.offset.y);
        det.area = std::fabs(cv::contourArea(corners[i]));
        detections.push_back(det);
      }
    }

    return detections;
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
        publishBestDetection(*best);
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 250,
          "FPS=%.2f | Detected %zu tag(s), best[%s:%d offset=(%.1f,%.1f) dist=%.1f]",
          fps_, detections.size(), best->dict_name.c_str(), best->id,
          best->offset.x, best->offset.y, best->center_dist);
      }
    } else {
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
  cv::VideoCapture capture_;
  bool using_camera_device_ = false;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  std::vector<DictionaryInfo> dictionaries_;

  std::chrono::steady_clock::time_point fps_window_start_;
  int frames_in_window_ = 0;
  double fps_ = 0.0;

  bool stream_writer_initialized_ = false;
  cv::VideoWriter stream_writer_;

  cv::Mat latest_frame_;
  bool new_frame_ = false;
  std::mutex frame_mutex_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionKRTINode>());
  rclcpp::shutdown();
  return 0;
}
