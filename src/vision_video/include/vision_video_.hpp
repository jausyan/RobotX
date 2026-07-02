#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#if defined(ENABLE_OPENVINO_RUNTIME)
#include <openvino/c/openvino.h>
#endif

class VisionVideoNode : public rclcpp::Node
{
public:
  VisionVideoNode();
  ~VisionVideoNode() override;

  struct Detection
  {
    int class_id;
    float confidence;
    cv::Rect box;
  };

private:
  static bool startsWith(const std::string & value, const std::string & prefix);

  void initializeOpenVINO();
#if defined(ENABLE_OPENVINO_RUNTIME)
  std::string getOpenVINOError(ov_status_e status) const;
#endif

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void setupCameraIntrinsics();
  void setupObjectPoints();

  bool estimatePose(const Detection & det, cv::Mat & rvec, cv::Mat & tvec) const;
  void publishLostPose();
  void publishPose(const cv::Mat & rvec, const cv::Mat & tvec);
  void publishLegacyPose(const cv::Mat & frame, const Detection & det);

  void openInputSource();
  void updateFps();
  void drawDetections(cv::Mat & frame, const std::vector<Detection> & detections) const;
  void drawCrosshair(cv::Mat & frame) const;
  void displayFrame(cv::Mat & frame, const std::vector<Detection> & detections);
  bool ensureStreamWriter();
  void streamFrame(const cv::Mat & frame, const std::vector<Detection> & detections);

  void processFrame();
  std::vector<Detection> infer(const cv::Mat & frame);

  // ── Parameters ──────────────────────────────────────────────────────────────
  std::string model_path_;
  std::string input_mode_;
  std::string input_source_;
  int input_width_;
  int input_height_;
  double conf_threshold_;
  double nms_threshold_;
  bool show_window_;
  std::string window_name_;
  bool enable_stream_;
  std::string stream_host_;
  int stream_port_;
  int stream_width_;
  int stream_height_;
  int stream_fps_;
  int stream_bitrate_kbps_;
  bool use_openvino_;
  std::string openvino_device_;
  std::string openvino_model_path_;
  double object_width_m_;
  double object_height_m_;
  std::string pose_frame_id_;
  bool use_camera_info_ = false;
  std::string camera_info_topic_;
  bool camera_info_received_ = false;

  // ── Camera intrinsics ───────────────────────────────────────────────────────
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::vector<cv::Point3f> object_points_;

  // ── Inference ───────────────────────────────────────────────────────────────
  cv::dnn::Net net_;
  cv::VideoCapture capture_;
  cv::VideoWriter stream_writer_;
  bool stream_writer_initialized_ = false;
#if defined(ENABLE_OPENVINO_RUNTIME)
  ov_core_t * openvino_core_ = nullptr;
  ov_compiled_model_t * openvino_compiled_model_ = nullptr;
  ov_infer_request_t * openvino_infer_request_ = nullptr;
#endif
  bool using_camera_device_ = false;

  // ── Topic-mode image buffer ─────────────────────────────────────────────────
  cv::Mat latest_frame_;
  bool new_frame_ = false;
  std::mutex frame_mutex_;

  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr legacy_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // ── FPS tracking ────────────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point fps_window_start_;
  std::size_t frames_in_window_ = 0;
  double fps_ = 0.0;
};
