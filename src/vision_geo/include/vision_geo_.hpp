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

#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vision_msgs/msg/detected_object.hpp"
#include "vision_msgs/msg/detected_object_array.hpp"

class VisionGeoNode : public rclcpp::Node
{
public:
  VisionGeoNode();
  ~VisionGeoNode() override;

  struct Detection
  {
    int class_id;
    float confidence;
    cv::Rect box;
  };

  struct MavrosState
  {
    double lat = 0.0;
    double lon = 0.0;
    double alt_agl = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool gps_valid = false;
    bool pose_valid = false;
  };

  struct GeoPoint
  {
    double lat = 0.0, lon = 0.0, alt = 0.0;
    double north_m = 0.0, east_m = 0.0;
    bool valid = false;
  };

  struct AccumDetection
  {
    double lat, lon;
    double north_m, east_m;   // relative to home
    int class_id;
    std::string label;
    float confidence;
    rclcpp::Time timestamp;
  };

private:
  // ── Setup ───────────────────────────────────────────────────────────────────
  void setupCameraIntrinsics();
  void setupCamToBodyRotation(double roll_deg, double pitch_deg, double yaw_deg);

  // ── Callbacks ───────────────────────────────────────────────────────────────
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void relAltCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // ── Core logic ──────────────────────────────────────────────────────────────
  void processFrame();
  std::vector<Detection> infer(const cv::Mat & frame);
  GeoPoint projectPixelToGPS(float u, float v, const MavrosState & state) const;

  // ── Math helpers ─────────────────────────────────────────────────────────────
  static cv::Mat quaternionToMat(double x, double y, double z, double w);
  static cv::Mat rpy2mat(double roll_deg, double pitch_deg, double yaw_deg);

  // ── Publishing ───────────────────────────────────────────────────────────────
  void publishDetections(
    const std::vector<Detection> & dets,
    const std::vector<GeoPoint> & geo,
    const rclcpp::Time & stamp);
  void updateMarkers(
    const std::vector<Detection> & dets,
    const std::vector<GeoPoint> & geo,
    const rclcpp::Time & stamp);
  void publishMarkers();

  // ── Display ──────────────────────────────────────────────────────────────────
  void updateFps();
  void drawDetections(
    cv::Mat & frame,
    const std::vector<Detection> & dets,
    const std::vector<GeoPoint> & geo) const;
  void drawCrosshair(cv::Mat & frame) const;
  void displayFrame(
    cv::Mat & frame,
    const std::vector<Detection> & dets,
    const std::vector<GeoPoint> & geo);

  // ── Input source ─────────────────────────────────────────────────────────────
  void openInputSource();
  static bool startsWith(const std::string & value, const std::string & prefix);

  // ── Marker color helper ───────────────────────────────────────────────────────
  static std::tuple<float, float, float> classColor(int class_id);

  // ── Parameters ───────────────────────────────────────────────────────────────
  std::string input_mode_;
  std::string input_source_;
  bool use_camera_info_ = false;
  std::string camera_info_topic_;
  bool camera_info_received_ = false;

  std::string model_path_;
  int input_width_;
  int input_height_;
  double conf_threshold_;
  double nms_threshold_;
  std::vector<std::string> class_names_;

  std::string gps_topic_;
  std::string pose_topic_;
  std::string rel_alt_topic_;
  bool use_fixed_pose_ = false;
  double fixed_lat_;
  double fixed_lon_;
  double fixed_altitude_m_;
  int max_pose_age_ms_;

  std::string output_topic_;
  std::string marker_topic_;
  std::string marker_frame_id_;
  double marker_lifetime_s_;
  double marker_size_;

  bool show_window_;
  std::string window_name_;

  // ── Camera intrinsics ────────────────────────────────────────────────────────
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat K_inv_;
  cv::Mat R_cam2body_;

  // ── Inference ────────────────────────────────────────────────────────────────
  cv::dnn::Net net_;
  cv::VideoCapture capture_;
  bool using_camera_device_ = false;

  // ── State ────────────────────────────────────────────────────────────────────
  MavrosState mavros_state_;
  std::mutex state_mutex_;

  double home_lat_ = 0.0;
  double home_lon_ = 0.0;
  bool home_set_ = false;

  std::vector<AccumDetection> accum_detections_;
  int marker_id_counter_ = 0;

  // ── Topic-mode image buffer ──────────────────────────────────────────────────
  cv::Mat latest_frame_;
  bool new_frame_ = false;
  std::mutex frame_mutex_;

  // ── ROS interfaces ───────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vision_msgs::msg::DetectedObjectArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rel_alt_sub_;

  // ── FPS ──────────────────────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point fps_window_start_;
  std::size_t frames_in_window_ = 0;
  double fps_ = 0.0;
};
