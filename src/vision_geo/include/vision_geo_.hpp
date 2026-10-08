#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#if defined(ENABLE_OPENVINO_RUNTIME)
#include <openvino/c/openvino.h>
#endif

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
    double x_local = 0.0;    // pose.position.x — North from home
    double y_local = 0.0;    // pose.position.y — West from home (= -East; negate to get East)
    double alt_local = 0.0;  // pose.position.z — Up from home
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool gps_valid = false;
    bool pose_valid = false;
    bool pose_z_valid = false;
  };

  struct GeoPoint
  {
    double lat = 0.0, lon = 0.0, alt = 0.0;
    double north_m = 0.0, east_m = 0.0;
    bool valid = false;
  };

  // Tracks a buoy currently visible in the frame (pixel space).
  // GPS observations accumulate here; committed to the map when buoy exits.
  // A buoy currently visible in the frame (pixel-space association).
  // Buffers observations until confirmed, then links to a permanent map point.
  struct ActiveTrack
  {
    int         class_id;
    std::string label;
    float       confidence = 0.0f;
    float       cx = 0.0f, cy = 0.0f;              // last pixel center
    std::vector<std::pair<double, double>> enu_buf; // (east_m, north_m) map coords, pre-link
    std::vector<std::pair<rclcpp::Time, int>> class_buf; // (stamp, class_id) light observations, pre-link
    int         frames_missing = 0;
    int         linked_buoy_id = -1;               // -1 until confirmed & linked
  };

  // One entry per PHYSICAL buoy in the GPS world map (any buoy class merges into it).
  // Position = running mean of EVERY observation across all passes (converges in place).
  // State = light pattern from the recent class history: red / green / entry (flashing blue) /
  //         exit (solid blue) / off / unknown (not enough history yet).
  struct BuoyMapPoint
  {
    int         buoy_id;
    int         class_id;
    std::string label;
    double      sum_east = 0.0, sum_north = 0.0;   // accumulators
    int         obs_count = 0;                     // total observations averaged in
    double      east_m = 0.0, north_m = 0.0;       // = sum / obs_count (map ENU from home)
    double      lat = 0.0, lon = 0.0;              // derived from east_m/north_m
    int         sighting_count = 0;                // distinct passes that confirmed it
    float       best_confidence = 0.0f;
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
    std::deque<std::pair<rclcpp::Time, std::string>> light_hist;  // (stamp, "red"/"green"/"blue"/"off")
    std::string state = "unknown";
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
  void relAltCallback(const std_msgs::msg::Float64::SharedPtr msg);

  // ── Core logic ──────────────────────────────────────────────────────────────
  void processFrame();
  std::vector<Detection> infer(const cv::Mat & frame);
  bool forwardOpenCV(const cv::Mat & blob, cv::Mat & out);

  // ── OpenVINO (same as vision_video: C API) ─────────────────────────────────
  void initializeOpenVINO();
  bool forwardOpenVINO(const cv::Mat & blob, cv::Mat & out);
#if defined(ENABLE_OPENVINO_RUNTIME)
  std::string getOpenVINOError(ov_status_e status) const;
#endif
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
  // add one observation to a buoy's running mean and refresh derived fields
  void addObservationToBuoy(BuoyMapPoint & pt, double east_m, double north_m,
                            float confidence, const rclcpp::Time & stamp);
  void publishMarkers();

  // ── Buoy light state (Task 1) ────────────────────────────────────────────────
  std::string lightColor(int class_id) const;   // "red"/"green"/"blue"/"off" from the class name
  void addLightObservation(BuoyMapPoint & pt, int class_id, const rclcpp::Time & stamp);
  void updateBuoyState(BuoyMapPoint & pt, const rclcpp::Time & stamp);
  static std::tuple<float, float, float> stateColor(const std::string & state);

  // ── Per-class topics ─────────────────────────────────────────────────────────
  // solvePnP pose of the closest detection per class → /vision_geo/target/<class>
  bool estimatePose(const Detection & det, cv::Mat & rvec, cv::Mat & tvec) const;
  void publishTargets(const std::vector<Detection> & dets, const rclcpp::Time & stamp);
  // confirmed buoys per state → /vision_geo/map/buoy_<state>
  void publishBuoyMap(const rclcpp::Time & stamp);

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

  // ── GStreamer stream (same as vision_hailo: RTP/H.264 over UDP) ─────────────
  bool ensureStreamWriter();
  void streamFrame(
    const cv::Mat & frame,
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

  bool enable_stream_;
  std::string stream_host_;
  int stream_port_;
  int stream_width_;
  int stream_height_;
  int stream_fps_;
  int stream_bitrate_kbps_;
  cv::VideoWriter stream_writer_;
  bool stream_writer_initialized_ = false;
  std::chrono::steady_clock::time_point last_stream_write_{};  // send at most stream_fps frames/s

  std::vector<double> object_size_m_;   // real object size per class, for solvePnP
  std::vector<bool> is_geo_class_;      // per class: GPS-projected + mapped (Task 1 buoys only)
  double state_window_s_;               // light history kept per buoy
  double flash_gap_min_s_;              // lit-to-lit gap that counts as a flash (off phase)
  double flash_gap_max_s_;
  double solid_min_s_;                  // lit this long without off → solid (exit)
  std::string target_topic_prefix_;
  std::string map_topic_prefix_;
  std::string pose_frame_id_;

  // ── Camera intrinsics ────────────────────────────────────────────────────────
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat K_inv_;
  cv::Mat R_cam2body_;

  // ── Inference ────────────────────────────────────────────────────────────────
  cv::dnn::Net net_;
  bool use_openvino_ = false;
  std::string openvino_device_;
  std::string openvino_model_path_;   // empty = model_path_ (.onnx); or an OpenVINO IR .xml
#if defined(ENABLE_OPENVINO_RUNTIME)
  ov_core_t * openvino_core_ = nullptr;
  ov_compiled_model_t * openvino_compiled_model_ = nullptr;
  ov_infer_request_t * openvino_infer_request_ = nullptr;
#endif
  cv::VideoCapture capture_;
  bool using_camera_device_ = false;

  // ── State ────────────────────────────────────────────────────────────────────
  MavrosState mavros_state_;
  std::mutex state_mutex_;

  double home_lat_ = 0.0;
  double home_lon_ = 0.0;
  bool home_set_ = false;

  std::vector<ActiveTrack>  active_tracks_;
  std::vector<BuoyMapPoint> buoy_map_;
  int    next_buoy_id_ = 1;

  double map_merge_radius_m_;
  int    min_obs_to_commit_;    // min GPS frames in buffer before committing a track
  int    track_exit_frames_;    // missed frames before track is committed
  double pixel_match_radius_px_; // pixel center distance to match same buoy across frames

  // ── Topic-mode image buffer ──────────────────────────────────────────────────
  cv::Mat latest_frame_;
  bool new_frame_ = false;
  std::mutex frame_mutex_;

  // ── ROS interfaces ───────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vision_msgs::msg::DetectedObjectArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;  // MAVROS rel_alt is Float64

  // target_pubs_ indexed by class id; state_pubs_ keyed by buoy state
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> target_pubs_;
  std::map<std::string, rclcpp::Publisher<vision_msgs::msg::DetectedObjectArray>::SharedPtr> state_pubs_;

  // ── FPS ──────────────────────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point fps_window_start_;
  std::size_t frames_in_window_ = 0;
  double fps_ = 0.0;
};

// Wait for expected_command on the mission topic (same behaviour as control's waitCommand(),
// without the hold-position part). A message matches when it starts with expected_command
// (case-insensitive), so "UAV-GO:RED:BLUE" matches "UAV-GO"; anything else is logged and ignored.
// Returns true when the command arrived, false on timeout.
bool waitCommand(const rclcpp::Node::SharedPtr & node, const std::string & expected_command = "UAV-GO",
                 const std::string & topic = "/mission/order", double timeout = 3600.0);
