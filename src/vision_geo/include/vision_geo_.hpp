#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <set>
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
    double x_local = 0.0;    // pose.position.x — East from the EKF origin (MAVROS ENU)
    double y_local = 0.0;    // pose.position.y — North from the EKF origin
    double alt_local = 0.0;  // pose.position.z — Up from the EKF origin
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time pose_stamp{0, 0, RCL_ROS_TIME};     // arrival time of the pose (for frame matching)
    rclcpp::Time rel_alt_stamp{0, 0, RCL_ROS_TIME};  // arrival time of rel_alt
    bool rel_alt_valid = false;
    bool gps_valid = false;
    bool pose_valid = false;
    bool pose_z_valid = false;
  };

  struct GeoPoint
  {
    double lat = 0.0, lon = 0.0, alt = 0.0;
    double north_m = 0.0, east_m = 0.0;
    bool valid = false;
    bool center = true;   // inside map_center_ratio (edge positions are less accurate)
    double drone_e = 0.0, drone_n = 0.0;  // drone position used for this projection
    double r_norm = 0.0;                  // pixel distance from the principal point, 0 centre .. 1 corner
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
    std::vector<bool> enu_center;                   // per enu_buf entry: seen in the image centre
    std::vector<std::pair<rclcpp::Time, int>> class_buf; // (stamp, class_id) light observations, pre-link
    int         frames_missing = 0;
    int         linked_buoy_id = -1;               // -1 until confirmed & linked
  };

  // One entry per PHYSICAL buoy in the GPS world map (any buoy class merges into it).
  // Position = median of the last obs_window observations (robust to sway / bad frames).
  // State = light pattern from the recent class history: red / green / entry (flashing blue) /
  //         exit (solid blue) / off / unknown (not enough history yet).
  struct BuoyMapPoint
  {
    int         buoy_id;
    int         class_id;
    std::string label;
    std::deque<std::tuple<double, double, bool>> obs_hist;  // last obs_window (east_m, north_m, center)
    int         obs_count = 0;                     // total observations ever added
    double      east_m = 0.0, north_m = 0.0;       // = median of obs_hist (map ENU from home)
    double      lat = 0.0, lon = 0.0;              // derived from east_m/north_m
    int         sighting_count = 0;                // distinct passes that confirmed it
    float       best_confidence = 0.0f;
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
    std::deque<std::pair<rclcpp::Time, std::string>> light_hist;  // (stamp, "red"/"green"/"blue"/"off")
    std::string state = "unknown";
    int         test_role = 0;                     // enable_testing: 1 = entry, 2 = exit, 0 = none
  };

private:
  // ── Setup ───────────────────────────────────────────────────────────────────
  void setupCameraIntrinsics();
  void setupCamToBodyRotation(double roll_deg, double pitch_deg, double yaw_deg);
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

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
  void addObservationToBuoy(BuoyMapPoint & pt, double east_m, double north_m, bool center,
                            float confidence, const rclcpp::Time & stamp);
  void publishMarkers();
  // pose of the drone at a given time (nearest entry of pose_hist_), altitude kept from `state`
  MavrosState poseAt(const rclcpp::Time & t, const MavrosState & state);
  // height above the water: rel_alt → pose.z → fixed altitude, + altitude_offset_m
  double altitudeAboveWater(const MavrosState & state) const;
  void checkPnpAltitude(const std::vector<Detection> & dets, const MavrosState & state);
  void checkMountYaw(const MavrosState & state);
  // fuse map points closer than cluster_radius_m into one buoy (repeat until none left)
  void clusterBuoyMap();
  static void mergeBuoyInto(BuoyMapPoint & keep, BuoyMapPoint & gone);
  static void updateBuoyPosition(BuoyMapPoint & pt, double home_lat, double home_lon,
                                 int min_center_obs);

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
  void undistortForStream(cv::Mat & img);
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
  bool letterbox_ = true;        // keep the aspect ratio into the model input (pad, like ultralytics)
  float lb_scale_ = 1.0f;        // letterbox of the current frame: model = frame * scale + pad
  int lb_pad_x_ = 0;
  int lb_pad_y_ = 0;
  // camera device mode (0 / "" = driver default, e.g. 640x480 YUYV on the C922)
  int camera_width_ = 0;
  int camera_height_ = 0;
  int camera_fps_ = 0;
  std::string camera_fourcc_;
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
  double altitude_offset_m_ = 0.0;   // takeoff point height above the water [m]
  double camera_latency_ms_ = 0.0;   // frame grab → real capture time; pose is taken from then
  double map_center_ratio_ = 1.0;    // only detections inside this central part of the image feed the map
  bool   pnp_alt_check_ = false;     // log PnP depth vs altitude (checks altitude / fx)
  bool   mount_yaw_check_ = false;   // log the cam_to_body yaw that matches the image motion
  std::deque<MavrosState> pose_hist_;  // last ~2 s of poses (guarded by state_mutex_)
  // mount yaw check: sums of unit vectors of (true - estimated) motion angle
  double yaw_chk_sum_c_ = 0.0, yaw_chk_sum_s_ = 0.0;
  int    yaw_chk_n_ = 0;
  double fx_chk_px_ = 0.0, fx_chk_m_ = 0.0;   // fx check: image motion [px] · altitude vs drone motion [m]
  // vision mode from the order topic:
  //   UAV-GO:VISION-MAP / VISION-TARGET / VISION-MAP-TARGET → run with those outputs
  //   UAV-GO:VISION-PAUSE or MISSION-DONE → pause (map kept and still published)
  bool paused_ = false;
  bool started_ = false;           // first start order received
  bool mode_map_ = true;           // build + publish the buoy map (map_topic_prefix)
  bool mode_target_ = true;        // publish target_topic_prefix<class>
  double order_timeout_s_ = 3600.0;
  rclcpp::Time wait_start_;
  int clear_count_ = 0;
  rclcpp::TimerBase::SharedPtr clear_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_sub_;
  void orderCallback(const std_msgs::msg::String::SharedPtr msg);
  void clearOutputs();                                   // empty map, zero targets, no markers
  void publishZeroTargets(const rclcpp::Time & stamp);   // all classes "not detected"
  double yaw_chk_prev_e_ = 0.0, yaw_chk_prev_n_ = 0.0;
  bool   yaw_chk_has_prev_ = false;
  std::vector<std::pair<double, double>> frame_flow_px_;  // this frame: (du, dv) of tracked buoys

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
  bool stream_undistort_ = false;        // straighten ONLY the stream copy (detection stays on the raw image)
  double stream_undistort_alpha_ = 1.0;  // 1 = keep the whole view (black curved borders), 0 = crop to valid pixels
  cv::Mat stream_map1_, stream_map2_;    // undistortion maps, built once per frame size
  cv::Size stream_map_size_;
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
  double cluster_radius_m_;     // map points closer than this are the SAME buoy (< half buoy spacing)
  int    min_obs_publish_;      // a map point is shown / published only after this many observations
  double max_tilt_deg_;         // skip map observations while the drone tilts more than this
  int    obs_window_;           // observations kept per buoy for the median position
  bool   enable_testing_;       // testing: 1st blue buoy = entry, 2nd blue buoy = exit
  int    test_roles_given_ = 0; // enable_testing: blue roles handed out so far (0..2)
  double survey_time_s_;
  // edge correction: ground offset × (1 + edge_k · r²), r = 0 centre .. 1 image corner
  double edge_k_ = 0.0;
  bool   edge_k_check_ = false;           // fit edge_k from buoys seen both in the centre and at the edge
  double edge_fit_num_ = 0.0, edge_fit_den_ = 0.0, edge_fit_ang_ = 0.0;
  int    edge_fit_n_ = 0;
  void addEdgeSample(const BuoyMapPoint & pt, const GeoPoint & obs);        // Task 1 feedback (/vision_geo/map/buoy_*) held back this long after start
  rclcpp::Time survey_start_{0, 0, RCL_ROS_TIME};
  bool   survey_started_ = false;
  bool   survey_done_ = false;
  std::set<std::pair<int, int>> seen_together_;  // buoy id pairs seen in the SAME frame → never fused
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
