#include <rclcpp/rclcpp.hpp>
#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOptions.h>
#include <jetson-inference/detectNet.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>

struct offset {
    double x;
    double y;
};

struct objDimentions {
    float width;
    float height;
    float depth;
};

// Kamera Matrix
cv::Mat cameraMat = (cv::Mat_<float>(3, 3) <<
    504.99132615, 0.0, 318.4012728,
    0.0, 506.13168687, 243.99035432,
    0.0, 0.0, 1.0);

// Distorsi Matrix
cv::Mat distMat = (cv::Mat_<float>(1, 5) <<
    0.07771398, -0.18711237, -0.00182709, 0.00026326, -0.06212367);

cv::Mat convertCUDAtoBGR(uchar3* img, int width, int height) {
    return cv::Mat(height, width, CV_8UC3, img);
}

class ObjectDetectionCam : public rclcpp::Node
{
public:
    ObjectDetectionCam() : Node("fusee_cam"), model_loaded_(false), closed_(false), gst_writer_initialized_(false) {
        declare_and_get_parameters();
        setup_publishers_and_subscribers();

        load_model();
        start_video_stream();
        start_gstreamer_pipeline();

        model_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "Node initialized. Model loaded, video stream & GStreamer active.");
        
        frame_width = stream_->GetWidth();
        frame_height = stream_->GetHeight();
        // out = cv::VideoWriter("/home/jetson/payload4.mp4", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30.0, cv::Size(frame_width, frame_height));
        RCLCPP_INFO(this->get_logger(), "Video writer is ready.");
    }

    ~ObjectDetectionCam() { 
        stop_gstreamer_pipeline(); 
        // out.release();
    }

    void process_video() {
        uchar3* image = nullptr;
        int status = 0;

        if (!stream_->Capture(&image, 1000, &status)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture video frame");
            return;
        }

        // cv::Mat cv_img_raw = convertCUDAtoBGR(image, frame_width, frame_height);
        // out.write(cv_img_raw);
        this->detection(net_, image, frame_width, frame_height, obj_pose_);
    }

    bool isModelLoaded() const { return model_loaded_; }
    bool isClosed() const { return closed_; }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr payload_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ember_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr outdoor_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    
    // cv::VideoWriter out;
    int frame_width;
    int frame_height;

    geometry_msgs::msg::PoseStamped obj_pose_;
    float current_range;

    offset offsetPayload_;
    objDimentions payload_;
    detectNet* net_;
    videoSource* stream_;
    bool model_loaded_, closed_;

    // GStreamer
    cv::VideoWriter gst_writer_;
    bool gst_writer_initialized_;
    std::string gstreamer_host_, model_variant_, model_path_, label_path_;
    int gstreamer_port_, gstreamer_bitrate_, gstreamer_framerate_, gstreamer_width_, gstreamer_height_;

    void declare_and_get_parameters() {
        // Payload offset
        this->declare_parameter("payload.offset_x", 0.0);
        this->declare_parameter("payload.offset_y", 0.0);

        // GStreamer
        this->declare_parameter("gstreamer.host", "192.168.50.212");
        this->declare_parameter("gstreamer.port", 5600);
        this->declare_parameter("gstreamer.bitrate", 500);
        this->declare_parameter("gstreamer.framerate", 25);
        this->declare_parameter("gstreamer.width", 640);
        this->declare_parameter("gstreamer.height", 480);
        this->declare_parameter("gstreamer.camera_path", "/dev/video0");

        // Model selection
        this->declare_parameter("model.variant", "malem");
        this->declare_parameter("model.threshold", 0.80);

        // Get values
        this->get_parameter("payload.offset_x", offsetPayload_.x);
        this->get_parameter("payload.offset_y", offsetPayload_.y);
        this->get_parameter("gstreamer.host", gstreamer_host_);
        this->get_parameter("gstreamer.port", gstreamer_port_);
        this->get_parameter("gstreamer.bitrate", gstreamer_bitrate_);
        this->get_parameter("gstreamer.framerate", gstreamer_framerate_);
        this->get_parameter("gstreamer.width", gstreamer_width_);
        this->get_parameter("gstreamer.height", gstreamer_height_);
        this->get_parameter("gstreamer.camera_path", camera_path);
        this->get_parameter("model.variant", model_variant_);
        this->get_parameter("model.threshold", threshold_);

        // Model path
        std::string base_path = "/home/jetson/jetson-inference/python/training/detection/ssd/models/";
        model_path_ = base_path + model_variant_ + "/ssd-mobilenet.onnx";
        label_path_ = base_path + model_variant_ + "/labels.txt";

        RCLCPP_INFO(this->get_logger(), "Payload offset: (%.2f, %.2f)", offsetPayload_.x, offsetPayload_.y);
        RCLCPP_INFO(this->get_logger(), "GStreamer: %s:%d (%dx%d @%dfps, %dkbps)", 
                    gstreamer_host_.c_str(), gstreamer_port_, gstreamer_width_, gstreamer_height_, gstreamer_framerate_, gstreamer_bitrate_);
        RCLCPP_INFO(this->get_logger(), "Model variant: %s", model_variant_.c_str());
    }

    void setup_publishers_and_subscribers() {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        payload_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/down/payload", 10);
        ember_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/down/ember", 10);
        outdoor_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/down/outdoor", 10);

        range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "mavros/rangefinder/rangefinder", qos, std::bind(&ObjectDetectionCam::range_cb, this, std::placeholders::_1));
    }

    void detection(detectNet* net, uchar3* image, int width, int height, geometry_msgs::msg::PoseStamped &obj_pose) {
        detectNet::Detection* detections = nullptr;
        int numDetections = net->Detect(image, width, height, &detections);

        if (numDetections == 0) {
            obj_pose.pose.position.x = 0;
            obj_pose.pose.position.y = 0;
            obj_pose.pose.position.z = 0;

            payload_pose_pub_->publish(obj_pose);
            ember_pose_pub_->publish(obj_pose);
            outdoor_pose_pub_->publish(obj_pose);

            // RCLCPP_INFO(this->get_logger(), "No detections");
        } 
        else {
            for (int n = 0; n < numDetections; n++) {
                const detectNet::Detection& det = detections[n];

                float object_size = 0.0f;
                switch (det.ClassID) {
                    case 1: object_size = 0.125f; break; // payload
                    case 2: object_size = 0.33f; break; // ember
                    case 3: object_size = 0.40f; break; // outdoor
                    default: continue; // skip unknown classes
                }

                std::vector<cv::Point3f> objPoints = {
                    {-object_size/2,  object_size/2, 0},
                    { object_size/2,  object_size/2, 0},
                    { object_size/2, -object_size/2, 0},
                    {-object_size/2, -object_size/2, 0}
                };

                // If the detection too big, it is most likely a false detection
                if(det.Right - det.Left > gstreamer_width_ * 0.75 || det.Bottom - det.Top > gstreamer_height_ * 0.75)
                {
                    obj_pose.pose.position.x = 0;
                    obj_pose.pose.position.y = 0;
                    obj_pose.pose.position.z = 0;

                    payload_pose_pub_->publish(obj_pose);
                    ember_pose_pub_->publish(obj_pose);
                    outdoor_pose_pub_->publish(obj_pose);
                    continue;
                }

                std::vector<cv::Point2f> bbox = {
                    {det.Left,  det.Top},
                    {det.Right, det.Top},
                    {det.Right, det.Bottom},
                    {det.Left,  det.Bottom}
                };

                cv::Mat rvec, tvec;
                if (cv::solvePnP(objPoints, bbox, cameraMat, distMat, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {
                    obj_pose.pose.position.x = tvec.at<double>(0) + offsetPayload_.x;
                    obj_pose.pose.position.y = tvec.at<double>(1) + offsetPayload_.y;
                    obj_pose.pose.position.z = tvec.at<double>(2);

                    RCLCPP_INFO(this->get_logger(), "ClassID=%u | Tvec: [%.3f, %.3f, %.3f]",
                                det.ClassID, tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                }

                // publish to the right topic
                if (det.ClassID == 1) payload_pose_pub_->publish(obj_pose);
                else if (det.ClassID == 2) ember_pose_pub_->publish(obj_pose);
                else if (det.ClassID == 3) outdoor_pose_pub_->publish(obj_pose);
            }
        }

        cv::Mat cv_img = convertCUDAtoBGR(image, width, height);
        send_frame_to_gstreamer(cv_img);
    }

    void load_model() {
        net_ = detectNet::Create(NULL, model_path_.c_str(), NULL, label_path_.c_str(),
                                 threshold_, "input_0", "scores", "boxes");

        if (!net_)
            RCLCPP_ERROR(this->get_logger(), "Failed to load detectNet model (%s)", model_variant_.c_str());
    }

    void start_video_stream() {
        videoOptions options;
        options.width = 640;
        options.height = 480;
        options.frameRate = 30;

        stream_ = videoSource::Create(camera_path.c_str(), options);
        if (!stream_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source");
        }
        std::string focus_auto_cmd = "v4l2-ctl -d " + camera_path + " -c focus_auto=0";
        std::string focus_absolute_cmd = "v4l2-ctl -d " + camera_path + " -c focus_absolute=0";
        system(focus_auto_cmd.c_str());
        system(focus_absolute_cmd.c_str());
    }

    // void start_gstreamer_pipeline() {
    //     if (gst_writer_initialized_) return;

    //     std::string pipeline = "appsrc ! videoconvert ! video/x-raw,format=YUY2,width=" +
    //         std::to_string(gstreamer_width_) + ",height=" + std::to_string(gstreamer_height_) +
    //         ",framerate=" + std::to_string(gstreamer_framerate_) +
    //         "/1 ! videoconvert ! x264enc tune=zerolatency bitrate=" +
    //         std::to_string(gstreamer_bitrate_) +
    //         " speed-preset=superfast ! rtph264pay config-interval=1 pt=96 ! " +
    //         "udpsink host=" + gstreamer_host_ + " port=" + std::to_string(gstreamer_port_);

    //     gst_writer_.open(pipeline, cv::CAP_GSTREAMER, 0, gstreamer_framerate_,
    //                      cv::Size(gstreamer_width_, gstreamer_height_), true);

    //     gst_writer_initialized_ = gst_writer_.isOpened();
    //     if (!gst_writer_initialized_)
    //         RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline");
    // }

    void start_gstreamer_pipeline() {
        if (gst_writer_initialized_) return;

        std::string multicast_ip = gstreamer_host_;

        std::string pipeline = "appsrc ! videoconvert ! video/x-raw,format=YUY2,width=" +
            std::to_string(gstreamer_width_) + ",height=" + std::to_string(gstreamer_height_) +
            ",framerate=" + std::to_string(gstreamer_framerate_) +
            "/1 ! videoconvert ! x264enc tune=zerolatency bitrate=" +
            std::to_string(gstreamer_bitrate_) +
            " speed-preset=superfast ! rtph264pay config-interval=1 pt=96 ! " +
            "udpsink host=" + multicast_ip +
            " port=" + std::to_string(gstreamer_port_) +
            " auto-multicast=true";

        gst_writer_.open(pipeline, cv::CAP_GSTREAMER, 0, gstreamer_framerate_,
                        cv::Size(gstreamer_width_, gstreamer_height_), true);

        gst_writer_initialized_ = gst_writer_.isOpened();
        if (!gst_writer_initialized_)
            RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer multicast pipeline");
    }

    void stop_gstreamer_pipeline() {
        if (gst_writer_initialized_ && gst_writer_.isOpened()) {
            gst_writer_.release();
            gst_writer_initialized_ = false;
        }
    }

    void send_frame_to_gstreamer(const cv::Mat& frame) {
        if (!gst_writer_initialized_ || !gst_writer_.isOpened()) return;
        cv::Mat resized_frame;
        if (frame.size() != cv::Size(gstreamer_width_, gstreamer_height_))
            cv::resize(frame, resized_frame, cv::Size(gstreamer_width_, gstreamer_height_));
        else
            resized_frame = frame;
        gst_writer_.write(resized_frame);
    }

    void range_cb(const sensor_msgs::msg::Range::SharedPtr msg) { current_range = msg->range; }

    float threshold_;
    std::string camera_path;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectionCam>();
    rclcpp::Rate rate(30);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->isModelLoaded()) node->process_video();
        if (node->isClosed()) break;
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
