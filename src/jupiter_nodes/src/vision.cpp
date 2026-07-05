// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vpi/VPI.h>
#include <vpi/algo/AprilTags.h>
#include <vpi/OpenCVInterop.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void vpi_check(VPIStatus status, const std::string& context) {
    if (status != VPI_SUCCESS) {
        throw std::runtime_error(context + " failed: " + vpiStatusGetName(status));
    }
}

// RAII guards for opaque VPI C handles
struct VpiStreamGuard {
    VPIStream handle = nullptr;
    VpiStreamGuard() = default;
    ~VpiStreamGuard() { if (handle) vpiStreamDestroy(handle); }
    VpiStreamGuard(const VpiStreamGuard&) = delete;
    VpiStreamGuard& operator=(const VpiStreamGuard&) = delete;
};

struct VpiPayloadGuard {
    VPIPayload handle = nullptr;
    VpiPayloadGuard() = default;
    ~VpiPayloadGuard() { if (handle) vpiPayloadDestroy(handle); }
    VpiPayloadGuard(const VpiPayloadGuard&) = delete;
    VpiPayloadGuard& operator=(const VpiPayloadGuard&) = delete;
};

struct VpiArrayGuard {
    VPIArray handle = nullptr;
    VpiArrayGuard() = default;
    ~VpiArrayGuard() { if (handle) vpiArrayDestroy(handle); }
    VpiArrayGuard(const VpiArrayGuard&) = delete;
    VpiArrayGuard& operator=(const VpiArrayGuard&) = delete;
};

// Per-frame image wrapper — stack-allocated, destroyed before function returns
struct VpiImageGuard {
    VPIImage handle = nullptr;
    VpiImageGuard() = default;
    ~VpiImageGuard() { if (handle) vpiImageDestroy(handle); }
    VpiImageGuard(const VpiImageGuard&) = delete;
    VpiImageGuard& operator=(const VpiImageGuard&) = delete;
};

// Shepperd's method: numerically stable rotation matrix → unit quaternion
geometry_msgs::msg::Quaternion rotation_matrix_to_quaternion(const cv::Mat& rot) {
    geometry_msgs::msg::Quaternion q;
    const double trace = rot.at<double>(0,0) + rot.at<double>(1,1) + rot.at<double>(2,2);

    if (trace > 0.0) {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (rot.at<double>(2,1) - rot.at<double>(1,2)) * s;
        q.y = (rot.at<double>(0,2) - rot.at<double>(2,0)) * s;
        q.z = (rot.at<double>(1,0) - rot.at<double>(0,1)) * s;
    } else if (rot.at<double>(0,0) > rot.at<double>(1,1) && rot.at<double>(0,0) > rot.at<double>(2,2)) {
        const double s = 2.0 * std::sqrt(1.0 + rot.at<double>(0,0) - rot.at<double>(1,1) - rot.at<double>(2,2));
        q.w = (rot.at<double>(2,1) - rot.at<double>(1,2)) / s;
        q.x = 0.25 * s;
        q.y = (rot.at<double>(0,1) + rot.at<double>(1,0)) / s;
        q.z = (rot.at<double>(0,2) + rot.at<double>(2,0)) / s;
    } else if (rot.at<double>(1,1) > rot.at<double>(2,2)) {
        const double s = 2.0 * std::sqrt(1.0 + rot.at<double>(1,1) - rot.at<double>(0,0) - rot.at<double>(2,2));
        q.w = (rot.at<double>(0,2) - rot.at<double>(2,0)) / s;
        q.x = (rot.at<double>(0,1) + rot.at<double>(1,0)) / s;
        q.y = 0.25 * s;
        q.z = (rot.at<double>(1,2) + rot.at<double>(2,1)) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + rot.at<double>(2,2) - rot.at<double>(0,0) - rot.at<double>(1,1));
        q.w = (rot.at<double>(1,0) - rot.at<double>(0,1)) / s;
        q.x = (rot.at<double>(0,2) + rot.at<double>(2,0)) / s;
        q.y = (rot.at<double>(1,2) + rot.at<double>(2,1)) / s;
        q.z = 0.25 * s;
    }
    return q;
}

static constexpr int MAX_TAG_DETECTIONS = 10;

} // namespace


class JupiterVision : public rclcpp::Node {
public:
    explicit JupiterVision() : Node("jupiter_vision") {
        declare_parameters();
        init_vpi();
        init_calibration();

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vision/marker_pose", 10);

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("camera_topic").as_string(), 10,
            std::bind(&JupiterVision::process_frame, this, std::placeholders::_1));

        trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/vision/trigger", 10,
            std::bind(&JupiterVision::trigger_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Jupiter Vision online — PVA AprilTags active");
    }

private:
    void declare_parameters() {
        this->declare_parameter("camera_topic",   "/camera/color/image_raw");
        this->declare_parameter("image_width",    1280);
        this->declare_parameter("image_height",   720);
        this->declare_parameter("marker_size_m",  0.100);  // 100mm tag — larger for stable solvePnP at close range
        this->declare_parameter("docking_tag_id", 1);
        // Frame the marker pose is published in. solvePnP returns the pose in the camera OPTICAL
        // convention (z forward, x right, y down), so this MUST be an optical frame — NOT camera_link
        // (body frame: x forward, z up) or every downstream TF transform mis-orients it (opennav_docking
        // would read the tag as "up" instead of "forward"). The Orbbec driver publishes this frame.
        this->declare_parameter("marker_frame_id", std::string("camera_color_optical_frame"));
        this->declare_parameter("snapshot_path",  std::string("/tmp/jupiter_view.jpg"));
        this->declare_parameter("show_preview",   false);
        // Orbbec Gemini 336 factory calibration defaults
        this->declare_parameter("cam_fx",   691.368);
        this->declare_parameter("cam_fy",   691.256);
        this->declare_parameter("cam_cx",   637.762);
        this->declare_parameter("cam_cy",   362.802);
        this->declare_parameter("dist_k1",  0.007423);
        this->declare_parameter("dist_k2", -0.047100);
        this->declare_parameter("dist_p1", -0.000144);
        this->declare_parameter("dist_p2", -0.000378);
        this->declare_parameter("dist_k3",  0.032794);
    }

    void init_vpi() {
        const int image_width  = this->get_parameter("image_width").as_int();
        const int image_height = this->get_parameter("image_height").as_int();

        vpi_check(vpiStreamCreate(VPI_BACKEND_CPU, &stream_.handle), "vpiStreamCreate");

        vpiInitAprilTagDecodeParams(&decode_params_);
        decode_params_.family = VPI_APRILTAG_36H11;

        vpi_check(
            vpiCreateAprilTagDetector(
                VPI_BACKEND_CPU, image_width, image_height, &decode_params_, &payload_.handle),
            "vpiCreateAprilTagDetector");

        vpi_check(
            vpiArrayCreate(MAX_TAG_DETECTIONS, VPI_ARRAY_TYPE_APRILTAG_DETECTION, VPI_BACKEND_CPU, &detections_.handle),
            "vpiArrayCreate");
    }

    void init_calibration() {
        const double fx = this->get_parameter("cam_fx").as_double();
        const double fy = this->get_parameter("cam_fy").as_double();
        const double cx = this->get_parameter("cam_cx").as_double();
        const double cy = this->get_parameter("cam_cy").as_double();

        camera_matrix_ = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

        const double k1 = this->get_parameter("dist_k1").as_double();
        const double k2 = this->get_parameter("dist_k2").as_double();
        const double p1 = this->get_parameter("dist_p1").as_double();
        const double p2 = this->get_parameter("dist_p2").as_double();
        const double k3 = this->get_parameter("dist_k3").as_double();

        dist_coeffs_ = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);

        marker_size_    = this->get_parameter("marker_size_m").as_double();
        docking_tag_id_ = this->get_parameter("docking_tag_id").as_int();
        show_preview_   = this->get_parameter("show_preview").as_bool();
        snapshot_path_  = this->get_parameter("snapshot_path").as_string();
        marker_frame_id_ = this->get_parameter("marker_frame_id").as_string();
    }

    void trigger_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data != "capture") {
            return;
        }
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (latest_frame_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Snapshot requested but no frame received yet");
            return;
        }
        if (!cv::imwrite(snapshot_path_, latest_frame_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write snapshot to %s", snapshot_path_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Snapshot saved to %s", snapshot_path_.c_str());
    }

    void process_frame(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (const cv_bridge::Exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge failed: %s", ex.what());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = cv_ptr->image;
        }

        cv::Mat gray_frame;
        cv::cvtColor(cv_ptr->image, gray_frame, cv::COLOR_BGR2GRAY);

        VpiImageGuard vpi_frame;
        vpi_check(
            vpiImageCreateWrapperOpenCVMat(gray_frame, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CPU, &vpi_frame.handle),
            "vpiImageCreateWrapperOpenCVMat");

        vpi_check(
            vpiSubmitAprilTagDetector(
                stream_.handle, VPI_BACKEND_CPU, payload_.handle,
                MAX_TAG_DETECTIONS, vpi_frame.handle, detections_.handle),
            "vpiSubmitAprilTagDetector");

        vpi_check(vpiStreamSync(stream_.handle), "vpiStreamSync");

        VPIArrayData out_data{};
        vpi_check(
            vpiArrayLockData(detections_.handle, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &out_data),
            "vpiArrayLockData");

        const auto* tag_list      = static_cast<const VPIAprilTagDetection*>(out_data.buffer.aos.data);
        const int   num_detections = *out_data.buffer.aos.sizePointer;

        for (int idx = 0; idx < num_detections; ++idx) {
            if (tag_list[idx].id == docking_tag_id_) {
                compute_and_publish_pose(tag_list[idx], msg->header.stamp);
            }
        }

        vpiArrayUnlock(detections_.handle);

        if (show_preview_) {
            cv::imshow("Jupiter Vision", cv_ptr->image);
            cv::waitKey(1);
        }
    }

    void compute_and_publish_pose(const VPIAprilTagDetection& tag, const rclcpp::Time& stamp) {
        const float half = static_cast<float>(marker_size_ / 2.0);
        const std::vector<cv::Point3f> object_points = {
            {-half,  half, 0.0f},
            { half,  half, 0.0f},
            { half, -half, 0.0f},
            {-half, -half, 0.0f},
        };

        std::vector<cv::Point2f> image_points;
        image_points.reserve(4);
        for (int corner = 0; corner < 4; ++corner) {
            image_points.push_back({tag.corners[corner].x, tag.corners[corner].y});
        }

        cv::Mat rvec, tvec;
        // VPI returns corners in distorted pixel coordinates, so dist_coeffs_ applies
        if (!cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec)) {
            RCLCPP_WARN(this->get_logger(), "solvePnP failed for tag %d", tag.id);
            return;
        }

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp    = stamp;
        pose_msg.header.frame_id = marker_frame_id_;
        pose_msg.pose.position.x = tvec.at<double>(0);
        pose_msg.pose.position.y = tvec.at<double>(1);
        pose_msg.pose.position.z = tvec.at<double>(2);
        pose_msg.pose.orientation = rotation_matrix_to_quaternion(rotation_matrix);

        pose_pub_->publish(pose_msg);
    }

    // RAII guards — destructors called automatically when node is destroyed
    VpiStreamGuard          stream_;
    VpiPayloadGuard         payload_;
    VpiArrayGuard           detections_;
    VPIAprilTagDecodeParams decode_params_{};

    cv::Mat     camera_matrix_;
    cv::Mat     dist_coeffs_;
    double      marker_size_{0.128};
    int         docking_tag_id_{1};
    bool        show_preview_{false};
    std::string snapshot_path_;
    std::string marker_frame_id_;

    std::mutex frame_mutex_;
    cv::Mat    latest_frame_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      img_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        trigger_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterVision>());
    rclcpp::shutdown();
    return 0;
}
