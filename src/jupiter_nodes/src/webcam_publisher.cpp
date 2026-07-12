// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// webcam_publisher — publish a UVC webcam (/dev/videoN) as sensor_msgs/Image.
//
// The AprilTag docking camera is a plain USB webcam on the robot's tail (facing the
// dock), separate from the Orbbec. jupiter_vision (VPI tag36h11) subscribes to this
// topic. Kept minimal: open V4L2, MJPG, publish BGR frames at a fixed rate.
//
//   ros2 run jupiter_nodes webcam_publisher --ros-args -p device:=8 -p width:=1280 -p height:=720

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <memory>
#include <string>

class WebcamPublisher : public rclcpp::Node {
public:
  WebcamPublisher() : Node("webcam_publisher") {
    device_ = declare_parameter("device", 8);              // /dev/video8 = the tail webcam
    width_  = declare_parameter("width",  1280);
    height_ = declare_parameter("height", 720);
    fps_    = declare_parameter("fps",    30.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "webcam_optical_frame");
    const std::string topic = declare_parameter<std::string>("topic", "/webcam/image_raw");

    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Cannot open /dev/video%d", device_);
      throw std::runtime_error("webcam open failed");
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));  // MJPG for HD bandwidth
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);

    pub_ = create_publisher<sensor_msgs::msg::Image>(topic, 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps_),
                               std::bind(&WebcamPublisher::tick, this));
    RCLCPP_INFO(get_logger(), "webcam_publisher: /dev/video%d %dx%d @%.0f -> %s",
                device_, width_, height_, fps_, topic.c_str());
  }

private:
  void tick() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "empty frame from webcam");
      return;
    }
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = now();
    msg->header.frame_id = frame_id_;
    pub_->publish(*msg);
  }

  cv::VideoCapture cap_;
  int device_, width_, height_;
  double fps_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebcamPublisher>());
  rclcpp::shutdown();
  return 0;
}
