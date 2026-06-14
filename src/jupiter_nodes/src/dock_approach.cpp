// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_approach — dead-simple AprilTag docking. No opennav, no frame-convention machinery.
//
// The whole algorithm, exactly as specified:
//   1. read the tag pose (/vision/marker_pose, from the VPI detector)
//   2. transform it into the robot base frame via tf2 (handles the camera mount + optical axes)
//   3. drive: turn to face the tag, creep forward, stop when distance == target (0.40 m)
//   4. stop on: reached target, lost the tag, or disengaged.
//
// Control: a plain proportional loop on a fixed-rate timer (turn-to-face, then approach).
// Forward-only (never reverses) so a control glitch can't run the robot backward.
//
// Engage/stop from the CLI:
//   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"   # start docking
//   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"  # stop + zero velocity
//
// SAFETY: publishes zero velocity the instant it disengages / loses the tag / reaches goal, and on
// shutdown. NOTE this does NOT remove the need for an ESP32 cmd_vel watchdog — if THIS node is hard-
// killed mid-command the firmware still holds the last value. That watchdog is the real safety net.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {
double clamp(double value, double lo, double hi) { return std::max(lo, std::min(hi, value)); }
}

class DockApproach : public rclcpp::Node {
public:
  DockApproach() : Node("dock_approach") {
    target_distance_   = declare_parameter("target_distance", 0.40);   // stop this far from the tag (m)
    distance_tol_      = declare_parameter("distance_tol", 0.03);      // "arrived" band (m)
    base_frame_        = declare_parameter("base_frame", std::string("base_footprint"));
    kp_linear_         = declare_parameter("kp_linear", 0.5);
    kp_angular_        = declare_parameter("kp_angular", 1.0);
    max_linear_        = declare_parameter("max_linear", 0.12);        // m/s, gentle
    max_angular_       = declare_parameter("max_angular", 0.5);        // rad/s
    face_gate_         = declare_parameter("face_gate_rad", 0.5);      // turn first if bearing exceeds this
    detection_timeout_ = declare_parameter("detection_timeout", 0.5); // s without a tag -> stop
    const double rate  = declare_parameter("control_rate", 20.0);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vision/marker_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_marker_ = *msg;
        last_marker_time_ = this->now();
        have_marker_ = true;
      });

    engage_srv_ = create_service<std_srvs::srv::SetBool>(
      "dock_engage",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        engaged_ = req->data;
        if (!engaged_) { publish_zero(); }
        res->success = true;
        res->message = engaged_ ? "docking engaged" : "docking stopped";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&DockApproach::control_step, this));

    RCLCPP_INFO(get_logger(), "dock_approach online — target %.2f m. Engage: /dock_engage (SetBool).",
                target_distance_);
  }

private:
  void publish_zero() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  void control_step() {
    // Keep asserting zero whenever not actively docking. The ESP32 has NO cmd_vel watchdog, so it
    // holds the last command it received — a single stop message can be dropped or stale, leaving the
    // robot creeping forward on the last approach velocity (observed: it reached 0.40 m, then crept into
    // the tag). Continuously publishing zero while idle guarantees it stays put. (In a future multi-
    // controller setup, gate this so it doesn't fight other cmd_vel sources; here dock_approach owns cmd_vel.)
    if (!engaged_) { publish_zero(); return; }

    // Watchdog: no recent tag detection -> stop.
    if (!have_marker_ || (this->now() - last_marker_time_).seconds() > detection_timeout_) {
      RCLCPP_WARN(get_logger(), "Lost the tag (no detection) -> stopping.");
      publish_zero();
      engaged_ = false;
      return;
    }

    // Tag pose in the robot base frame (tf2 handles camera mount + optical axes).
    geometry_msgs::msg::PoseStamped tag_base;
    try {
      tf_buffer_->transform(latest_marker_, tag_base, base_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF transform failed: %s", ex.what());
      publish_zero();
      return;
    }

    const double x = tag_base.pose.position.x;   // forward (+)
    const double y = tag_base.pose.position.y;   // left (+)
    const double distance = std::hypot(x, y);
    const double bearing  = std::atan2(y, x);    // angle to the tag (0 = dead ahead)

    if (distance <= target_distance_ + distance_tol_) {
      RCLCPP_INFO(get_logger(), "Docked: %.3f m from tag (target %.2f). Stopping.",
                  distance, target_distance_);
      publish_zero();
      engaged_ = false;
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = clamp(kp_angular_ * bearing, -max_angular_, max_angular_);
    // Turn to face the tag before driving; only creep forward when roughly aligned.
    if (std::fabs(bearing) < face_gate_) {
      cmd.linear.x = clamp(kp_linear_ * (distance - target_distance_), 0.0, max_linear_);
    }
    cmd_pub_->publish(cmd);
  }

  // params
  double target_distance_, distance_tol_, kp_linear_, kp_angular_;
  double max_linear_, max_angular_, face_gate_, detection_timeout_;
  std::string base_frame_;

  // state
  geometry_msgs::msg::PoseStamped latest_marker_;
  rclcpp::Time last_marker_time_;
  bool have_marker_{false};
  bool engaged_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr engage_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockApproach>());
  rclcpp::shutdown();
  return 0;
}
