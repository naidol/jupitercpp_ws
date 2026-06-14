// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_approach — AprilTag docking with SQUARE-UP, holonomic (mecanum) control.
//
// Jupiter is mecanum and strafe (vy) is kept for docking, so we don't need a curved diff-drive path.
// Three errors are servoed straight to zero, independently and without singularities:
//   vx  -> approach-point forward error   (close the distance, along the tag normal)
//   vy  -> approach-point lateral error   (STRAFE onto the tag's centreline)
//   wz  -> bearing to the tag             (keep facing it -> tag stays centred in the camera)
//
// Each tick, in the base frame (recomputed from the live tag pose):
//   1. transform the tag into base_footprint via tf2.
//   2. from the tag ORIENTATION, take its outward normal (out of the tag face, toward the robot).
//   3. APPROACH POINT A = tag + target_distance * normal  (directly in front of the tag).
//   4. command vx,vy proportional to A (drive the base onto A), wz proportional to the tag bearing.
//   5. stop when at A (rho small) AND squared (facing the tag).
//
// Earlier v2 used a graceful diff-drive law that went singular as it neared the goal (w slammed to
// ±max, spun, and lost the tag). Holonomic servo has no such singularity and keeps the tag in view.
//
// Engage/stop:
//   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"   # start
//   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"  # stop + zero
//
// SAFETY: zero the instant it disengages / loses the tag / reaches goal, and zero every tick while
// idle (ESP32 has NO cmd_vel watchdog -> a single stop can be lost). Velocities capped low. A
// firmware cmd_vel watchdog is still the real safety net.
//
// REQUIRES the ESP32 to execute vy (mecanum strafe). If it doesn't, the lateral error won't close
// (robot won't move sideways) — fall back to a diff-drive cross-track controller.
//
// Tag-orientation caveat: a flat tag's normal from solvePnP gets noisy near head-on (planar
// ambiguity); square-up is most reliable from moderate offsets.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {
double clamp(double value, double lo, double hi) { return std::max(lo, std::min(hi, value)); }
}

class DockApproach : public rclcpp::Node {
public:
  DockApproach() : Node("dock_approach") {
    target_distance_   = declare_parameter("target_distance", 0.40);   // stand this far out, ALONG the tag normal (m)
    position_tol_      = declare_parameter("position_tol", 0.04);      // "on the spot" band (m)
    yaw_tol_           = declare_parameter("yaw_tol", 0.05);           // "squared" band (rad, ~2.9 deg)
    base_frame_        = declare_parameter("base_frame", std::string("base_footprint"));
    k_lin_             = declare_parameter("k_lin", 0.8);              // linear servo gain (vx, vy)
    k_ang_             = declare_parameter("k_ang", 1.0);              // angular servo gain (wz)
    max_linear_        = declare_parameter("max_linear", 0.12);        // m/s cap on vx and vy, gentle
    max_angular_       = declare_parameter("max_angular", 0.4);        // rad/s cap on wz
    detection_timeout_ = declare_parameter("detection_timeout", 0.5); // s without a tag -> stop
    docked_on_loss_rho_  = declare_parameter("docked_on_loss_rho", 0.12);  // tag lost within this of the goal -> count as docked
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
        if (engaged_) { last_rho_ = 1e9; }  // fresh start each docking run
        else { publish_zero(); }
        res->success = true;
        res->message = engaged_ ? "docking engaged" : "docking stopped";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    // Topic trigger (mirrors the service) so the brain can engage docking from a voice command.
    engage_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dock/engage", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        engaged_ = msg->data;
        if (engaged_) { last_rho_ = 1e9; }
        else { publish_zero(); }
        RCLCPP_INFO(get_logger(), "%s (via /dock/engage)", engaged_ ? "docking engaged" : "docking stopped");
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&DockApproach::control_step, this));

    RCLCPP_INFO(get_logger(), "dock_approach online — holonomic square-up, target %.2f m. Engage: /dock_engage.",
                target_distance_);
  }

private:
  void publish_zero() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  void control_step() {
    // Assert zero whenever not actively docking (ESP32 has no cmd_vel watchdog -> a single stop can be
    // lost, leaving the robot creeping on a stale command). dock_approach owns cmd_vel in this launch.
    if (!engaged_) { publish_zero(); return; }

    // Watchdog: no recent tag detection -> stop.
    if (!have_marker_ || (this->now() - last_marker_time_).seconds() > detection_timeout_) {
      if (last_rho_ <= docked_on_loss_rho_) {
        // The tag commonly drops out in the last few cm (it fills the frame / clips an edge). If we were
        // essentially at the goal, that's a successful dock, not a failure.
        RCLCPP_INFO(get_logger(), "Docked (tag lost within %.2f m of the goal; last rho %.3f m). Stopping.",
                    docked_on_loss_rho_, last_rho_);
      } else {
        RCLCPP_WARN(get_logger(), "Lost the tag (no detection) -> stopping.");
      }
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

    const double tx = tag_base.pose.position.x;          // tag forward (+) in base frame
    const double ty = tag_base.pose.position.y;          // tag left (+)
    const double tag_dist = std::hypot(tx, ty);
    const double bearing  = std::atan2(ty, tx);          // 0 = tag dead ahead

    // Tag outward normal (its local +z, "out of the face") projected to the ground plane.
    tf2::Quaternion q;
    tf2::fromMsg(tag_base.pose.orientation, q);
    const tf2::Matrix3x3 rot(q);
    double nx = rot[0][2];                               // tag +z axis expressed in base frame
    double ny = rot[1][2];
    const double nlen = std::hypot(nx, ny);
    if (nlen < 1e-3) {
      nx = -tx; ny = -ty;                                // degenerate -> point from tag toward robot
      const double l = std::hypot(nx, ny);
      if (l > 1e-6) { nx /= l; ny /= l; }
    } else {
      nx /= nlen; ny /= nlen;
    }
    // Planar-ambiguity guard: the normal must point toward the robot, not through the tag.
    if (nx * tx + ny * ty > 0.0) { nx = -nx; ny = -ny; }

    // Approach point: target_distance out from the tag along its normal (directly in front of the tag).
    const double ax = tx + target_distance_ * nx;        // forward error to the approach point
    const double ay = ty + target_distance_ * ny;        // lateral error to the approach point
    const double rho = std::hypot(ax, ay);
    last_rho_ = rho;                                     // remembered for the close-range "docked on tag-loss" case

    // Stop: at the approach point AND squared on the tag.
    if (rho <= position_tol_ && std::fabs(bearing) <= yaw_tol_) {
      RCLCPP_INFO(get_logger(), "Docked SQUARE: %.3f m from tag, lateral %.3f m, bearing %.1f deg. Stopping.",
                  tag_dist, ay, bearing * 180.0 / M_PI);
      publish_zero();
      engaged_ = false;
      return;
    }

    // Holonomic servo: drive base_footprint onto the approach point, rotate to keep facing the tag.
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = clamp(k_lin_ * ax,      -max_linear_,  max_linear_);
    cmd.linear.y  = clamp(k_lin_ * ay,      -max_linear_,  max_linear_);   // STRAFE onto the centreline
    cmd.angular.z = clamp(k_ang_ * bearing, -max_angular_, max_angular_);
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "approach: dist %.2fm bearing %.0fdeg | A(%.2f,%.2f) rho %.2f | vx %.2f vy %.2f w %.2f",
      tag_dist, bearing * 180.0 / M_PI, ax, ay, rho, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  // params
  double target_distance_, position_tol_, yaw_tol_;
  double k_lin_, k_ang_, max_linear_, max_angular_, detection_timeout_;
  double docked_on_loss_rho_;
  std::string base_frame_;

  // state
  geometry_msgs::msg::PoseStamped latest_marker_;
  rclcpp::Time last_marker_time_;
  bool have_marker_{false};
  bool engaged_{false};
  double last_rho_{1e9};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr engage_srv_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockApproach>());
  rclcpp::shutdown();
  return 0;
}
