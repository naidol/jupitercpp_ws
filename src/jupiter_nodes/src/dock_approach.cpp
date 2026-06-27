// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_approach — AprilTag docking, 2-phase mecanum controller.
//
// Phase 1 ALIGN:    Rotate in place until bearing < 2 deg.
//
// Phase 2 DRIVE-IN: Constant forward speed + fixed lateral_bias to cancel
//                   mechanical left-drift (empirically tuned to 0.04 m/s).
//                   No proportional vy or wz corrections — those caused
//                   oscillation and guide-rail crashes during tuning.
//                   Nav2 delivers the robot square to the dock; guide fins
//                   handle the last few mm. Stop on distance or tag loss.
//
// Engage/stop:
//   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"
//   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}

enum class DockPhase { ALIGN, DRIVE_IN };

class DockApproach : public rclcpp::Node {
public:
  DockApproach() : Node("dock_approach") {
    target_distance_      = declare_parameter("target_distance",      0.22);   // m — stop at this distance
    position_tol_         = declare_parameter("position_tol",         0.04);   // m — forward tolerance
    align_exit_threshold_ = declare_parameter("align_exit_threshold", 0.035);  // rad (2 deg)
    max_angular_align_    = declare_parameter("max_angular_align",    0.50);   // rad/s
    max_linear_           = declare_parameter("max_linear",           0.10);   // m/s — constant forward speed
    max_angular_drivein_  = declare_parameter("max_angular_drivein",  0.05);   // rad/s — tight cap, bearing-only correction during drive-in
    k_ang_                = declare_parameter("k_ang",                4.0);    // angular proportional gain — high so small bearings exceed stiction (~0.15 rad/s)
    detection_timeout_    = declare_parameter("detection_timeout",    0.5);    // s
    docked_on_loss_tx_    = declare_parameter("docked_on_loss_tx",    0.35);   // m — tag lost this close = docked
    base_frame_           = declare_parameter("base_frame",           std::string("base_footprint"));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_pub_     = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vision/marker_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_marker_ = *msg;
        last_marker_time_ = this->now();
        have_marker_ = true;
      });

    auto on_engage = [this](bool engage) {
      engaged_ = engage;
      if (engaged_) {
        phase_        = DockPhase::ALIGN;
        last_tx_      = 1e9;
        last_ty_      = 0.0;
        stuck_tx_ref_ = 1e9;
        stuck_since_  = this->now();
      } else { publish_zero(); }
      RCLCPP_INFO(get_logger(), "%s", engaged_ ? "docking engaged — ALIGN phase" : "docking stopped");
    };

    engage_srv_ = create_service<std_srvs::srv::SetBool>(
      "dock_engage",
      [on_engage](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        on_engage(req->data);
        res->success = true;
        res->message = req->data ? "docking engaged" : "docking stopped";
      });

    engage_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dock/engage", 10,
      [on_engage](std_msgs::msg::Bool::SharedPtr msg) { on_engage(msg->data); });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / declare_parameter("control_rate", 20.0)),
      std::bind(&DockApproach::control_step, this));

    RCLCPP_INFO(get_logger(), "dock_approach online. Engage: /dock_engage.");
  }

private:
  void publish_zero() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  void control_step() {
    if (!engaged_) { publish_zero(); return; }

    if (!have_marker_ || (this->now() - last_marker_time_).seconds() > detection_timeout_) {
      if (last_tx_ <= docked_on_loss_tx_) {
        RCLCPP_INFO(get_logger(), "Docked (tag lost at tx=%.3f m). Stopping.", last_tx_);
      } else {
        RCLCPP_WARN(get_logger(), "Lost tag (tx=%.3f m) — stopping.", last_tx_);
      }
      publish_zero();
      engaged_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped tag_base;
    try {
      tf_buffer_->transform(latest_marker_, tag_base, base_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF failed: %s", ex.what());
      publish_zero();
      return;
    }

    const double tx      = tag_base.pose.position.x;
    const double ty      = tag_base.pose.position.y;
    const double bearing = std::atan2(ty, tx);

    // Reject solvePnP normal flips — ty cannot jump >15 cm in one control step.
    if (std::fabs(ty - last_ty_) > 0.15) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "Pose flip rejected: ty %.3f -> %.3f (%.0f cm jump)", last_ty_, ty, std::fabs(ty - last_ty_) * 100.0);
      return;
    }
    last_tx_ = tx;
    last_ty_ = ty;

    geometry_msgs::msg::Twist cmd;

    // ── Phase 1: ALIGN ─────────────────────────────────────────────────────────
    if (phase_ == DockPhase::ALIGN) {
      if (std::fabs(bearing) < align_exit_threshold_) {
        phase_ = DockPhase::DRIVE_IN;
        RCLCPP_INFO(get_logger(), "ALIGN done (bearing %.1f deg, tx=%.2f m) -> DRIVE-IN",
                    bearing * 180.0 / M_PI, tx);
      }
      cmd.angular.z = clamp(k_ang_ * bearing, -max_angular_align_, max_angular_align_);
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "ALIGN: bearing %.1f deg | wz %.2f", bearing * 180.0 / M_PI, cmd.angular.z);
      return;
    }

    // ── Phase 2: DRIVE-IN ──────────────────────────────────────────────────────
    // Stop 1: reached target distance.
    if (tx <= target_distance_ + position_tol_) {
      RCLCPP_INFO(get_logger(), "Docked: tx=%.3f m, ty=%.3f m, bearing=%.1f deg. Stopping.",
                  tx, ty, bearing * 180.0 / M_PI);
      publish_zero();
      engaged_ = false;
      return;
    }

    // Stop 2: stuck detector — physically blocked by guide fins or contact made.
    const auto now = this->now();
    if (tx < stuck_tx_ref_ - 0.02) {
      stuck_tx_ref_ = tx;
      stuck_since_  = now;
    }
    if (tx < 0.5 && (now - stuck_since_).seconds() > 3.0) {
      RCLCPP_INFO(get_logger(), "Docked (stuck at tx=%.3f m for >3 s). Stopping.", tx);
      publish_zero();
      engaged_ = false;
      return;
    }

    // Constant forward + bearing-only correction to keep robot pointed at tag.
    // No vy: fixed lateral strafe causes rotational coupling on mecanum that compounds bearing error.
    // When bearing=0, the robot is headed directly at the tag — no lateral correction needed.
    // Guide fins handle the last few mm of lateral error.
    cmd.linear.x = max_linear_;
    cmd.angular.z = clamp(k_ang_ * bearing, -max_angular_drivein_, max_angular_drivein_);
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "DRIVE-IN: tx %.2f m, ty %.3f m, bearing %.1f deg | vx %.2f wz %.3f",
      tx, ty, bearing * 180.0 / M_PI, cmd.linear.x, cmd.angular.z);
  }

  // params
  double target_distance_, position_tol_, align_exit_threshold_;
  double max_angular_align_, max_linear_, max_angular_drivein_;
  double k_ang_, detection_timeout_, docked_on_loss_tx_;
  std::string base_frame_;

  // state
  geometry_msgs::msg::PoseStamped latest_marker_;
  rclcpp::Time last_marker_time_;
  bool have_marker_{false};
  bool engaged_{false};
  DockPhase phase_{DockPhase::ALIGN};
  double last_tx_{1e9};
  double last_ty_{0.0};
  double stuck_tx_ref_{1e9};
  rclcpp::Time stuck_since_;

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
