// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_approach — AprilTag docking controller.
//
// AprilTag solvePnP is used for long-range bearing alignment only (tx > 0.5 m).
// Inside 0.5 m the pose estimate is too noisy for steering — the robot locks its
// heading and drives straight; physical guide rails handle the final alignment.
//
// Phase APPROACH:    Forward + gentle bearing correction until 0.5 m checkpoint.
//                    If alignment is good enough at checkpoint -> STRAIGHT_IN.
//                    If alignment is too poor -> REVERSE and retry.
// Phase STRAIGHT_IN: Pure straight drive (zero wz). Guide rails take over.
//                    Stop on distance, stuck, or tag loss near dock.
// Phase REVERSE:     Back up ~0.5 m with slight outward steering to escape guide rail.
//                    Then retry APPROACH.
//
// Engage:  ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"
// Stop:    ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace {
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}

enum class DockPhase { APPROACH, STRAIGHT_IN, REVERSE };

class DockApproach : public rclcpp::Node {
public:
  DockApproach() : Node("dock_approach") {
    // APPROACH phase
    approach_speed_     = declare_parameter("approach_speed",      0.08);   // m/s
    k_bearing_          = declare_parameter("k_bearing",           0.4);    // bearing gain
    max_wz_             = declare_parameter("max_wz",              0.12);   // rad/s cap
    bearing_deadband_   = declare_parameter("bearing_deadband",    0.052);  // rad (3 deg)
    bearing_smooth_n_   = declare_parameter("bearing_smooth_n",    5);

    // Checkpoint — transition to STRAIGHT_IN or REVERSE
    checkpoint_tx_      = declare_parameter("checkpoint_tx",       0.50);   // m
    checkpoint_bearing_ = declare_parameter("checkpoint_bearing",  0.14);   // rad (8 deg)
    checkpoint_ty_      = declare_parameter("checkpoint_ty",       0.10);   // m

    // STRAIGHT_IN phase — pure forward, no steering, guide rails take over
    straight_speed_     = declare_parameter("straight_speed",      0.06);   // m/s — slow and steady
    target_distance_    = declare_parameter("target_distance",     0.22);   // m — stop distance
    position_tol_       = declare_parameter("position_tol",        0.04);   // m
    docked_on_loss_tx_  = declare_parameter("docked_on_loss_tx",   0.40);   // m — tag lost this close = docked
    stuck_timeout_      = declare_parameter("stuck_timeout",       4.0);    // s — physically stopped = docked

    // REVERSE phase
    reverse_distance_   = declare_parameter("reverse_distance",    0.55);   // m
    reverse_speed_      = declare_parameter("reverse_speed",       0.12);   // m/s

    // Misc
    detection_timeout_  = declare_parameter("detection_timeout",   0.8);    // s
    base_frame_         = declare_parameter("base_frame",          std::string("base_footprint"));

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
      if (engaged_) { reset_state(); }
      else { publish_zero(); }
      RCLCPP_INFO(get_logger(), "%s", engaged_ ? "docking engaged" : "docking stopped");
    };

    engage_srv_ = create_service<std_srvs::srv::SetBool>(
      "dock_engage",
      [on_engage](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        on_engage(req->data);
        res->success = true;
        res->message = req->data ? "engaged" : "stopped";
      });

    engage_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dock/engage", 10,
      [on_engage](std_msgs::msg::Bool::SharedPtr msg) { on_engage(msg->data); });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / declare_parameter("control_rate", 20.0)),
      std::bind(&DockApproach::control_step, this));

    RCLCPP_INFO(get_logger(), "dock_approach online. Engage: ros2 service call /dock_engage std_srvs/srv/SetBool \"{data: true}\"");
  }

private:
  void publish_zero() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  void reset_state() {
    phase_             = DockPhase::APPROACH;
    last_tx_           = 1e9;
    last_ty_           = 0.0;
    checkpoint_passed_ = false;
    reverse_tx_start_  = 1e9;
    straight_tx_start_ = 1e9;
    stuck_tx_ref_      = 1e9;
    stuck_since_       = this->now();
    retry_count_       = 0;
    bearing_buf_.fill(0.0);
    bearing_idx_       = 0;
    bearing_count_     = 0;
    first_reading_     = true;
    last_checkpoint_bearing_ = 0.0;
  }

  void control_step() {
    if (!engaged_) { publish_zero(); return; }

    const bool tag_fresh = have_marker_ && (this->now() - last_marker_time_).seconds() < detection_timeout_;

    // Tag loss handling depends on phase
    if (!tag_fresh) {
      if (phase_ == DockPhase::STRAIGHT_IN && last_tx_ <= docked_on_loss_tx_) {
        RCLCPP_INFO(get_logger(), "Docked (tag lost at tx=%.3f m).", last_tx_);
        publish_zero(); engaged_ = false; return;
      }
      if (phase_ == DockPhase::STRAIGHT_IN) {
        // Keep driving straight — tag loss at close range is expected
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = straight_speed_;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "STRAIGHT-IN: tag absent, driving straight");
        return;
      }
      RCLCPP_WARN(get_logger(), "Lost tag (tx=%.3f m) — stopping.", last_tx_);
      publish_zero(); engaged_ = false; return;
    }

    geometry_msgs::msg::PoseStamped tag_base;
    try {
      tf_buffer_->transform(latest_marker_, tag_base, base_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF: %s", ex.what());
      publish_zero(); return;
    }

    const double tx          = tag_base.pose.position.x;
    const double ty          = tag_base.pose.position.y;
    const double raw_bearing = std::atan2(ty, tx);

    bearing_buf_[bearing_idx_] = raw_bearing;
    bearing_idx_ = (bearing_idx_ + 1) % bearing_smooth_n_;
    if (bearing_count_ < bearing_smooth_n_) bearing_count_++;
    double bearing = 0.0;
    for (int i = 0; i < bearing_count_; ++i) bearing += bearing_buf_[i];
    bearing /= bearing_count_;

    if (!first_reading_ && std::fabs(ty - last_ty_) > 0.15) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Flip rejected: ty %.3f->%.3f", last_ty_, ty);
      return;
    }
    first_reading_ = false;
    last_tx_ = tx;
    last_ty_ = ty;

    geometry_msgs::msg::Twist cmd;

    // ── REVERSE ───────────────────────────────────────────────────────────────
    if (phase_ == DockPhase::REVERSE) {
      const double reversed = tx - reverse_tx_start_;
      if (reversed >= reverse_distance_) {
        RCLCPP_INFO(get_logger(), "REVERSE done (%.2f m backed) — retry #%d", reversed, retry_count_);
        phase_             = DockPhase::APPROACH;
        checkpoint_passed_ = false;
        bearing_buf_.fill(0.0); bearing_idx_ = 0; bearing_count_ = 0;
        first_reading_     = true;
        publish_zero(); return;
      }
      cmd.linear.x = -reverse_speed_;
      // Steer outward during reverse to escape guide rail: positive bearing at checkpoint
      // means robot was left of dock — steer left while reversing (positive wz while going back)
      cmd.angular.z = clamp(0.3 * last_checkpoint_bearing_, -0.15, 0.15);
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "REVERSE: tx %.2f m (%.2f m remaining) wz %.2f", tx, reverse_distance_ - reversed, cmd.angular.z);
      return;
    }

    // ── STRAIGHT_IN ───────────────────────────────────────────────────────────
    if (phase_ == DockPhase::STRAIGHT_IN) {
      // Stop: target distance reached
      if (tx <= target_distance_ + position_tol_) {
        RCLCPP_INFO(get_logger(), "Docked: tx=%.3f m ty=%.3f m. Stopping.", tx, ty);
        publish_zero(); engaged_ = false; return;
      }
      // Stop: physically stuck (guide rail contact / pogo pin hit)
      if (tx < stuck_tx_ref_ - 0.02) { stuck_tx_ref_ = tx; stuck_since_ = this->now(); }
      if (tx < 0.50 && (this->now() - stuck_since_).seconds() > stuck_timeout_) {
        RCLCPP_INFO(get_logger(), "Docked (stuck at tx=%.3f m). Stopping.", tx);
        publish_zero(); engaged_ = false; return;
      }
      cmd.linear.x = straight_speed_;   // pure straight — NO wz
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "STRAIGHT-IN: tx %.3f m ty %.3f m bearing %.1f deg (ignored)", tx, ty, bearing * 180.0 / M_PI);
      return;
    }

    // ── APPROACH ──────────────────────────────────────────────────────────────
    // Stop: somehow got very close without triggering checkpoint
    if (tx <= target_distance_ + position_tol_) {
      RCLCPP_INFO(get_logger(), "Docked: tx=%.3f m.", tx);
      publish_zero(); engaged_ = false; return;
    }

    // Checkpoint: decide whether to continue straight or reverse
    if (!checkpoint_passed_ && tx < checkpoint_tx_) {
      checkpoint_passed_       = true;
      last_checkpoint_bearing_ = bearing;
      const bool bearing_bad   = std::fabs(bearing) > checkpoint_bearing_;
      const bool ty_bad        = std::fabs(ty)      > checkpoint_ty_;
      if (bearing_bad || ty_bad) {
        retry_count_++;
        reverse_tx_start_ = tx;
        phase_ = DockPhase::REVERSE;
        RCLCPP_WARN(get_logger(),
          "Checkpoint FAIL tx=%.2f m bearing=%.1f deg ty=%.3f m — REVERSE #%d",
          tx, bearing * 180.0 / M_PI, ty, retry_count_);
        publish_zero(); return;
      }
      RCLCPP_INFO(get_logger(),
        "Checkpoint PASS tx=%.2f m bearing=%.1f deg ty=%.3f m — STRAIGHT-IN (guide rails take over)",
        tx, bearing * 180.0 / M_PI, ty);
      phase_             = DockPhase::STRAIGHT_IN;
      straight_tx_start_ = tx;
      stuck_tx_ref_      = tx;
      stuck_since_       = this->now();
      cmd.linear.x       = straight_speed_;
      cmd_pub_->publish(cmd);
      return;
    }

    // Bearing correction — only in APPROACH, only outside dead-band
    cmd.linear.x = approach_speed_;
    if (std::fabs(bearing) > bearing_deadband_) {
      cmd.angular.z = clamp(k_bearing_ * bearing, -max_wz_, max_wz_);
    }
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "APPROACH: tx %.2f m ty %.3f m bearing %.1f deg | vx %.2f wz %.3f",
      tx, ty, bearing * 180.0 / M_PI, cmd.linear.x, cmd.angular.z);
  }

  // params
  double approach_speed_, k_bearing_, max_wz_, bearing_deadband_;
  double checkpoint_tx_, checkpoint_bearing_, checkpoint_ty_;
  double straight_speed_, target_distance_, position_tol_, docked_on_loss_tx_, stuck_timeout_;
  double reverse_distance_, reverse_speed_, detection_timeout_;
  int    bearing_smooth_n_;
  std::string base_frame_;

  // state
  geometry_msgs::msg::PoseStamped latest_marker_;
  rclcpp::Time last_marker_time_;
  rclcpp::Time stuck_since_;
  bool have_marker_{false};
  bool engaged_{false};
  DockPhase phase_{DockPhase::APPROACH};
  double last_tx_{1e9}, last_ty_{0.0};
  double reverse_tx_start_{1e9}, straight_tx_start_{1e9};
  double stuck_tx_ref_{1e9};
  double last_checkpoint_bearing_{0.0};
  bool   checkpoint_passed_{false};
  int    retry_count_{0};
  std::array<double, 10> bearing_buf_{};
  int bearing_idx_{0}, bearing_count_{0};
  bool first_reading_{true};

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
