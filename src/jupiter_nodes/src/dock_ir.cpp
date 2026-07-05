// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_ir — IR beacon docking controller.
//
// Subscribes to /dock/ir (UInt8 bitmask published by ESP32):
//   0 = no beam        → stop / lost dock
//   1 = left beam only → steer left  (robot is right of centre)
//   2 = right beam only→ steer right (robot is left of centre)
//   3 = both beams     → drive straight (robot is centred)
//
// Stop condition: robot commanded forward but odom velocity < stuck threshold
// for stuck_timeout seconds → pogo pins have made contact → declare docked.
//
// Engage:  ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"
// Stop:    ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}

// IR bitmask values — must match ESP32 firmware
static constexpr uint8_t IR_NONE  = 0;
static constexpr uint8_t IR_LEFT  = 1;
static constexpr uint8_t IR_RIGHT = 2;
static constexpr uint8_t IR_BOTH  = 3;

class DockIr : public rclcpp::Node {
public:
  DockIr() : Node("dock_ir") {
    approach_speed_      = declare_parameter("approach_speed",      0.08);  // m/s forward
    steer_wz_            = declare_parameter("steer_wz",            0.20);  // rad/s when off-centre
    ir_timeout_          = declare_parameter("ir_timeout",          1.0);   // s — stop if no IR messages at all
    beam_lost_timeout_   = declare_parameter("beam_lost_timeout",   2.0);   // s — stop if beam absent this long
    stuck_vel_threshold_ = declare_parameter("stuck_vel_threshold", 0.01);  // m/s — below this = physically stopped
    stuck_timeout_       = declare_parameter("stuck_timeout",       3.0);   // s — stopped this long = docked
    control_rate_        = declare_parameter("control_rate",        20.0);  // Hz

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    ir_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/dock/ir", 10,
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        const uint8_t val = msg->data;
        last_ir_time_ = this->now();
        have_ir_      = true;
        if (val != IR_NONE) {
          latest_ir_        = val;   // update steering state only on valid reading
          last_beam_time_   = this->now();
        }
        // IR_NONE ignored here — beam_lost_timeout handles disengage
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_linear_x_ = msg->twist.twist.linear.x;
      });

    auto on_engage = [this](bool engage) {
      engaged_ = engage;
      if (engaged_) {
        reset_state();
        RCLCPP_INFO(get_logger(), "IR docking engaged.");
      } else {
        publish_zero();
        RCLCPP_INFO(get_logger(), "IR docking stopped.");
      }
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
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&DockIr::control_step, this));

    RCLCPP_INFO(get_logger(), "dock_ir online. Engage: ros2 service call /dock_engage std_srvs/srv/SetBool \"{data: true}\"");
  }

private:
  void publish_zero() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  void reset_state() {
    stuck_since_      = this->now();
    last_beam_time_   = this->now();
    moving_confirmed_ = false;
    // have_ir_ intentionally NOT reset — IR signal flows continuously;
    // ir_timeout handles stale detection independently of engage state.
  }

  void control_step() {
    if (!engaged_) { publish_zero(); return; }

    const bool ir_fresh = have_ir_ && (this->now() - last_ir_time_).seconds() < ir_timeout_;

    if (!ir_fresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No IR signal — stopped.");
      publish_zero();
      engaged_ = false;
      return;
    }

    // Stuck detection: once we've confirmed the robot is moving, watch for it stopping
    if (std::fabs(odom_linear_x_) > stuck_vel_threshold_) {
      moving_confirmed_ = true;
      stuck_since_      = this->now();
    }

    if (moving_confirmed_ && (this->now() - stuck_since_).seconds() > stuck_timeout_) {
      RCLCPP_INFO(get_logger(), "Docked — robot stopped (pogo contact).");
      publish_zero();
      engaged_ = false;
      return;
    }

    geometry_msgs::msg::Twist cmd;

    switch (latest_ir_) {
      case IR_BOTH:
        // Centred — drive straight
        cmd.linear.x  = approach_speed_;
        cmd.angular.z = 0.0;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "BOTH beams — straight");
        break;

      case IR_LEFT:
        // Robot is right of centre — steer left (positive wz)
        cmd.linear.x  = approach_speed_;
        cmd.angular.z = steer_wz_;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "LEFT beam — steer left");
        break;

      case IR_RIGHT:
        // Robot is left of centre — steer right (negative wz)
        cmd.linear.x  = approach_speed_;
        cmd.angular.z = -steer_wz_;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "RIGHT beam — steer right");
        break;

      case IR_NONE:
      default:
        // No beam — stop and wait; disengage only after beam_lost_timeout
        if ((this->now() - last_beam_time_).seconds() > beam_lost_timeout_) {
          RCLCPP_WARN(get_logger(), "Beam lost for %.1fs — disengaging.", beam_lost_timeout_);
          publish_zero();
          engaged_ = false;
          return;
        }
        publish_zero();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "No beam — waiting...");
        return;
    }

    cmd_pub_->publish(cmd);
  }

  // params
  double approach_speed_, steer_wz_, ir_timeout_, beam_lost_timeout_;
  double stuck_vel_threshold_, stuck_timeout_, control_rate_;

  // state
  uint8_t         latest_ir_{IR_NONE};
  rclcpp::Time    last_ir_time_;
  bool            have_ir_{false};
  bool            engaged_{false};
  bool            moving_confirmed_{false};
  rclcpp::Time    stuck_since_;
  rclcpp::Time    last_beam_time_;
  double          odom_linear_x_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr      ir_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr         engage_srv_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       engage_sub_;
  rclcpp::TimerBase::SharedPtr                               timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockIr>());
  rclcpp::shutdown();
  return 0;
}
