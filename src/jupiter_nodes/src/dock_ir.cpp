// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_ir — IR beacon docking controller.
//
// Subscribes to /dock/ir (UInt8 bitmask published by ESP32):
//   0 = no beam        → beam lost (rotate-search, then give up)
//   1 = left beam only → steer left  (robot is right of centre)
//   2 = right beam only→ steer right (robot is left of centre)
//   3 = both beams     → drive straight (robot is centred)
//
// Docking is a HANDOFF: IR delivers the robot into the guide-rail mouth roughly
// square; the guide rails + magnetic pogo pins (self-snap at ~2-4mm) do the final
// alignment. IR does not need pogo precision.
//
// reverse:=true  → back in caster-first, so the low-friction rear swivel caster
//   (not the rubber drive wheels, which resist lateral push) is what the rails
//   square up — and the Orbbec/face/voice keep facing the room while charging.
//   Requires the IR receivers mounted at the (rear) docking end. Find steer_sign
//   empirically on first reverse test (flip to -1.0 if it diverges), same as we
//   validated the forward sign. See project_ir_docking memory.
//
// Engage:  ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"
// Stop:    ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <cmath>
#include <memory>

// IR bitmask values — must match ESP32 firmware
static constexpr uint8_t IR_NONE  = 0;
static constexpr uint8_t IR_LEFT  = 1;
static constexpr uint8_t IR_RIGHT = 2;
static constexpr uint8_t IR_BOTH  = 3;

class DockIr : public rclcpp::Node {
public:
  DockIr() : Node("dock_ir") {
    approach_speed_      = declare_parameter("approach_speed",      0.08);  // m/s
    steer_wz_            = declare_parameter("steer_wz",            0.20);  // rad/s when off-centre
    ir_timeout_          = declare_parameter("ir_timeout",          1.0);   // s — normal max IR-msg gap before pausing
    ir_hard_timeout_     = declare_parameter("ir_hard_timeout",     8.0);   // s — IR silent this long → give up (rides over ESP32 micro-ROS reconnects)
    beam_lost_timeout_   = declare_parameter("beam_lost_timeout",   2.0);   // s — retained for launch compatibility (unused)
    stuck_vel_threshold_ = declare_parameter("stuck_vel_threshold", 0.01);  // m/s — below this = physically stopped
    stuck_timeout_       = declare_parameter("stuck_timeout",       3.0);   // s — stalled this long WHILE DRIVING = docked
    control_rate_        = declare_parameter("control_rate",        20.0);  // Hz

    // Reverse docking (back in caster-first) — default off preserves the forward dock.
    reverse_             = declare_parameter("reverse",             false);
    steer_sign_          = declare_parameter("steer_sign",          1.0);   // set -1.0 if reverse steering diverges

    // Rotate-to-search on beam loss instead of aborting.
    search_enabled_      = declare_parameter("search_enabled",      true);
    beam_hold_           = declare_parameter("beam_hold",           0.8);   // s — hold last reading (bridge burst gaps) before searching
    search_wz_           = declare_parameter("search_wz",           0.30);  // rad/s — rotate speed while searching
    search_timeout_      = declare_parameter("search_timeout",      3.0);   // s — search this long, then give up

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    ir_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/dock/ir", 10,
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        const uint8_t val = msg->data;
        last_ir_time_ = this->now();
        have_ir_      = true;
        if (val != IR_NONE) {
          latest_ir_      = val;   // steering state updates only on a valid reading
          last_beam_time_ = this->now();
          beam_ever_      = true;
        }
        // IR_NONE handled by beam_age/search logic in control_step
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
        RCLCPP_INFO(get_logger(), "IR docking engaged (%s).", reverse_ ? "reverse" : "forward");
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
    engage_time_      = this->now();
    moving_confirmed_ = false;
    beam_ever_        = false;
    // have_ir_ intentionally NOT reset — IR flows continuously; ir_timeout handles staleness.
  }

  void control_step() {
    if (!engaged_) { publish_zero(); return; }

    // IR message flow. Brief silence — the ESP32 re-attaching to a fresh agent, or a
    // mid-approach micro-ROS blip — means STOP and WAIT (stay engaged). Give up only
    // if the topic stays silent for ir_hard_timeout.
    const double ir_gap = have_ir_ ? (this->now() - last_ir_time_).seconds()
                                   : (this->now() - engage_time_).seconds();
    if (ir_gap > ir_hard_timeout_) {
      RCLCPP_WARN(get_logger(), "No IR signal for %.1fs — disengaging.", ir_gap);
      publish_zero();
      engaged_ = false;
      return;
    }
    if (!have_ir_ || ir_gap > ir_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for IR signal...");
      publish_zero();
      stuck_since_ = this->now();   // paused, not stalled
      return;
    }

    // Engaged but no beam acquired yet — hold still (place robot facing the dock).
    if (!beam_ever_) {
      publish_zero();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for beam...");
      return;
    }

    const double drive    = reverse_ ? -approach_speed_ : approach_speed_;
    const double beam_age = (this->now() - last_beam_time_).seconds();

    geometry_msgs::msg::Twist cmd;
    bool driving = false;

    if (beam_age <= beam_hold_) {
      // Beam present (or a brief burst gap) — drive on the last valid reading.
      driving = true;
      switch (latest_ir_) {
        case IR_LEFT:
          cmd.linear.x  = drive;
          cmd.angular.z = steer_sign_ * steer_wz_;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "LEFT beam — steer");
          break;
        case IR_RIGHT:
          cmd.linear.x  = drive;
          cmd.angular.z = -steer_sign_ * steer_wz_;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "RIGHT beam — steer");
          break;
        case IR_BOTH:
        default:
          cmd.linear.x  = drive;
          cmd.angular.z = 0.0;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "BOTH beams — straight");
          break;
      }
    } else if (search_enabled_ && beam_age <= beam_hold_ + search_timeout_) {
      // Beam lost — rotate in place to re-acquire, toward the side last seen.
      const double dir = (latest_ir_ == IR_RIGHT) ? -1.0 : 1.0;   // LEFT/BOTH → rotate +, RIGHT → rotate -
      cmd.linear.x  = 0.0;
      cmd.angular.z = steer_sign_ * dir * search_wz_;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Beam lost — searching...");
    } else {
      // Search exhausted (or disabled) — give up.
      RCLCPP_WARN(get_logger(), "Beam lost — disengaging.");
      publish_zero();
      engaged_ = false;
      return;
    }

    // Docked (pogo-contact) detection — only meaningful WHILE DRIVING IN.
    // (Rotating during a search is a legitimate zero-linear-velocity state, not a stall.)
    if (driving) {
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
    } else {
      stuck_since_ = this->now();   // searching — reset the stall clock
    }

    cmd_pub_->publish(cmd);
  }

  // params
  double approach_speed_, steer_wz_, ir_timeout_, ir_hard_timeout_, beam_lost_timeout_;
  double stuck_vel_threshold_, stuck_timeout_, control_rate_;
  bool   reverse_{false}, search_enabled_{true};
  double steer_sign_{1.0}, beam_hold_{0.8}, search_wz_{0.30}, search_timeout_{3.0};

  // state
  uint8_t      latest_ir_{IR_NONE};
  rclcpp::Time last_ir_time_;
  bool         have_ir_{false};
  bool         engaged_{false};
  bool         moving_confirmed_{false};
  bool         beam_ever_{false};
  rclcpp::Time stuck_since_;
  rclcpp::Time last_beam_time_;
  rclcpp::Time engage_time_;
  double       odom_linear_x_{0.0};

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
