// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_aligner (STAGE A: REVERSE_IN) — closed-loop reverse into the charging dock.
//
// The Dreame-style plan: robot gets to a pre-dock pose roughly centred & squared to
// the dock (~0.7 m out, rear facing it), then reverses straight in under IMU yaw-hold
// while the internal rails do the final mm of alignment, until the rear prox sensors
// seat. This node is STAGE A: the REVERSE_IN controller only. SQUARE (get onto the
// dock's normal axis from an offset) and Nav2 pre-dock delivery are later stages.
//
// WHY IMU yaw-hold and not reflector-skew all the way in: the continuity test
// (2026-07-24) showed reflector SKEW is clean only while range > ~0.4 m; below ~0.35 m
// the 250 mm strip subtends ~65 deg, the PCA line-fits a wide ARC not a line, and skew
// balloons to a meaningless +18 deg. So heading is held by the IMU (captured at start),
// NOT by the reflector, through the close-in. Lateral centring from the reflector is
// available as an OPTIONAL trim in the reliable band (default OFF for the first test).
//
// SAFETY: speeds capped low (proven v_lin<=0.06, v_ang<=0.12). Aborts to a full stop if
// the dock pose goes invalid too long or on timeout — never adventures. Publishes at
// 20 Hz (well inside the 400 ms cmd_vel watchdog); the firmware seat-reflex (stop reverse
// on both-prox / 1.5 s grace) is an independent backstop. Starts ONLY on an explicit
// service call — never moves on launch.
//
// START:  ros2 service call /dock/align_start  std_srvs/srv/Trigger
// CANCEL: ros2 service call /dock/align_cancel std_srvs/srv/Trigger
//
// Subscribes: /dock/reflector (Float32MultiArray, from dock_reflector),
//             /imu/data (sensor_msgs/Imu, yaw source), /dock/contact (UInt8 prox bitmask).
// Publishes:  /cmd_vel (Twist), /dock/aligner_state (String).

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace {
// /dock/reflector index convention (must match dock_reflector.cpp).
constexpr int D_VALID   = 0;
constexpr int D_LATERAL = 2;
constexpr int D_RANGE   = 3;
constexpr int D_SKEW    = 5;

double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}  // namespace

class DockAligner : public rclcpp::Node {
public:
  DockAligner() : Node("dock_aligner") {
    reflector_topic_ = declare_parameter<std::string>("reflector_topic", "/dock/reflector");
    imu_topic_       = declare_parameter<std::string>("imu_topic",       "/imu/data");
    contact_topic_   = declare_parameter<std::string>("contact_topic",   "/dock/contact");
    cmd_vel_topic_   = declare_parameter<std::string>("cmd_vel_topic",   "/cmd_vel");
    state_topic_     = declare_parameter<std::string>("state_topic",     "/dock/aligner_state");

    control_hz_       = declare_parameter("control_hz",        20.0);
    v_reverse_        = declare_parameter("v_reverse",         0.05);   // m/s, proven-slow
    kp_yaw_           = declare_parameter("kp_yaw",            0.8);    // rad/s per rad heading error
    max_angular_      = declare_parameter("max_angular",       0.12);   // rad/s cap (user-proven)
    lateral_gain_     = declare_parameter("lateral_gain",      0.0);    // OFF for v1; enable after basic reverse confirmed
    lateral_trust_range_ = declare_parameter("lateral_trust_range", 0.40); // m — reflector lateral only trusted beyond this
    max_lateral_corr_ = declare_parameter("max_lateral_corr",  0.05);   // rad/s clamp on the lateral trim term
    seat_contact_mask_= declare_parameter("seat_contact_mask", 3);      // both prox = seated
    seat_range_floor_ = declare_parameter("seat_range_floor",  0.15);   // m — secondary stop if contact never arrives
    invalid_abort_s_  = declare_parameter("invalid_abort_s",   1.0);    // s of invalid pose -> abort
    acquire_stable_s_ = declare_parameter("acquire_stable_s",  0.5);    // s of continuous valid pose before moving
    acquire_timeout_s_= declare_parameter("acquire_timeout_s", 10.0);   // s to get a lock -> abort
    overall_timeout_s_= declare_parameter("overall_timeout_s", 60.0);   // s cap on the whole reverse

    cmd_pub_   = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(1).transient_local());

    refl_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      reflector_topic_, 10, std::bind(&DockAligner::on_reflector, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(), std::bind(&DockAligner::on_imu, this, std::placeholders::_1));
    contact_sub_ = create_subscription<std_msgs::msg::UInt8>(
      contact_topic_, 10, std::bind(&DockAligner::on_contact, this, std::placeholders::_1));

    start_srv_ = create_service<std_srvs::srv::Trigger>("/dock/align_start",
      std::bind(&DockAligner::on_start, this, std::placeholders::_1, std::placeholders::_2));
    cancel_srv_ = create_service<std_srvs::srv::Trigger>("/dock/align_cancel",
      std::bind(&DockAligner::on_cancel, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&DockAligner::on_tick, this));

    set_state(IDLE);
    RCLCPP_INFO(get_logger(),
      "dock_aligner (Stage A reverse-in) ready. v_rev=%.2f kp_yaw=%.2f max_ang=%.2f lat_gain=%.2f. "
      "Call /dock/align_start to begin.", v_reverse_, kp_yaw_, max_angular_, lateral_gain_);
  }

private:
  enum State { IDLE, ACQUIRE, REVERSE_IN, SEATED, ABORT };
  const char* state_name(State s) const {
    switch (s) { case IDLE: return "IDLE"; case ACQUIRE: return "ACQUIRE";
                 case REVERSE_IN: return "REVERSE_IN"; case SEATED: return "SEATED";
                 default: return "ABORT"; }
  }
  void set_state(State s) {
    state_ = s;
    std_msgs::msg::String m; m.data = state_name(s);
    state_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "state -> %s", state_name(s));
  }

  void on_reflector(const std_msgs::msg::Float32MultiArray::SharedPtr m) {
    if (m->data.size() < 6) return;
    refl_valid_   = (m->data[D_VALID] > 0.5f);
    refl_lateral_ = m->data[D_LATERAL];
    refl_range_   = m->data[D_RANGE];
    refl_skew_    = m->data[D_SKEW];
    last_refl_time_ = now();
    if (refl_valid_) last_valid_refl_time_ = last_refl_time_;
  }
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr m) {
    const auto& q = m->orientation;
    // yaw from quaternion (z-axis rotation)
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_imu_ = true;
  }
  void on_contact(const std_msgs::msg::UInt8::SharedPtr m) { contact_mask_ = m->data; }

  void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (state_ == REVERSE_IN || state_ == ACQUIRE) {
      res->success = false; res->message = "already running";
      return;
    }
    if (!have_imu_) {
      res->success = false; res->message = "no IMU yet — is the micro-ROS agent up?";
      return;
    }
    acquire_start_ = now();
    valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    set_state(ACQUIRE);
    res->success = true; res->message = "acquiring dock lock...";
  }
  void on_cancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    abort_reason_ = "cancelled by service";
    set_state(ABORT);
    res->success = true; res->message = "aborting -> stop";
  }

  void publish_stop() {
    geometry_msgs::msg::Twist z;   // all-zero
    cmd_pub_->publish(z);
  }

  void on_tick() {
    const rclcpp::Time t = now();
    switch (state_) {
      case IDLE:
      case SEATED:
      case ABORT:
        // Hold still. Publishing zero keeps the base braked and the watchdog fed.
        publish_stop();
        return;

      case ACQUIRE: {
        const bool refl_fresh = have_refl() && refl_valid_;
        if (!refl_fresh) {
          valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        } else if (valid_since_.nanoseconds() == 0) {
          valid_since_ = t;
        }
        publish_stop();   // do NOT move until locked
        if (valid_since_.nanoseconds() != 0 &&
            (t - valid_since_).seconds() >= acquire_stable_s_) {
          target_yaw_ = yaw_;              // capture heading to hold through the reverse
          motion_start_ = t;
          set_state(REVERSE_IN);
          RCLCPP_INFO(get_logger(), "locked: range %.3f m, lateral %+.3f m. Holding yaw %.1f deg, reversing in.",
                      refl_range_, refl_lateral_, target_yaw_ * 180.0 / M_PI);
        } else if ((t - acquire_start_).seconds() > acquire_timeout_s_) {
          abort_reason_ = "no stable dock lock within acquire timeout";
          set_state(ABORT);
        }
        return;
      }

      case REVERSE_IN: {
        // --- termination checks (in priority order) ---
        if ((contact_mask_ & 0x03) == seat_contact_mask_) {
          RCLCPP_INFO(get_logger(), "both prox seated (contact=%u) — stop.", contact_mask_);
          set_state(SEATED); publish_stop(); return;
        }
        if (have_refl() && refl_valid_ && refl_range_ > 0.0 && refl_range_ < seat_range_floor_) {
          RCLCPP_WARN(get_logger(), "range %.3f m below floor with no both-prox — stopping (rails/firmware take over).",
                      refl_range_);
          set_state(SEATED); publish_stop(); return;
        }
        if ((t - last_valid_refl_time_).seconds() > invalid_abort_s_) {
          abort_reason_ = "dock pose invalid too long";
          set_state(ABORT); publish_stop(); return;
        }
        if ((t - motion_start_).seconds() > overall_timeout_s_) {
          abort_reason_ = "overall reverse timeout";
          set_state(ABORT); publish_stop(); return;
        }

        // --- control law: reverse holding captured IMU yaw (+ optional lateral trim) ---
        const double yaw_err = wrap_pi(target_yaw_ - yaw_);
        double angular = kp_yaw_ * yaw_err;

        // Optional lateral centring, ONLY in the reflector's reliable band. Default OFF
        // (lateral_gain 0). SIGN IS UNVERIFIED — confirm empirically before enabling:
        // reversing, dock behind; if dock lateral is +y (left), the rear must swing +y.
        if (lateral_gain_ > 0.0 && refl_valid_ && refl_range_ > lateral_trust_range_) {
          const double lat_term = clampd(lateral_gain_ * refl_lateral_,
                                          -max_lateral_corr_, max_lateral_corr_);
          angular += lat_term;
        }

        angular = clampd(angular, -max_angular_, max_angular_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -v_reverse_;   // negative = reverse, caster-first into the dock
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "REVERSE_IN: range %.3f | lat %+.3f | yaw_err %+.1f deg | ang %+.2f | contact %u",
          refl_range_, refl_lateral_, yaw_err * 180.0 / M_PI, angular, contact_mask_);
        return;
      }
    }
  }

  bool have_refl() const {
    return last_refl_time_.nanoseconds() != 0 && (now() - last_refl_time_).seconds() < 0.5;
  }

  // params
  std::string reflector_topic_, imu_topic_, contact_topic_, cmd_vel_topic_, state_topic_;
  double control_hz_, v_reverse_, kp_yaw_, max_angular_, lateral_gain_, lateral_trust_range_,
         max_lateral_corr_, seat_range_floor_, invalid_abort_s_, acquire_stable_s_,
         acquire_timeout_s_, overall_timeout_s_;
  int seat_contact_mask_;

  // live inputs
  bool   refl_valid_{false}, have_imu_{false};
  double refl_lateral_{0}, refl_range_{0}, refl_skew_{0}, yaw_{0}, target_yaw_{0};
  uint8_t contact_mask_{0};
  rclcpp::Time last_refl_time_{0,0,RCL_ROS_TIME}, last_valid_refl_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time acquire_start_{0,0,RCL_ROS_TIME}, valid_since_{0,0,RCL_ROS_TIME}, motion_start_{0,0,RCL_ROS_TIME};

  State state_{IDLE};
  std::string abort_reason_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr refl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr             contact_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, cancel_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockAligner>());
  rclcpp::shutdown();
  return 0;
}
