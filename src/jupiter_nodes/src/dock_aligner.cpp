// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_aligner — closed-loop SQUARE-then-REVERSE into the charging dock (Dreame-style).
//
// FLOW: ACQUIRE (stable reflector lock) -> SQUARE (rotate in place at pre-dock to null
// the dock-face heading error, with full runway) -> REVERSE_IN (reverse while keeping
// square to the dock) -> SEATED (both prox, or partial-seat termination).
//
// HEADING METRIC — nyaw, not skew: the reflector debug array carries `skew` (whether the
// robot is on the dock's normal AXIS — a POSITION metric that in-place rotation cannot
// change) and `bearing`. The robot's true heading-squareness to the dock FACE is the dock
// outward-normal heading in the robot frame, nyaw = wrap(skew + bearing + pi). nyaw == 0
// means the robot's rear axis is normal to the dock face (both rear prox will meet the
// plates together). SQUARE and the reverse-band heading law both drive nyaw -> 0.
//
// WHY nyaw only while range > reflector_trust_range (~0.35 m): below that the 250 mm strip
// subtends ~65 deg, the PCA fits a wide ARC not a line, and the angle balloons (artefact).
// So heading comes from the reflector with runway (far), then from an IMU yaw-hold captured
// at the trust boundary for the final close-in where the rails do the last mm.
//
// The skew that stranded a real dock (2026-07-25: right prox seated, left open) came from
// reversing with an imperfect heading and no runway to fix it near the seat. Squaring first
// + holding square to the dock through the approach removes that.
//
// SAFETY: low speed caps; publishes 20 Hz (inside the 400 ms watchdog); firmware seat-reflex
// is an independent backstop; aborts to a stop on lost pose / timeout; never moves on launch
// (service-triggered). `square_only:=true` does SQUARE then stops — use it to verify the
// rotation sign safely (pure in-place rotation cannot collide) before ever reversing.
//
// START:  ros2 service call /dock/align_start  std_srvs/srv/Trigger
// CANCEL: ros2 service call /dock/align_cancel std_srvs/srv/Trigger

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
constexpr int D_BEARING = 4;
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
    v_reverse_        = declare_parameter("v_reverse",         0.05);   // m/s gentle APPROACH speed (heading stays stable)
    v_push_           = declare_parameter("v_push",            0.14);   // m/s firmer TERMINAL push inside the throat (a gentle creep can't overcome steel-on-PLA rail friction; a hand-push at this force slides+squares it home)
    throat_zone_      = declare_parameter("throat_zone",       0.28);   // m — at/below this reflector range: steering OFF + firm push, let the guide rails slide+square the robot (don't fight them)
    max_angular_      = declare_parameter("max_angular",       0.25);   // rad/s cap

    // Heading (dock-face squareness) control from the reflector, used in SQUARE and in the
    // far band of REVERSE_IN. heading_sign flips the rotation direction (verify with
    // square_only:=true if unsure): az = clamp(heading_sign * kp_heading * nyaw).
    kp_heading_       = declare_parameter("kp_heading",        1.2);
    heading_sign_     = declare_parameter("heading_sign",      1.0);
    square_tol_       = declare_parameter("square_tol_deg",    1.0) * M_PI / 180.0;   // ±1deg = ~1.75cm drift over 1m creep, inside a ±3-5cm funnel
    square_settle_s_  = declare_parameter("square_settle_s",   0.3);                  // hold in-band this long before locking heading (chassis settle)
    square_timeout_s_ = declare_parameter("square_timeout_s",  25.0);
    // In-place square uses a MIN rotational speed to reliably break floor scrub (a pure
    // proportional cmd stalls below scrub then overshoots — hunting). Proportional above the
    // floor for a gentle approach, hard-stop inside the deadband. (Gemini-recipe, 2026-07-27)
    square_omega_min_ = declare_parameter("square_omega_min",  0.15);   // rad/s floor to overcome scrub
    square_omega_max_ = declare_parameter("square_omega_max",  0.30);   // rad/s cap far from target
    square_only_      = declare_parameter("square_only",       false); // SQUARE then stop (sign check)

    // Below this range the reflector angle is unreliable -> hold IMU yaw instead.
    reflector_trust_range_ = declare_parameter("reflector_trust_range", 0.35);
    kp_yaw_           = declare_parameter("kp_yaw",            0.8);    // IMU yaw-hold gain (close-in)

    // Optional lateral centring trim (default OFF; sign unverified).
    lateral_gain_     = declare_parameter("lateral_gain",      0.0);
    lat_sign_         = declare_parameter("lat_sign",          1.0);    // steer-direction for cross-track; flip if lat diverges
    max_lateral_corr_ = declare_parameter("max_lateral_corr",  0.05);   // rad/s clamp on the cross-track contribution

    seat_contact_mask_= declare_parameter("seat_contact_mask", 3);      // both prox
    seat_zone_        = declare_parameter("seat_zone",         0.30);   // m — only seat-terminate this close
    seat_settle_s_    = declare_parameter("seat_settle_s",     2.0);    // s of no-progress + contact -> seated
    stuck_abort_s_    = declare_parameter("stuck_abort_s",     4.0);    // s of no-progress + NO contact near seat -> abort (jammed skewed)
    stall_eps_        = declare_parameter("stall_eps",         0.008);  // m — progress smaller than this = stalled
    seat_range_floor_ = declare_parameter("seat_range_floor",  0.15);   // m — hard stop even with no contact

    // FINAL PUSH: one prox latched but the other won't seat (robot arrived a touch skewed) -> pivot
    // the un-seated rear corner IN by rotating about the seated one, until BOTH latch. Sign is
    // DERIVED, not guessed: contact=1 (left seated, right open) -> CW (az<0) swings the right-rear
    // toward the dock; contact=2 (right seated) -> CCW (az>0). Firmware zeros angular the instant
    // both prox confirm (dock_seated), so this is self-terminating and can't over-rotate.
    push_omega_       = declare_parameter("push_omega",        0.22);   // rad/s pivot rate for the seat nudge
    push_v_           = declare_parameter("push_v",            0.03);   // m/s gentle reverse kept during the nudge (firmware may gate it)
    push_timeout_s_   = declare_parameter("push_timeout_s",    4.0);    // s — give up nudging -> accept partial seat

    invalid_abort_s_  = declare_parameter("invalid_abort_s",   1.0);
    acquire_stable_s_ = declare_parameter("acquire_stable_s",  0.5);
    acquire_timeout_s_= declare_parameter("acquire_timeout_s", 10.0);
    overall_timeout_s_= declare_parameter("overall_timeout_s", 90.0);

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
      "dock_aligner ready. v_rev=%.2f max_ang=%.2f kp_head=%.2f(sign %+.0f) square_tol=%.1fdeg "
      "trust_range=%.2f square_only=%d. Call /dock/align_start.",
      v_reverse_, max_angular_, kp_heading_, heading_sign_, square_tol_*180.0/M_PI,
      reflector_trust_range_, square_only_ ? 1 : 0);
  }

private:
  enum State { IDLE, ACQUIRE, SQUARE, REVERSE_IN, PUSH, SEATED, ABORT };
  const char* state_name(State s) const {
    switch (s) { case IDLE: return "IDLE"; case ACQUIRE: return "ACQUIRE"; case SQUARE: return "SQUARE";
                 case REVERSE_IN: return "REVERSE_IN"; case PUSH: return "PUSH";
                 case SEATED: return "SEATED"; default: return "ABORT"; }
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
    const double bearing = m->data[D_BEARING];
    const double skew    = m->data[D_SKEW];
    refl_nyaw_    = wrap_pi(skew + bearing + M_PI);   // dock-face heading error (0 = square)
    last_refl_time_ = now();
    if (refl_valid_) last_valid_refl_time_ = last_refl_time_;
  }
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr m) {
    const auto& q = m->orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_imu_ = true;
  }
  void on_contact(const std_msgs::msg::UInt8::SharedPtr m) { contact_mask_ = m->data; }

  void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (state_ == ACQUIRE || state_ == SQUARE || state_ == REVERSE_IN) {
      res->success = false; res->message = "already running"; return;
    }
    if (!have_imu_) { res->success = false; res->message = "no IMU yet — micro-ROS agent up?"; return; }
    acquire_start_ = now();
    valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    in_throat_ = false;
    set_state(ACQUIRE);
    res->success = true; res->message = "acquiring dock lock...";
  }
  void on_cancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    abort_reason_ = "cancelled by service"; set_state(ABORT);
    res->success = true; res->message = "aborting -> stop";
  }

  void publish_stop() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }
  bool have_refl() const {
    return last_refl_time_.nanoseconds() != 0 && (now() - last_refl_time_).seconds() < 0.5;
  }
  // Heading command to null the dock-face error (used in SQUARE and the reverse far-band).
  double heading_cmd() const {
    return clampd(heading_sign_ * kp_heading_ * refl_nyaw_, -max_angular_, max_angular_);
  }

  void on_tick() {
    const rclcpp::Time t = now();
    switch (state_) {
      case IDLE:
      case SEATED:
      case ABORT:
        publish_stop();
        return;

      case ACQUIRE: {
        const bool fresh = have_refl() && refl_valid_;
        if (!fresh) valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        else if (valid_since_.nanoseconds() == 0) valid_since_ = t;
        publish_stop();
        if (valid_since_.nanoseconds() != 0 && (t - valid_since_).seconds() >= acquire_stable_s_) {
          square_ok_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          set_state(SQUARE);
          RCLCPP_INFO(get_logger(), "locked: range %.3f lateral %+.3f nyaw %+.1f deg -> squaring.",
                      refl_range_, refl_lateral_, refl_nyaw_ * 180.0 / M_PI);
        } else if ((t - acquire_start_).seconds() > acquire_timeout_s_) {
          abort_reason_ = "no stable dock lock"; set_state(ABORT);
        }
        return;
      }

      case SQUARE: {
        if ((t - last_valid_refl_time_).seconds() > invalid_abort_s_) {
          abort_reason_ = "pose invalid during square"; set_state(ABORT); publish_stop(); return;
        }
        // Rotate in place to null the dock-face heading error. No linear motion = full runway,
        // no collision risk (this is also what square_only tests for sign).
        const bool squared = std::fabs(refl_nyaw_) <= square_tol_;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        if (squared) {
          cmd.angular.z = 0.0;   // hard-stop inside the ±square_tol deadband -> let the chassis settle
          if (square_ok_since_.nanoseconds() == 0) square_ok_since_ = t;   // start settle timer
        } else {
          // Proportional above the scrub floor, capped; the MIN speed guarantees it actually
          // rotates instead of stalling below breakaway then overshooting (the old hunting).
          const double mag = clampd(kp_heading_ * std::fabs(refl_nyaw_),
                                    square_omega_min_, square_omega_max_);
          cmd.angular.z = heading_sign_ * std::copysign(mag, refl_nyaw_);
          square_ok_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());   // reset settle timer
        }
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "SQUARE: nyaw %+.1f deg | az %+.2f | range %.3f%s",
          refl_nyaw_ * 180.0 / M_PI, cmd.angular.z, refl_range_, squared ? "  [in tol]" : "");

        if (square_ok_since_.nanoseconds() != 0 && (t - square_ok_since_).seconds() >= square_settle_s_) {
          if (square_only_) { RCLCPP_INFO(get_logger(), "square_only: squared, stopping."); set_state(SEATED); publish_stop(); return; }
          best_range_ = 1e9; best_range_time_ = t; imu_hold_active_ = false;
          motion_start_ = t; set_state(REVERSE_IN);
          RCLCPP_INFO(get_logger(), "squared (nyaw %+.1f deg) — reversing in.", refl_nyaw_ * 180.0 / M_PI);
        } else if ((t - acquire_start_).seconds() > square_timeout_s_) {
          abort_reason_ = "square did not converge (check heading_sign?)"; set_state(ABORT); publish_stop();
        }
        return;
      }

      case REVERSE_IN: {
        // --- terminations ---
        if ((contact_mask_ & 0x03) == seat_contact_mask_) {
          RCLCPP_INFO(get_logger(), "both prox seated (contact=%u).", contact_mask_);
          set_state(SEATED); publish_stop(); return;
        }
        if (have_refl() && refl_valid_ && refl_range_ > 0.0 && refl_range_ < seat_range_floor_) {
          RCLCPP_WARN(get_logger(), "range %.3f below floor — stop (rails/firmware take over).", refl_range_);
          set_state(SEATED); publish_stop(); return;
        }
        // Partial-seat / stall termination: in the seat zone, touching at least one prox, and no
        // further progress for seat_settle_s -> stop (don't grind against the dock).
        if (refl_range_ < best_range_ - stall_eps_) { best_range_ = refl_range_; best_range_time_ = t; }
        if (refl_range_ < seat_zone_ && (contact_mask_ & 0x03) != 0 &&
            (t - best_range_time_).seconds() > seat_settle_s_) {
          RCLCPP_WARN(get_logger(), "one prox latched (contact=%u, left=%d right=%d), no progress %.1fs "
                      "at range %.3f — final PUSH to seat the other.", contact_mask_,
                      (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0, seat_settle_s_, refl_range_);
          push_start_ = t; set_state(PUSH); return;
        }
        // Jammed near the seat but NO prox engaged (arrived too skewed) — don't grind; abort.
        if (refl_range_ < seat_zone_ && (contact_mask_ & 0x03) == 0 &&
            (t - best_range_time_).seconds() > stuck_abort_s_) {
          abort_reason_ = "stuck near seat with no prox contact (arrived skewed)";
          RCLCPP_WARN(get_logger(), "jammed at range %.3f, no prox, no progress %.1fs — abort.",
                      refl_range_, stuck_abort_s_);
          set_state(ABORT); publish_stop(); return;
        }
        if ((t - last_valid_refl_time_).seconds() > invalid_abort_s_) {
          abort_reason_ = "dock pose invalid too long"; set_state(ABORT); publish_stop(); return;
        }
        if ((t - motion_start_).seconds() > overall_timeout_s_) {
          abort_reason_ = "overall reverse timeout"; set_state(ABORT); publish_stop(); return;
        }

        // --- TERMINAL THROAT PUSH ---
        // Once the rear is in the funnel throat, STOP steering (angular=0) and push firmly straight.
        // The mechanical guide rails do the squaring; a gentle creep can't beat steel-on-PLA friction
        // (a hand-push at this force slid it home). Steering here would only FIGHT the rails. Latched,
        // so a reflector dropout at close range can't kick us back to steering mid-throat.
        if (refl_valid_ && refl_range_ > 0.0 && refl_range_ < throat_zone_) in_throat_ = true;
        if (in_throat_) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x  = -v_push_;
          cmd.angular.z = 0.0;
          cmd_pub_->publish(cmd);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
            "THROAT PUSH: range %.3f | v %.2f | steering OFF (rails square) | contact %u",
            refl_range_, v_push_, contact_mask_);
          return;
        }

        // --- heading law: reflector nyaw with runway (far), IMU-hold for the close-in ---
        double angular;
        if (refl_valid_ && refl_range_ > reflector_trust_range_) {
          angular = heading_cmd();                       // keep squaring to the dock
          imu_hold_active_ = false;                      // re-arm the hold capture
        } else {
          if (!imu_hold_active_) { imu_hold_yaw_ = yaw_; imu_hold_active_ = true; } // capture at boundary
          angular = clampd(kp_yaw_ * wrap_pi(imu_hold_yaw_ - yaw_), -max_angular_, max_angular_);
        }
        // CROSS-TRACK correction (far band only): steer the rear onto the dock's normal axis so
        // a lateral staging offset gets pulled in instead of missing the funnel. lat_sign sets the
        // steer direction (verify empirically — wrong sign makes lat diverge). Clamped so it can't
        // overpower the heading term.
        if (lateral_gain_ > 0.0 && refl_valid_ && refl_range_ > reflector_trust_range_) {
          angular = clampd(angular + clampd(lat_sign_ * lateral_gain_ * refl_lateral_,
                                            -max_lateral_corr_, max_lateral_corr_),
                           -max_angular_, max_angular_);
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -v_reverse_;
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "REVERSE_IN: range %.3f | nyaw %+.1f | lat %+.3f | az %+.2f | %s | contact %u",
          refl_range_, refl_nyaw_ * 180.0 / M_PI, refl_lateral_, angular,
          imu_hold_active_ ? "IMU-hold" : "refl-square", contact_mask_);
        return;
      }

      case PUSH: {
        // Both prox confirmed -> full seat (firmware also zeros angular the moment dock_seated).
        if ((contact_mask_ & 0x03) == seat_contact_mask_) {
          RCLCPP_INFO(get_logger(), "PUSH: both prox seated (contact=%u).", contact_mask_);
          set_state(SEATED); publish_stop(); return;
        }
        // Bounced off entirely -> accept and stop (don't chase a lost contact).
        if ((contact_mask_ & 0x03) == 0) {
          RCLCPP_WARN(get_logger(), "PUSH: lost prox contact — stopping.");
          set_state(SEATED); publish_stop(); return;
        }
        // Give up nudging after the timeout -> accept the partial seat honestly.
        if ((t - push_start_).seconds() > push_timeout_s_) {
          RCLCPP_WARN(get_logger(), "PUSH: timeout %.1fs — accepting partial seat (contact=%u, left=%d right=%d).",
                      push_timeout_s_, contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0);
          set_state(SEATED); publish_stop(); return;
        }
        // Pivot the un-seated rear corner toward the dock. Derived sign: left-only latched -> CW
        // (az<0) drives the right-rear in; right-only -> CCW (az>0). Gentle reverse keeps pressure.
        const bool left_only = (contact_mask_ & 0x03) == 1;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -push_v_;
        cmd.angular.z = left_only ? -push_omega_ : push_omega_;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
          "PUSH: contact %u (left=%d right=%d) | az %+.2f | range %.3f | %.1fs",
          contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0,
          cmd.angular.z, refl_range_, (t - push_start_).seconds());
        return;
      }
    }
  }

  // params
  std::string reflector_topic_, imu_topic_, contact_topic_, cmd_vel_topic_, state_topic_;
  double control_hz_, v_reverse_, v_push_, throat_zone_, max_angular_, kp_heading_, heading_sign_, square_tol_,
         square_settle_s_, square_timeout_s_, square_omega_min_, square_omega_max_,
         reflector_trust_range_, kp_yaw_, lateral_gain_, lat_sign_,
         max_lateral_corr_, seat_zone_, seat_settle_s_, stuck_abort_s_, stall_eps_, seat_range_floor_,
         push_omega_, push_v_, push_timeout_s_,
         invalid_abort_s_, acquire_stable_s_, acquire_timeout_s_, overall_timeout_s_;
  bool square_only_;
  int seat_contact_mask_;

  // live inputs / state
  bool   refl_valid_{false}, have_imu_{false}, imu_hold_active_{false}, in_throat_{false};
  double refl_lateral_{0}, refl_range_{0}, refl_nyaw_{0}, yaw_{0}, imu_hold_yaw_{0}, best_range_{1e9};
  uint8_t contact_mask_{0};
  rclcpp::Time last_refl_time_{0,0,RCL_ROS_TIME}, last_valid_refl_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time acquire_start_{0,0,RCL_ROS_TIME}, valid_since_{0,0,RCL_ROS_TIME},
               square_ok_since_{0,0,RCL_ROS_TIME}, motion_start_{0,0,RCL_ROS_TIME},
               best_range_time_{0,0,RCL_ROS_TIME}, push_start_{0,0,RCL_ROS_TIME};
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
