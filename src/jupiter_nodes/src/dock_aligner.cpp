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
#include <std_msgs/msg/float32.hpp>
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
    confidence_topic_= declare_parameter<std::string>("confidence_topic", "/dock/reflector_confidence");
    imu_topic_       = declare_parameter<std::string>("imu_topic",       "/imu/data");
    contact_topic_   = declare_parameter<std::string>("contact_topic",   "/dock/contact");
    cmd_vel_topic_   = declare_parameter<std::string>("cmd_vel_topic",   "/cmd_vel");
    state_topic_     = declare_parameter<std::string>("state_topic",     "/dock/aligner_state");

    control_hz_       = declare_parameter("control_hz",        20.0);
    v_reverse_        = declare_parameter("v_reverse",         0.04);   // m/s gentler APPROACH for better lateral control
    v_push_           = declare_parameter("v_push",            0.14);   // m/s firmer TERMINAL push inside the throat (a gentle creep can't overcome steel-on-PLA rail friction; a hand-push at this force slides+squares it home)
    throat_zone_      = declare_parameter("throat_zone",       0.10);   // m — measured throat starts near 0.08 m from dock
    max_angular_      = declare_parameter("max_angular",       0.25);   // rad/s cap

    // Heading (dock-face squareness) control from the reflector, used in SQUARE and in the
    // far band of REVERSE_IN. heading_sign flips the rotation direction (verify with
    // square_only:=true if unsure): az = clamp(heading_sign * kp_heading * nyaw).
    kp_heading_       = declare_parameter("kp_heading",        0.9);
    heading_sign_     = declare_parameter("heading_sign",      1.0);
    square_tol_       = declare_parameter("square_tol_deg",    1.0) * M_PI / 180.0;   // ±1deg = ~1.75cm drift over 1m creep, inside a ±3-5cm funnel
    square_settle_s_  = declare_parameter("square_settle_s",   0.3);                  // hold in-band this long before locking heading (chassis settle)
    square_timeout_s_ = declare_parameter("square_timeout_s",  25.0);
    // In-place square uses a MIN rotational speed to reliably break floor scrub (a pure
    // proportional cmd stalls below scrub then overshoots — hunting). Proportional above the
    // floor for a gentle approach, hard-stop inside the deadband. (Gemini-recipe, 2026-07-27)
    square_omega_min_ = declare_parameter("square_omega_min",  0.15);   // rad/s floor to overcome scrub
    square_omega_max_ = declare_parameter("square_omega_max",  0.30);   // rad/s cap far from target
    square_diverge_step_deg_ = declare_parameter("square_diverge_step_deg", 1.5); // if |nyaw| grows by this much repeatedly, flip runtime square sign
    square_diverge_count_max_ = declare_parameter("square_diverge_count_max", 6);  // consecutive worsening samples before sign flip
    square_lateral_kp_ = declare_parameter("square_lateral_kp", 2.0);   // in-place yaw trim from lateral error when heading already in tolerance
    square_lateral_omega_max_ = declare_parameter("square_lateral_omega_max", 0.12); // keep recentering trim gentle
    square_recenter_v_ = declare_parameter("square_recenter_v", 0.025); // m/s tiny reverse while recentering lateral (avoids swivel-only oscillation)
    square_recenter_heading_kp_ = declare_parameter("square_recenter_heading_kp", 0.4); // keep heading near zero during curved recenter
    square_only_      = declare_parameter("square_only",       false); // SQUARE then stop (sign check)

    // Below this range the reflector angle is less reliable -> hold IMU yaw instead.
    // With short rails, keep this boundary outside the funnel mouth.
    reflector_trust_range_ = declare_parameter("reflector_trust_range", 0.20);
    kp_yaw_           = declare_parameter("kp_yaw",            0.8);    // IMU yaw-hold gain (close-in)
    heading_crosscheck_enable_ = declare_parameter("heading_crosscheck_enable", true);
    heading_crosscheck_thresh_deg_ = declare_parameter("heading_crosscheck_thresh_deg", 6.0);
    heading_crosscheck_count_max_ = declare_parameter("heading_crosscheck_count_max", 6);

    // Lateral centring trim: pull back to dock midpoint during reverse.
    lateral_gain_     = declare_parameter("lateral_gain",      0.6);
    close_lateral_gain_= declare_parameter("close_lateral_gain", 0.25);  // below close_range, let heading dominate and trim lateral more gently
    close_range_      = declare_parameter("close_range",       0.22);    // m
    lateral_setpoint_m_ = declare_parameter("lateral_setpoint_m", 0.01); // bias target (+left) to counter systematic right-offset arrivals
    close_lateral_deadband_m_ = declare_parameter("close_lateral_deadband_m", 0.012); // ignore tiny lateral residuals near center
    close_lateral_boost_start_ = declare_parameter("close_lateral_boost_start", 0.22); // begin anti-drift behavior near funnel mouth
    close_lateral_boost_error_ = declare_parameter("close_lateral_boost_error", 0.03); // m lateral error threshold for boost
    close_lateral_boost_gain_  = declare_parameter("close_lateral_boost_gain", 0.70);  // stronger centering gain when drifting
    close_reverse_slow_scale_  = declare_parameter("close_reverse_slow_scale", 0.65);   // slow down to regain centering authority
    lat_sign_         = declare_parameter("lat_sign",         -1.0);    // +lateral (dock left) -> steer right (CW)
    max_lateral_corr_ = declare_parameter("max_lateral_corr",  0.05);   // rad/s clamp on the cross-track contribution

    // Enter throat-push only when close AND reasonably centered/squared.
    throat_lateral_max_ = declare_parameter("throat_lateral_max", 0.04);
    throat_nyaw_max_    = declare_parameter("throat_nyaw_max_deg", 4.0) * M_PI / 180.0;
    throat_commit_lateral_max_ = declare_parameter("throat_commit_lateral_max", 0.03); // stricter gate to start push
    throat_commit_nyaw_max_    = declare_parameter("throat_commit_nyaw_max_deg", 3.0) * M_PI / 180.0;
    throat_gate_s_      = declare_parameter("throat_gate_s", 0.40);   // require continuous in-gate time before steering-off push
    mouth_realign_range_ = declare_parameter("mouth_realign_range", 0.18); // measured funnel mouth starts near 0.16 m from dock
    mouth_realign_lateral_max_ = declare_parameter("mouth_realign_lateral_max", 0.03);
    mouth_realign_nyaw_max_    = declare_parameter("mouth_realign_nyaw_max_deg", 4.0) * M_PI / 180.0;
    mouth_realign_omega_max_   = declare_parameter("mouth_realign_omega_max", 0.16);
    // Child-drive guard: in this corridor, if we are already near-square/near-center,
    // force straight reverse (az=0) to prevent unnecessary late banking.
    near_hold_start_range_ = declare_parameter("near_hold_start_range", 0.30);
    near_hold_end_range_   = declare_parameter("near_hold_end_range",   0.18);
    near_hold_lateral_max_ = declare_parameter("near_hold_lateral_max", 0.03);
    near_hold_nyaw_max_    = declare_parameter("near_hold_nyaw_max_deg", 3.0) * M_PI / 180.0;

    seat_contact_mask_= declare_parameter("seat_contact_mask", 3);      // both prox
    seat_zone_        = declare_parameter("seat_zone",         0.14);   // m — only seat-terminate inside the short funnel/throat region
    seat_settle_s_    = declare_parameter("seat_settle_s",     2.0);    // s of no-progress + contact -> seated
    stuck_abort_s_    = declare_parameter("stuck_abort_s",     4.0);    // s of no-progress + NO contact near seat -> abort (jammed skewed)
    stall_eps_        = declare_parameter("stall_eps",         0.008);  // m — progress smaller than this = stalled
    seat_range_floor_ = declare_parameter("seat_range_floor",  0.15);   // m — hard stop even with no contact

    // Safety option: disable terminal shove behaviors (throat push / unskew / push stages).
    // When false, controller will stop+abort on partial-seat conditions instead of repeated pushing.
    enable_terminal_push_ = declare_parameter("enable_terminal_push", false);

    // SINGLE-PROX UNSKEW: with short throat rails, run a brief wheel-biased pivot first
    // so the open corner swings in before falling back to staged PUSH.
    unskew_omega_      = declare_parameter("unskew_omega", 0.40);        // rad/s pivot command magnitude
    unskew_track_width_m_ = declare_parameter("unskew_track_width_m", 0.30); // wheel track used for one-wheel pivot v term
    unskew_v_cap_      = declare_parameter("unskew_v_cap", 0.06);        // m/s linear cap during unskew
    unskew_timeout_s_  = declare_parameter("unskew_timeout_s", 2.2);     // s before escalating to PUSH

    // FINAL PUSH: one prox latched but the other won't seat (robot arrived a touch skewed) -> pivot
    // the un-seated rear corner IN by rotating about the seated one, until BOTH latch. Sign is
    // DERIVED, not guessed: contact=1 (left seated, right open) -> CW (az<0) swings the right-rear
    // toward the dock; contact=2 (right seated) -> CCW (az>0). Firmware zeros angular the instant
    // both prox confirm (dock_seated), so this is self-terminating and can't over-rotate.
    push_omega_       = declare_parameter("push_omega",        0.22);   // rad/s pivot rate for the seat nudge
    push_v_           = declare_parameter("push_v",            0.03);   // m/s gentle reverse kept during the nudge (firmware may gate it)
    push_timeout_s_   = declare_parameter("push_timeout_s",    6.0);    // s — allow boost + pivot-focus stages before aborting partial seat
    push_boost_after_s_ = declare_parameter("push_boost_after_s", 1.5); // if still single-prox after this, increase pressure/torque
    push_omega_boost_   = declare_parameter("push_omega_boost",   0.28);
    push_v_boost_       = declare_parameter("push_v_boost",       0.05);
    push_pivot_after_s_ = declare_parameter("push_pivot_after_s", 3.0); // if still single-prox, prioritize corner swing-in over linear shove
    push_omega_pivot_   = declare_parameter("push_omega_pivot",   0.36);
    push_v_pivot_       = declare_parameter("push_v_pivot",       0.01);

    invalid_abort_s_  = declare_parameter("invalid_abort_s",   1.0);
    acquire_stable_s_ = declare_parameter("acquire_stable_s",  0.5);
    acquire_timeout_s_= declare_parameter("acquire_timeout_s", 10.0);
    overall_timeout_s_= declare_parameter("overall_timeout_s", 90.0);

    // Reflector quality gates.
    min_confidence_    = declare_parameter("min_confidence",    0.70);
    min_valid_frames_  = declare_parameter("min_valid_frames",  4);
    reflector_stale_s_ = declare_parameter("reflector_stale_s", 0.25);
    center_tolerance_m_= declare_parameter("center_tolerance_m", 0.05);

    cmd_pub_   = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(1).transient_local());

    refl_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      reflector_topic_, 10, std::bind(&DockAligner::on_reflector, this, std::placeholders::_1));
    conf_sub_ = create_subscription<std_msgs::msg::Float32>(
      confidence_topic_, 10, std::bind(&DockAligner::on_confidence, this, std::placeholders::_1));
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
  enum State { IDLE, ACQUIRE, SQUARE, REVERSE_IN, UNSKEW, PUSH, SEATED, ABORT };
  const char* state_name(State s) const {
    switch (s) { case IDLE: return "IDLE"; case ACQUIRE: return "ACQUIRE"; case SQUARE: return "SQUARE";
                 case REVERSE_IN: return "REVERSE_IN"; case UNSKEW: return "UNSKEW"; case PUSH: return "PUSH";
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

    // Keep runtime validity alive on fresh valid geometry even if confidence dips briefly.
    if (control_pose_ok()) {
      last_valid_refl_time_ = last_refl_time_;
    }

    if (pose_qualified()) {
      ++consecutive_good_frames_;
    } else {
      consecutive_good_frames_ = 0;
    }
  }
  void on_confidence(const std_msgs::msg::Float32::SharedPtr m) {
    refl_confidence_ = static_cast<double>(m->data);
    last_conf_time_ = now();
  }
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr m) {
    const auto& q = m->orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_imu_ = true;
  }
  void on_contact(const std_msgs::msg::UInt8::SharedPtr m) { contact_mask_ = m->data; }

  void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (state_ == ACQUIRE || state_ == SQUARE || state_ == REVERSE_IN || state_ == UNSKEW || state_ == PUSH) {
      res->success = false; res->message = "already running"; return;
    }
    if (!have_imu_) { res->success = false; res->message = "no IMU yet — micro-ROS agent up?"; return; }
    acquire_start_ = now();
    valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    in_throat_ = false;
    unskew_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    throat_ready_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    consecutive_good_frames_ = 0;
    refl_confidence_ = 0.0;
    set_state(ACQUIRE);
    res->success = true; res->message = "acquiring dock lock...";
  }
  void on_cancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    abort_reason_ = "cancelled by service"; set_state(ABORT);
    res->success = true; res->message = "aborting -> stop";
  }

  void publish_stop() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }
  bool fresh_refl() const {
    return last_refl_time_.nanoseconds() != 0 &&
           (now() - last_refl_time_).seconds() <= reflector_stale_s_;
  }
  bool fresh_conf() const {
    return last_conf_time_.nanoseconds() != 0 &&
           (now() - last_conf_time_).seconds() <= reflector_stale_s_;
  }
  bool pose_qualified() const {
    return refl_valid_ && fresh_refl() && fresh_conf() && (refl_confidence_ >= min_confidence_);
  }
  bool control_pose_ok() const {
    // For close-range steering/centering we trust fresh valid geometry even when confidence dips.
    return refl_valid_ && fresh_refl();
  }
  bool stable_lock() const {
    return pose_qualified() && consecutive_good_frames_ >= min_valid_frames_;
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
        const bool fresh = stable_lock();
        if (!fresh) valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        else if (valid_since_.nanoseconds() == 0) valid_since_ = t;
        publish_stop();
        if (valid_since_.nanoseconds() != 0 && (t - valid_since_).seconds() >= acquire_stable_s_) {
          square_ok_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          square_runtime_sign_ = heading_sign_;
          square_diverge_count_ = 0;
          last_square_nyaw_ = std::numeric_limits<double>::quiet_NaN();
          set_state(SQUARE);
          RCLCPP_INFO(get_logger(), "locked: range %.3f lateral %+.3f nyaw %+.1f deg -> squaring.",
                      refl_range_, refl_lateral_, refl_nyaw_ * 180.0 / M_PI);
        } else if ((t - acquire_start_).seconds() > acquire_timeout_s_) {
          abort_reason_ = "no stable dock lock"; set_state(ABORT);
        }
        return;
      }

      case SQUARE: {
        if (!fresh_refl() || !fresh_conf()) {
          abort_reason_ = "reflector/confidence stale during square";
          set_state(ABORT); publish_stop(); return;
        }
        if ((t - last_valid_refl_time_).seconds() > invalid_abort_s_) {
          abort_reason_ = "pose invalid during square"; set_state(ABORT); publish_stop(); return;
        }
        // Rotate in place to null the dock-face heading error. No linear motion = full runway,
        // no collision risk (this is also what square_only tests for sign).
        const bool squared = std::fabs(refl_nyaw_) <= square_tol_;
        const double lateral_err_raw = refl_lateral_ - lateral_setpoint_m_;
        const double lateral_err = (std::fabs(lateral_err_raw) <= close_lateral_deadband_m_) ? 0.0 : lateral_err_raw;

        const bool in_near_hold = control_pose_ok() && refl_range_ > near_hold_end_range_ &&
                                  refl_range_ < near_hold_start_range_;
        const bool near_hold_ok = std::fabs(lateral_err_raw) <= near_hold_lateral_max_ &&
                                  std::fabs(refl_nyaw_) <= near_hold_nyaw_max_;
        if (in_near_hold && near_hold_ok && (contact_mask_ & 0x03) == 0) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = -v_reverse_;
          cmd.angular.z = 0.0;
          cmd_pub_->publish(cmd);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
            "NEAR HOLD: straight reverse lock (range %.3f | lat_err_raw %+.3f | nyaw %+.1fdeg)",
            refl_range_, lateral_err_raw, refl_nyaw_ * 180.0 / M_PI);
          return;
        }
        const bool lateral_ok = std::fabs(lateral_err) <= center_tolerance_m_;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        if (squared) {
          if (lateral_ok) {
            cmd.angular.z = 0.0;   // hard-stop only when heading+lateral are both ready
            if (square_ok_since_.nanoseconds() == 0) square_ok_since_ = t;   // start settle timer
          } else {
            // Heading is good, but lateral is not. Do a tiny curved reverse to re-center;
            // swivel-only behavior can hunt left/right without reducing cross-track error.
            cmd.linear.x = -square_recenter_v_;
            const double lateral_term = clampd(lat_sign_ * square_lateral_kp_ * lateral_err,
                                               -square_lateral_omega_max_, square_lateral_omega_max_);
            const double heading_term = clampd(heading_sign_ * square_recenter_heading_kp_ * refl_nyaw_,
                                               -0.08, 0.08);
            cmd.angular.z = clampd(lateral_term + heading_term,
                                   -square_lateral_omega_max_, square_lateral_omega_max_);
            square_ok_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "SQUARE heading in tol but lateral_err %+.3f m (raw %+.3f, set %.3f) exceeds %.3f m. Recentering arc (v %.3f az %+.2f).",
              lateral_err, refl_lateral_, lateral_setpoint_m_, center_tolerance_m_, cmd.linear.x, cmd.angular.z);
          }
        } else {
          // Proportional above the scrub floor, capped; the MIN speed guarantees it actually
          // rotates instead of stalling below breakaway then overshooting (the old hunting).
          const double mag = clampd(kp_heading_ * std::fabs(refl_nyaw_),
                                    square_omega_min_, square_omega_max_);
          cmd.angular.z = square_runtime_sign_ * std::copysign(mag, refl_nyaw_);

          // If heading error repeatedly worsens with the same sign, flip square rotation sign.
          if (std::isfinite(last_square_nyaw_)) {
            const double step = square_diverge_step_deg_ * M_PI / 180.0;
            const bool same_side = (refl_nyaw_ * last_square_nyaw_) > 0.0;
            const bool worsening = std::fabs(refl_nyaw_) > std::fabs(last_square_nyaw_) + step;
            if (same_side && worsening) {
              ++square_diverge_count_;
            } else {
              square_diverge_count_ = std::max(0, square_diverge_count_ - 1);
            }
            if (square_diverge_count_ >= square_diverge_count_max_) {
              square_runtime_sign_ = -square_runtime_sign_;
              square_diverge_count_ = 0;
              RCLCPP_WARN(get_logger(),
                "SQUARE divergence detected (nyaw %+.1fdeg). Flipping runtime square sign to %+.0f.",
                refl_nyaw_ * 180.0 / M_PI, square_runtime_sign_);
            }
          }
          last_square_nyaw_ = refl_nyaw_;
          square_ok_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());   // reset settle timer
        }
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "SQUARE: nyaw %+.1f deg | az %+.2f | range %.3f%s",
          refl_nyaw_ * 180.0 / M_PI, cmd.angular.z, refl_range_, squared ? "  [in tol]" : "");

        if (square_ok_since_.nanoseconds() != 0 && (t - square_ok_since_).seconds() >= square_settle_s_) {
          if (square_only_) { RCLCPP_INFO(get_logger(), "square_only: squared, stopping."); set_state(SEATED); publish_stop(); return; }
          best_range_ = 1e9; best_range_time_ = t;
          // Lock IMU heading exactly at square handoff for reverse-time consistency checks.
          imu_hold_yaw_ = yaw_;
          imu_hold_active_ = true;
          heading_crosscheck_bad_count_ = 0;
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
        if (pose_qualified() && refl_range_ > 0.0 && refl_range_ < seat_range_floor_) {
          if ((contact_mask_ & 0x03) != 0) {
            RCLCPP_WARN(get_logger(), "range %.3f below floor with contact=%u — stop (rails/firmware take over).",
                        refl_range_, contact_mask_);
            set_state(SEATED); publish_stop(); return;
          }
          abort_reason_ = "range floor reached with no prox contact";
          RCLCPP_WARN(get_logger(), "range %.3f below floor but no prox contact — abort.", refl_range_);
          set_state(ABORT); publish_stop(); return;
        }
        // Partial-seat / stall termination: in the seat zone, touching at least one prox, and no
        // further progress for seat_settle_s -> stop (don't grind against the dock).
        if (refl_range_ < best_range_ - stall_eps_) { best_range_ = refl_range_; best_range_time_ = t; }
        if (refl_range_ < seat_zone_ && (contact_mask_ & 0x03) != 0 &&
            (t - best_range_time_).seconds() > seat_settle_s_) {
          if (!enable_terminal_push_) {
            abort_reason_ = "terminal push disabled on partial seat";
            RCLCPP_WARN(get_logger(),
                        "partial seat detected (contact=%u) but enable_terminal_push=false — stopping.",
                        contact_mask_);
            set_state(ABORT); publish_stop(); return;
          }
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
        if (!fresh_refl() || !fresh_conf()) {
          abort_reason_ = "reflector/confidence stale during reverse";
          set_state(ABORT); publish_stop(); return;
        }
        if ((t - last_valid_refl_time_).seconds() > invalid_abort_s_) {
          abort_reason_ = "dock pose invalid too long"; set_state(ABORT); publish_stop(); return;
        }
        if ((t - motion_start_).seconds() > overall_timeout_s_) {
          abort_reason_ = "overall reverse timeout"; set_state(ABORT); publish_stop(); return;
        }

        const double lateral_err_raw = refl_lateral_ - lateral_setpoint_m_;
        const double lateral_err = (std::fabs(lateral_err_raw) <= close_lateral_deadband_m_) ? 0.0 : lateral_err_raw;

        // One-prox capture inside throat is a skewed rail-hook condition. Attempt controlled
        // PUSH recovery (pivot the unseated corner in). Success still requires both prox.
        if (control_pose_ok() && refl_range_ > 0.0 && refl_range_ < throat_zone_ &&
            (contact_mask_ & 0x03) != 0 && (contact_mask_ & 0x03) != seat_contact_mask_) {
          if (!enable_terminal_push_) {
            abort_reason_ = "terminal push disabled on throat single-prox";
            RCLCPP_WARN(get_logger(),
              "single prox latch in throat (range %.3f, contact=%u) and enable_terminal_push=false — stopping.",
              refl_range_, contact_mask_);
            set_state(ABORT); publish_stop(); return;
          }
          RCLCPP_WARN(get_logger(),
            "single prox latch in throat (range %.3f, contact=%u, lat %+.3f, nyaw %+.1fdeg) — trying UNSKEW first.",
            refl_range_, contact_mask_, refl_lateral_, refl_nyaw_ * 180.0 / M_PI);
          unskew_start_ = t;
          set_state(UNSKEW);
          return;
        }

        // Near the funnel mouth, do not keep reversing if still skewed/off-center.
        // Hold and re-square first to avoid grinding into a rail-jam state.
        const bool mouth_zone = control_pose_ok() && refl_range_ > throat_zone_ && refl_range_ < mouth_realign_range_;
        if (mouth_zone &&
            (std::fabs(lateral_err) > mouth_realign_lateral_max_ || std::fabs(refl_nyaw_) > mouth_realign_nyaw_max_)) {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = 0.0;
          const double lateral_term = clampd(lat_sign_ * 0.9 * lateral_err, -0.08, 0.08);
          const double heading_term = clampd(heading_sign_ * kp_heading_ * refl_nyaw_, -mouth_realign_omega_max_, mouth_realign_omega_max_);
          cmd.angular.z = clampd(heading_term + lateral_term, -mouth_realign_omega_max_, mouth_realign_omega_max_);
          cmd_pub_->publish(cmd);
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            "MOUTH REALIGN: range %.3f | lat_err %+.3f (raw %+.3f) | nyaw %+.1fdeg | az %+.2f",
            refl_range_, lateral_err, lateral_err_raw, refl_nyaw_ * 180.0 / M_PI, cmd.angular.z);
          return;
        }

        // --- TERMINAL THROAT PUSH ---
        // Once the rear is in the funnel throat, STOP steering (angular=0) and push firmly straight.
        // The mechanical guide rails do the squaring; a gentle creep can't beat steel-on-PLA friction
        // (a hand-push at this force slid it home). Steering here would only FIGHT the rails. Latched,
        // so a reflector dropout at close range can't kick us back to steering mid-throat.
        const bool throat_ready =
          control_pose_ok() && refl_range_ > 0.0 && refl_range_ < throat_zone_ &&
          std::fabs(refl_lateral_ - lateral_setpoint_m_) <= throat_commit_lateral_max_ &&
          std::fabs(refl_nyaw_) <= throat_commit_nyaw_max_;

        if (throat_ready) {
          if (throat_ready_since_.nanoseconds() == 0) {
            throat_ready_since_ = t;
          }
        } else {
          throat_ready_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }

        if (!throat_ready && control_pose_ok() && refl_range_ > 0.0 && refl_range_ < throat_zone_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            "close but not centered for throat-push: range %.3f lat %+.3f nyaw %+.1fdeg",
            refl_range_, refl_lateral_, refl_nyaw_ * 180.0 / M_PI);
        }
        if (throat_ready_since_.nanoseconds() != 0 &&
            (t - throat_ready_since_).seconds() >= throat_gate_s_) {
          if (!enable_terminal_push_) {
            abort_reason_ = "terminal push disabled at throat handover";
            RCLCPP_WARN(get_logger(),
              "throat handover reached (range %.3f) but enable_terminal_push=false — stopping.",
              refl_range_);
            set_state(ABORT); publish_stop(); return;
          }
          in_throat_ = true;
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x  = -v_push_;
          cmd.angular.z = 0.0;
          cmd_pub_->publish(cmd);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
            "THROAT PUSH: range %.3f | v %.2f | steering OFF (rails square) | contact %u",
            refl_range_, v_push_, contact_mask_);
          return;
        }

        // --- heading law: LiDAR-nyaw primary, but cross-check against IMU lock from square handoff ---
        double angular;
        if (control_pose_ok()) {
          const double imu_err = wrap_pi(imu_hold_yaw_ - yaw_);
          const double mismatch = std::fabs(wrap_pi(refl_nyaw_ - imu_err));
          const double mismatch_thresh = heading_crosscheck_thresh_deg_ * M_PI / 180.0;
          bool use_imu_guard = false;
          if (heading_crosscheck_enable_) {
            if (mismatch > mismatch_thresh) {
              ++heading_crosscheck_bad_count_;
            } else {
              heading_crosscheck_bad_count_ = std::max(0, heading_crosscheck_bad_count_ - 1);
            }
            use_imu_guard = heading_crosscheck_bad_count_ >= heading_crosscheck_count_max_;
            if (use_imu_guard) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "HEADING XCHECK: LiDAR/IMU mismatch %.1fdeg > %.1fdeg (count=%d) -> IMU guard heading.",
                mismatch * 180.0 / M_PI, heading_crosscheck_thresh_deg_, heading_crosscheck_bad_count_);
            }
          }
          if (use_imu_guard) {
            angular = clampd(kp_yaw_ * imu_err, -max_angular_, max_angular_);
            imu_hold_active_ = true;
          } else {
            angular = heading_cmd();
            imu_hold_active_ = false;
          }
        } else {
          if (!imu_hold_active_) { imu_hold_yaw_ = yaw_; imu_hold_active_ = true; } // capture at boundary
          angular = clampd(kp_yaw_ * wrap_pi(imu_hold_yaw_ - yaw_), -max_angular_, max_angular_);
        }
        // CROSS-TRACK correction: steer the rear onto the dock's normal axis so a lateral
        // staging offset gets pulled in instead of missing the funnel.
        double lat_gain = (control_pose_ok() && refl_range_ > 0.0 && refl_range_ < close_range_)
                            ? close_lateral_gain_
                            : lateral_gain_;
        const bool close_drift_window = control_pose_ok() && refl_range_ > 0.0 &&
                                        refl_range_ < close_lateral_boost_start_ &&
                std::fabs(lateral_err) >= close_lateral_boost_error_;
        if (close_drift_window) {
          // Near 0.5 m, if lateral error is growing, prioritize centering over speed.
          lat_gain = std::max(lat_gain, close_lateral_boost_gain_);
        }
        if (lat_gain > 0.0 && control_pose_ok() && !in_throat_) {
          angular = clampd(angular + clampd(lat_sign_ * lat_gain * lateral_err,
                                            -max_lateral_corr_, max_lateral_corr_),
                           -max_angular_, max_angular_);
        }

        geometry_msgs::msg::Twist cmd;
        double v_cmd = v_reverse_;
        if (close_drift_window) {
          v_cmd = std::max(0.02, v_reverse_ * close_reverse_slow_scale_);
        }
        cmd.linear.x = -v_cmd;
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "REVERSE_IN: range %.3f | nyaw %+.1f | lat %+.3f (err %+.3f,raw %+.3f,set %.3f) | az %+.2f | lat_gain %.2f | v %.2f | %s | contact %u",
          refl_range_, refl_nyaw_ * 180.0 / M_PI, refl_lateral_, lateral_err, lateral_err_raw, lateral_setpoint_m_, angular,
          lat_gain, v_cmd, imu_hold_active_ ? "IMU-hold" : "refl-square", contact_mask_);
        return;
      }

      case UNSKEW: {
        if (!enable_terminal_push_) {
          abort_reason_ = "unskew disabled by parameter";
          RCLCPP_WARN(get_logger(), "UNSKEW entered while enable_terminal_push=false — aborting.");
          set_state(ABORT); publish_stop(); return;
        }
        // One-prox correction for short-throat geometry: pivot around seated side first.
        if ((contact_mask_ & 0x03) == seat_contact_mask_) {
          RCLCPP_INFO(get_logger(), "UNSKEW: both prox seated (contact=%u).", contact_mask_);
          set_state(SEATED); publish_stop(); return;
        }
        if ((contact_mask_ & 0x03) == 0) {
          abort_reason_ = "unskew lost prox contact";
          RCLCPP_WARN(get_logger(), "UNSKEW: lost prox contact — aborting.");
          set_state(ABORT); publish_stop(); return;
        }
        if ((t - unskew_start_).seconds() > unskew_timeout_s_) {
          RCLCPP_WARN(get_logger(),
            "UNSKEW: timeout %.1fs with partial seat (contact=%u) — escalating to PUSH.",
            unskew_timeout_s_, contact_mask_);
          push_start_ = t;
          set_state(PUSH);
          return;
        }

        const uint8_t contact = (contact_mask_ & 0x03);
        const bool left_only = (contact == 1);
        const bool right_only = (contact == 2);
        if (!left_only && !right_only) {
          RCLCPP_WARN(get_logger(), "UNSKEW: unexpected contact mask=%u — escalating to PUSH.", contact_mask_);
          push_start_ = t;
          set_state(PUSH);
          return;
        }

        // Choose v so seated-side wheel stays near zero: v ~= |omega|*track/2.
        const double omega_mag = std::fabs(unskew_omega_);
        const double one_wheel_v = 0.5 * unskew_track_width_m_ * omega_mag;
        const double v_cmd = std::min(unskew_v_cap_, one_wheel_v);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -v_cmd;
        cmd.angular.z = left_only ? -omega_mag : omega_mag;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
          "UNSKEW: contact %u (left=%d right=%d) | az %+.2f | v %.2f | range %.3f | %.1fs",
          contact_mask_, left_only ? 1 : 0, right_only ? 1 : 0,
          cmd.angular.z, v_cmd, refl_range_, (t - unskew_start_).seconds());
        return;
      }

      case PUSH: {
        if (!enable_terminal_push_) {
          abort_reason_ = "push disabled by parameter";
          RCLCPP_WARN(get_logger(), "PUSH entered while enable_terminal_push=false — aborting.");
          set_state(ABORT); publish_stop(); return;
        }
        // Both prox confirmed -> full seat (firmware also zeros angular the moment dock_seated).
        if ((contact_mask_ & 0x03) == seat_contact_mask_) {
          RCLCPP_INFO(get_logger(), "PUSH: both prox seated (contact=%u).", contact_mask_);
          set_state(SEATED); publish_stop(); return;
        }
        // Bounced off entirely during push -> abort (report as failed docking).
        if ((contact_mask_ & 0x03) == 0) {
          abort_reason_ = "push lost prox contact";
          RCLCPP_WARN(get_logger(), "PUSH: lost prox contact — aborting.");
          set_state(ABORT); publish_stop(); return;
        }
        // Give up nudging after timeout -> abort (partial seat is not success).
        if ((t - push_start_).seconds() > push_timeout_s_) {
          abort_reason_ = "push timeout partial seat";
          RCLCPP_WARN(get_logger(), "PUSH: timeout %.1fs with partial seat (contact=%u, left=%d right=%d) — abort.",
                      push_timeout_s_, contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0);
          set_state(ABORT); publish_stop(); return;
        }
        // Pivot the un-seated rear corner toward the dock. Derived sign: left-only latched -> CW
        // (az<0) drives the right-rear in; right-only -> CCW (az>0). Gentle reverse keeps pressure.
        const bool left_only = (contact_mask_ & 0x03) == 1;
        const double push_elapsed = (t - push_start_).seconds();
        enum class PushStage { BASE, BOOST, PIVOT };
        PushStage stage = PushStage::BASE;
        if (push_elapsed >= push_pivot_after_s_) {
          stage = PushStage::PIVOT;
        } else if (push_elapsed >= push_boost_after_s_) {
          stage = PushStage::BOOST;
        }

        double omega_cmd = push_omega_;
        double v_cmd = push_v_;
        const char* stage_label = "";
        if (stage == PushStage::BOOST) {
          omega_cmd = push_omega_boost_;
          v_cmd = push_v_boost_;
          stage_label = " [boost]";
        } else if (stage == PushStage::PIVOT) {
          // Pivot-focus phase: lower linear shove, higher angular swing to pull the open corner in.
          omega_cmd = push_omega_pivot_;
          v_cmd = push_v_pivot_;
          stage_label = " [pivot]";
        }
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -v_cmd;
        cmd.angular.z = left_only ? -omega_cmd : omega_cmd;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
          "PUSH: contact %u (left=%d right=%d) | az %+.2f | v %.2f | range %.3f | %.1fs%s",
          contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0,
          cmd.angular.z, v_cmd, refl_range_, push_elapsed, stage_label);
        return;
      }
    }
  }

  // params
    std::string reflector_topic_, confidence_topic_, imu_topic_, contact_topic_, cmd_vel_topic_, state_topic_;
  double control_hz_, v_reverse_, v_push_, throat_zone_, max_angular_, kp_heading_, heading_sign_, square_tol_,
      square_settle_s_, square_timeout_s_, square_omega_min_, square_omega_max_, square_diverge_step_deg_, square_lateral_kp_, square_lateral_omega_max_,
      square_recenter_v_, square_recenter_heading_kp_,
      reflector_trust_range_, kp_yaw_, heading_crosscheck_thresh_deg_, lateral_gain_, close_lateral_gain_, close_range_,
      lateral_setpoint_m_, close_lateral_deadband_m_, close_lateral_boost_start_, close_lateral_boost_error_, close_lateral_boost_gain_, close_reverse_slow_scale_, lat_sign_,
      max_lateral_corr_, throat_gate_s_, seat_zone_, seat_settle_s_, stuck_abort_s_, stall_eps_, seat_range_floor_,
      mouth_realign_range_, mouth_realign_lateral_max_, mouth_realign_nyaw_max_, mouth_realign_omega_max_,
      near_hold_start_range_, near_hold_end_range_, near_hold_lateral_max_, near_hold_nyaw_max_,
      unskew_omega_, unskew_track_width_m_, unskew_v_cap_, unskew_timeout_s_,
        push_omega_, push_v_, push_timeout_s_, push_boost_after_s_, push_omega_boost_, push_v_boost_,
      push_pivot_after_s_, push_omega_pivot_, push_v_pivot_,
      invalid_abort_s_, acquire_stable_s_, acquire_timeout_s_, overall_timeout_s_,
      min_confidence_, reflector_stale_s_, center_tolerance_m_,
      throat_lateral_max_, throat_nyaw_max_, throat_commit_lateral_max_, throat_commit_nyaw_max_;
  bool square_only_, enable_terminal_push_, heading_crosscheck_enable_;
    int seat_contact_mask_, min_valid_frames_, square_diverge_count_max_, heading_crosscheck_count_max_;

  // live inputs / state
  bool   refl_valid_{false}, have_imu_{false}, imu_hold_active_{false}, in_throat_{false};
    double refl_lateral_{0}, refl_range_{0}, refl_nyaw_{0}, refl_confidence_{0.0},
      yaw_{0}, imu_hold_yaw_{0}, best_range_{1e9}, square_runtime_sign_{1.0}, last_square_nyaw_{std::numeric_limits<double>::quiet_NaN()};
    int    consecutive_good_frames_{0}, square_diverge_count_{0}, heading_crosscheck_bad_count_{0};
  uint8_t contact_mask_{0};
    rclcpp::Time last_refl_time_{0,0,RCL_ROS_TIME}, last_conf_time_{0,0,RCL_ROS_TIME},
       last_valid_refl_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time acquire_start_{0,0,RCL_ROS_TIME}, valid_since_{0,0,RCL_ROS_TIME},
               square_ok_since_{0,0,RCL_ROS_TIME}, motion_start_{0,0,RCL_ROS_TIME},
               best_range_time_{0,0,RCL_ROS_TIME}, unskew_start_{0,0,RCL_ROS_TIME}, push_start_{0,0,RCL_ROS_TIME},
               throat_ready_since_{0,0,RCL_ROS_TIME};
  State state_{IDLE};
  std::string abort_reason_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr refl_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            conf_sub_;
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
