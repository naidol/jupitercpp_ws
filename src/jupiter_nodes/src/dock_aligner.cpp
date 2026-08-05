// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_aligner — PURE-PURSUIT reverse docking onto the charging dock (two-strip redesign).
//
// FLOW: ACQUIRE (stable two-strip lock) -> AIM (rotate in place to point the REAR at the
// carrot ON the centreline) -> REVERSE_IN (straight cruise segments; drift -> stop and
// re-AIM) -> THROAT (blind firm push, rails absorb the residual angle) -> PUSH (corner nudge
// if one prox) -> SEATED.   *** STATUS 2026-08-05: BUILT, NEVER TESTED. ***
//
// PRIORITY (Logan, 2026-08-05): CENTRE FIRST, square second — and square only roughly. The
// throat/rails exist precisely to absorb the last few degrees of ANGLE; what they can never
// absorb is arriving OFF-CENTRE (missing the mouth). Every earlier scheme had this backwards:
// "square to the dock face" (reflector nyaw) controls angle only — a robot can be perfectly
// square and still 6 cm off the centreline ("square != centred"). The old close-in IMU
// yaw-hold then froze that flawed heading and reversed blind, so any offset/caster transient
// was locked in -> banked left/right -> missed the 100 mm throat. THE FIX: the live target is
// the CENTRELINE. A carrot point sits ON the dock's outward-normal axis, a fixed lookahead L
// along-axis toward the dock (passing BEHIND the face at the end — aim deep at the pogo
// centre). Driving the REAR at the carrot converges lateral offset first and heading with it
// (e_y ~ exp(-travel/L)) — centring is the act, squareness is the byproduct, the throat does
// the rest. Stable by construction: ONE loop (the bolted-on cross-track trim oscillated; gone).
//
// This is only possible because dock_reflector now reads TWO vertical retro strips (250 mm
// baseline): centre = midpoint of two tight centroids, angle = perpendicular to the baseline
// — both proven stable from 1 m all the way to the throat (the single-strip PCA angle went
// to mush below ~0.35 m, which is what forced the IMU-hold hack in the first place).
//
// STOP-AND-RE-AIM PILOT (final form, 2026-08-05): this drivetrain has NO usable steering
// authority while rolling at crawl speed — proven live: a SATURATED correction (az 0.20 for
// 2+ s at v 0.14) produced no rotation while the caster kept pulling the tail (the wheel-speed
// differential ±3 cm/s cannot overpower the loaded caster's side-force). Every in-motion
// steering scheme (P-loop, damped, pursuit, pulse) failed on exactly this. What the robot DOES
// do reliably — proven every single run — is (1) rotate in place to ±1 deg and (2) reverse
// dead straight open-loop (angular=0 -> gyro flat). So the controller uses ONLY those two
// primitives, like a driver backing a trailer:
//   AIM in place at the centreline carrot -> CRUISE straight, angular EXACTLY 0, monitored on
//   the IMU (navigator: each reflector sample refreshes yaw_des = yaw + aim, folding lateral
//   drift into the target) -> drift beyond drift_deadband? STOP, re-AIM in place, cruise
//   again -> inside commit_range: no more stops, straight into the throat, rails take the
//   residual angle. Each stop-and-re-aim also settles the caster (the pre-flip for free).
// A reflector dropout mid-cruise degrades gracefully: yaw_des freezes -> pure IMU-hold line.
//
// SAFETY: low speed caps; publishes 20 Hz (inside the 400 ms firmware watchdog); firmware
// seat-reflex is an independent backstop; aborts on lost pose (outside the throat) / timeout;
// never moves on launch (service-triggered). `square_only:=true` does SQUARE then stops —
// use it to verify rotation sign safely before ever reversing.
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
constexpr int D_ALONG   = 1;
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

    // Defaults below are the PROVEN values from the 2026-08 tuning sessions — baked in so a
    // plain `ros2 run` needs no parameter soup.
    control_hz_       = declare_parameter("control_hz",        20.0);
    v_reverse_        = declare_parameter("v_reverse",         0.14);   // m/s — 0.05..0.08 sat below wheel stiction: steering was commanded but not executed
    v_push_           = declare_parameter("v_push",            0.14);   // m/s firm TERMINAL push (steel-on-PLA rail friction needs it)
    throat_zone_      = declare_parameter("throat_zone",       0.28);   // m — at/below this: steering OFF, rails do the squaring
    max_angular_      = declare_parameter("max_angular",       0.20);   // rad/s cap

    // Pure-pursuit homing. heading_sign flips rotation direction (verify with square_only).
    kp_heading_       = declare_parameter("kp_heading",        1.2);    // gain on the rear-aim error
    heading_sign_     = declare_parameter("heading_sign",      1.0);
    pursuit_lookahead_= declare_parameter("pursuit_lookahead", 0.35);   // m carrot distance along the dock axis; smaller = tighter centring, larger = gentler

    // STOP-AND-RE-AIM pilot: while rolling the command is angular = EXACTLY 0, always (there
    // is no in-motion steering authority on this drivetrain — see header). Drift beyond the
    // deadband -> stop, re-AIM in place (the proven rotation), cruise again. Inside
    // commit_range no more stops: straight into the throat, the rails absorb the residual.
    drift_deadband_   = declare_parameter("drift_deadband_deg", 3.0) * M_PI / 180.0;
    commit_range_     = declare_parameter("commit_range",       0.45);   // m — final re-aim happens at/above this; below it, committed

    square_tol_       = declare_parameter("square_tol_deg",    1.0) * M_PI / 180.0;
    square_settle_s_  = declare_parameter("square_settle_s",   0.3);    // hold in-band this long before reversing (chassis settle)
    square_timeout_s_ = declare_parameter("square_timeout_s",  25.0);
    // In-place rotation uses a MIN speed to break floor scrub (pure proportional stalls below
    // scrub then overshoots — hunting). Proportional above the floor, hard-stop in the deadband.
    square_omega_min_ = declare_parameter("square_omega_min",  0.15);
    square_omega_max_ = declare_parameter("square_omega_max",  0.30);
    square_only_      = declare_parameter("square_only",       false);  // SQUARE then stop (sign check)

    seat_contact_mask_= declare_parameter("seat_contact_mask", 3);      // both prox
    seat_zone_        = declare_parameter("seat_zone",         0.34);   // m — rear touches the dock at ~0.31 reflector range; 0.30 left a dead band where nothing fired
    seat_settle_s_    = declare_parameter("seat_settle_s",     2.0);    // s no-progress + contact -> PUSH
    stuck_abort_s_    = declare_parameter("stuck_abort_s",     4.0);    // s no-progress + NO contact near seat -> abort
    stall_eps_        = declare_parameter("stall_eps",         0.008);  // m — progress smaller than this = stalled
    seat_range_floor_ = declare_parameter("seat_range_floor",  0.15);   // m — hard stop even with no contact

    // FINAL PUSH: one prox latched, the other won't seat -> pivot the open corner in about the
    // seated one. Sign DERIVED: contact=1 (left seated) -> CW; contact=2 (right) -> CCW.
    // Firmware zeros angular the instant both prox confirm, so this self-terminates.
    push_omega_       = declare_parameter("push_omega",        0.22);
    push_v_           = declare_parameter("push_v",            0.03);
    push_timeout_s_   = declare_parameter("push_timeout_s",    4.0);

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
      "dock_aligner (STOP-AND-RE-AIM, UNTESTED) ready. v_rev=%.2f max_ang=%.2f kp=%.2f(sign %+.0f) "
      "L=%.2f drift=%.1fdeg commit=%.2f square_only=%d. Call /dock/align_start.",
      v_reverse_, max_angular_, kp_heading_, heading_sign_, pursuit_lookahead_,
      drift_deadband_*180.0/M_PI, commit_range_, square_only_ ? 1 : 0);
  }

private:
  enum State { IDLE, ACQUIRE, AIM, REVERSE_IN, PUSH, SEATED, ABORT };
  const char* state_name(State s) const {
    switch (s) { case IDLE: return "IDLE"; case ACQUIRE: return "ACQUIRE"; case AIM: return "AIM";
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
    refl_along_   = m->data[D_ALONG];
    refl_lateral_ = m->data[D_LATERAL];
    refl_range_   = m->data[D_RANGE];
    const double bearing = m->data[D_BEARING];
    const double skew    = m->data[D_SKEW];
    refl_nyaw_    = wrap_pi(skew + bearing + M_PI);   // dock-face heading error (0 = square)
    last_refl_time_ = now();
    if (refl_valid_) {
      last_valid_refl_time_ = last_refl_time_;
      // NAVIGATOR (no command issued here): each valid sample refreshes the IMU heading the
      // pilot should HOLD — current yaw plus the rear-aim error to the centreline carrot.
      // Lateral drift folds itself into this target; a dropout simply freezes it (pure
      // IMU-hold cruise), which is exactly the eyes-closed degradation we want.
      if (have_imu_) yaw_des_ = yaw_ + heading_sign_ * rear_aim_error();
    }
  }
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr m) {
    const auto& q = m->orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_imu_ = true;
  }
  void on_contact(const std_msgs::msg::UInt8::SharedPtr m) { contact_mask_ = m->data; }

  void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (state_ == ACQUIRE || state_ == AIM || state_ == REVERSE_IN) {
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

  // Rear-aim error to the pursuit carrot. The carrot sits ON the dock's outward-normal axis,
  // a fixed lookahead L along-axis toward the dock from the robot's own axis projection —
  // so the robot->carrot separation is ALWAYS (L along-axis, e_y across): no degeneracy, and
  // close-in the carrot passes BEHIND the face = "aim deep at the pogo centre" endgame.
  // Aiming the REAR at the carrot converges offset and heading together (e_y ~ exp(-travel/L)).
  // Returns 0 when the rear points dead at the carrot; + = carrot lies behind-RIGHT (az>0
  // swings the rear right; matches the proven SQUARE sign convention on-axis).
  double rear_aim_error() const {
    const double nx = std::cos(refl_nyaw_), ny = std::sin(refl_nyaw_);   // dock outward normal (robot frame)
    const double s_r = -(refl_along_ * nx + refl_lateral_ * ny);          // robot's along-axis distance off the face
    const double cx = refl_along_   + (s_r - pursuit_lookahead_) * nx;    // carrot = axis point L closer to the dock
    const double cy = refl_lateral_ + (s_r - pursuit_lookahead_) * ny;
    return wrap_pi(std::atan2(cy, cx) - M_PI);                            // bearing off dead-astern
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
          set_state(AIM);
          RCLCPP_INFO(get_logger(),
            "locked: range %.3f lateral %+.3f nyaw %+.1f deg aim %+.1f deg -> aiming rear at the centreline.",
            refl_range_, refl_lateral_, refl_nyaw_ * 180.0 / M_PI, rear_aim_error() * 180.0 / M_PI);
        } else if ((t - acquire_start_).seconds() > acquire_timeout_s_) {
          abort_reason_ = "no stable dock lock"; set_state(ABORT);
        }
        return;
      }

      case AIM: {
        if ((t - last_valid_refl_time_).seconds() > invalid_abort_s_) {
          abort_reason_ = "pose invalid during aim"; set_state(ABORT); publish_stop(); return;
        }
        // Rotate in place to point the REAR at the carrot ON the centreline (NOT merely
        // square to the face: from an offset start, "square" would still miss the axis —
        // this aims the reverse at the centreline from the outset; centring comes first).
        // In-place = full runway, no collision risk (square_only tests the sign the same way).
        const double aim = rear_aim_error();
        const bool aimed = std::fabs(aim) <= square_tol_;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        if (aimed) {
          cmd.angular.z = 0.0;   // hard-stop in the deadband -> let the chassis settle
          if (square_ok_since_.nanoseconds() == 0) square_ok_since_ = t;
        } else {
          const double mag = clampd(kp_heading_ * std::fabs(aim),
                                    square_omega_min_, square_omega_max_);
          cmd.angular.z = heading_sign_ * std::copysign(mag, aim);
          square_ok_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "AIM: aim %+.1f deg | nyaw %+.1f | az %+.2f | range %.3f%s",
          aim * 180.0 / M_PI, refl_nyaw_ * 180.0 / M_PI, cmd.angular.z, refl_range_,
          aimed ? "  [in tol]" : "");

        if (square_ok_since_.nanoseconds() != 0 && (t - square_ok_since_).seconds() >= square_settle_s_) {
          if (square_only_) { RCLCPP_INFO(get_logger(), "square_only: aimed, stopping."); set_state(SEATED); publish_stop(); return; }
          best_range_ = 1e9; best_range_time_ = t;
          yaw_des_ = yaw_;   // cruise starts on the aimed heading; navigator refreshes it
          motion_start_ = t; set_state(REVERSE_IN);
          RCLCPP_INFO(get_logger(), "aimed (aim %+.1f, nyaw %+.1f deg) — cruising down the centreline.",
                      aim * 180.0 / M_PI, refl_nyaw_ * 180.0 / M_PI);
        } else if ((t - acquire_start_).seconds() > square_timeout_s_) {
          abort_reason_ = "aim did not converge (check heading_sign?)"; set_state(ABORT); publish_stop();
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
        // Partial-seat / stall handling near the seat (unchanged from the proven recipe).
        if (refl_range_ < best_range_ - stall_eps_) { best_range_ = refl_range_; best_range_time_ = t; }
        if (refl_range_ < seat_zone_ && (contact_mask_ & 0x03) != 0 &&
            (t - best_range_time_).seconds() > seat_settle_s_) {
          RCLCPP_WARN(get_logger(), "one prox latched (contact=%u, left=%d right=%d), no progress %.1fs "
                      "at range %.3f — final PUSH to seat the other.", contact_mask_,
                      (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0, seat_settle_s_, refl_range_);
          push_start_ = t; set_state(PUSH); return;
        }
        if (refl_range_ < seat_zone_ && (contact_mask_ & 0x03) == 0 &&
            (t - best_range_time_).seconds() > stuck_abort_s_) {
          abort_reason_ = "stuck near seat with no prox contact (arrived skewed)";
          RCLCPP_WARN(get_logger(), "jammed at range %.3f, no prox, no progress %.1fs — abort.",
                      refl_range_, stuck_abort_s_);
          set_state(ABORT); publish_stop(); return;
        }
        // Reflector loss aborts only OUTSIDE the throat: the throat push is deliberately blind
        // (steering off, rails square) and the strips may leave useful lidar geometry at the
        // seat — the stall/jam/floor/timeout guards above still terminate a blind push safely.
        if (!in_throat_ && (t - last_valid_refl_time_).seconds() > invalid_abort_s_) {
          abort_reason_ = "dock pose invalid too long"; set_state(ABORT); publish_stop(); return;
        }
        if ((t - motion_start_).seconds() > overall_timeout_s_) {
          abort_reason_ = "overall reverse timeout"; set_state(ABORT); publish_stop(); return;
        }

        // --- TERMINAL THROAT PUSH ---
        // In the funnel throat: STOP steering, push firmly straight, let the rails square the
        // last mm (steering here would only fight them). Latched so a close-range reflector
        // dropout can't kick us back to steering mid-throat.
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

        // --- CRUISE: straight only; drift -> stop and re-AIM (no in-motion steering, ever) ---
        // Proven live 2026-08-05: a SATURATED in-motion correction produced no rotation — the
        // loaded caster overpowers the tiny wheel differential at crawl speed. So while
        // rolling the command is angular EXACTLY 0 (the plant reverses dead straight), and a
        // real drift is fixed the only way this drivetrain can: stop, rotate in place (the
        // proven ±1 deg primitive), continue. Inside commit_range: committed, no more stops.
        const double err = wrap_pi(yaw_des_ - yaw_);
        if (refl_range_ > commit_range_ && std::fabs(err) > drift_deadband_) {
          RCLCPP_INFO(get_logger(),
            "drifted %+.1f deg at range %.3f — stopping to re-aim (no in-motion steering).",
            err * 180.0 / M_PI, refl_range_);
          publish_stop();
          acquire_start_ = t;   // re-anchor the AIM timeout for this re-aim
          square_ok_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          set_state(AIM);
          return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -v_reverse_;
        cmd.angular.z = 0.0;   // eyes closed: never steer while rolling
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "CRUISE: range %.3f | err %+.1f deg | nyaw %+.1f | lat %+.3f | az 0.00 | contact %u",
          refl_range_, err * 180.0 / M_PI, refl_nyaw_ * 180.0 / M_PI, refl_lateral_, contact_mask_);
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
  double control_hz_, v_reverse_, v_push_, throat_zone_, max_angular_, kp_heading_, heading_sign_,
         pursuit_lookahead_, drift_deadband_, commit_range_, square_tol_,
         square_settle_s_, square_timeout_s_, square_omega_min_, square_omega_max_,
         seat_zone_, seat_settle_s_, stuck_abort_s_, stall_eps_, seat_range_floor_,
         push_omega_, push_v_, push_timeout_s_,
         invalid_abort_s_, acquire_stable_s_, acquire_timeout_s_, overall_timeout_s_;
  bool square_only_;
  int seat_contact_mask_;

  // live inputs / state
  bool   refl_valid_{false}, have_imu_{false}, in_throat_{false};
  double refl_along_{0}, refl_lateral_{0}, refl_range_{0}, refl_nyaw_{0},
         yaw_{0}, yaw_des_{0}, best_range_{1e9};
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
