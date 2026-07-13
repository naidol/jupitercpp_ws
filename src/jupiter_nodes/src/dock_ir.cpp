// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_ir — reverse docking controller. TWO-PHASE, one stop signal.
//
//   FAR  (steer)     <- AprilTag bearing on the dock  (/vision/marker_pose)  [lateral @ distance]
//   NEAR (steer)     <- IR "heartbeat" balance        (/dock/ir_rate)        [lateral @ the mouth]
//   DISTANCE (stop)  <- LiDAR wall range              (/dock/range)
//   SQUARE           <- the guide rails, mechanically. NOT sensed here.
//
// Why two phases (learned the hard way 2026-07-07): the IR balance is FLAT far out (both receivers
// saturated — can't resolve lateral until the mouth) and the base can't creep, so a late, sharp IR
// correction wiggles and arrives too late for a laterally-offset start. The rear webcam sees the
// dock's AprilTag and gives LATERAL AT DISTANCE — so we null the tag bearing over the ~1 m runway
// to arrive centred, then HAND OFF to IR + rails + LiDAR contact-stop when the tag drops out near
// the mouth (z <= handoff_dist) or leaves the frame. If a tag is never seen we run pure-IR (the
// original beam-gated behaviour) unchanged.
//
// The IR balance signal (per-side demodulated-burst RATE from the receivers — the flashing-LED
// "heartbeat"): balance = (left_eps - right_eps)/(left_eps + right_eps).
//   balance = 0  -> centred        |  >0 -> left stronger  |  <0 -> right stronger
//
// The AprilTag pose (/vision/marker_pose) is in the camera optical frame: x = lateral (right +),
// z = distance (forward +). bearing = atan2(x, z); we steer to null it.
//
// reverse:=true backs in caster-first (rails square the low-friction caster; drive wheels stay
//   front for door bumps; camera/face keep facing the room). The camera faces the dock along the
//   reverse-travel direction, so angular.z sign vs tag bearing is not obvious — pin BOTH signs on
//   the first run (flip steer_sign / apriltag_steer_sign to -1.0 if the correction diverges).
//
// Engage:  ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"
// Stop:    ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <vector>

#include <algorithm>
#include <cmath>
#include <memory>

// /dock/range index convention — must match dock_range.cpp.
static constexpr int R_VALID        = 0;
static constexpr int R_DIST_TO_POGO = 1;
static constexpr int R_WALL_ANGLE   = 3;   // rad, 0 = square-on
static constexpr int R_SIZE         = 6;

class DockIr : public rclcpp::Node {
public:
  DockIr() : Node("dock_ir") {
    // Drive / stop — TWO SPEEDS: FAR needs speed so the bearing-null pursuit stays damped
    // (at very low speed the base rotates fast but barely translates -> overshoot/ring on an
    // offset recovery); NEAR must be slow so the guide rails have time to thread the caster in.
    approach_speed_ = declare_parameter("approach_speed", 0.15);  // m/s — FAR (AprilTag align) drive speed
    near_speed_     = declare_parameter("near_speed",     0.08);  // m/s — NEAR (rail-thread + seat) drive speed
    contact_dist_   = declare_parameter("contact_dist",   0.0);   // m — stop when dist_to_pogo <= this
    stall_band_     = declare_parameter("stall_band",     0.05);  // m — stall = "docked" only within this of contact; farther = stuck

    // FAR phase — AprilTag bearing null (lateral @ distance).
    k_bearing_        = declare_parameter("k_bearing",        0.30);  // rad/s per rad of tag bearing (P)
    align_wz_min_     = declare_parameter("align_wz_min",     0.35);  // rad/s — minimum ALIGN rotation command (below this the base stalls rotationally, PIDs wind up, lurch)
    k_bearing_d_      = declare_parameter("k_bearing_d",      0.15);  // rad/s per rad/s of bearing rate (D — damps the ring)
    bearing_deadband_ = declare_parameter("bearing_deadband", 0.05);  // rad — don't chase noise inside this
    handoff_dist_     = declare_parameter("handoff_dist",     0.35);  // m — tag z at/below this -> hand off to IR near phase
    near_commit_dist_ = declare_parameter("near_commit_dist", 0.45);  // m — LiDAR to-pogo must be <= this to allow a blind near-phase after TAG LOSS (farther = abort)
    handoff_x_max_    = declare_parameter("handoff_x_max",    0.06);  // m — |tag x| beyond this at handoff -> go-around (would straddle a rail, not thread the channel)
    handoff_angle_max_ = declare_parameter("handoff_angle_max", 0.14); // rad (~8°) — LiDAR wall angle beyond this at handoff -> go-around (arriving too skewed for the rails)
    dock_x_ref_       = declare_parameter("dock_x_ref",       0.010); // m — tag x when PROPERLY seated (measured signature)
    dock_x_tol_       = declare_parameter("dock_x_tol",       0.045); // m — seated tag x within ref±tol = VERIFIED; outside = MISSED. Good seats cluster ±0.03; the real straddle read +0.065 err (0.030 false-missed a good dock — the famous self-undock)

    // Go-around (the "missed approach" routine): decision conditions not met at handoff, or a
    // verified miss -> drive FORWARD away from the dock, steering on the tag to re-centre (pulling
    // forward straightens, like un-jackknifing a trailer), back to retry_dist -> re-attempt.
    retry_dist_        = declare_parameter("retry_dist",        0.95);  // m — tag z to retreat to before re-attempting
    forward_clear_min_ = declare_parameter("forward_clear_min", 0.40);  // m — anything closer than this AHEAD halts a go-around (couch guard)
    max_attempts_      = static_cast<int>(declare_parameter("max_attempts", 3));  // dock attempts before giving up
    go_steer_sign_     = declare_parameter("go_steer_sign",     1.0);   // steering sign while driving FORWARD (opposite geometry to reversing; pin empirically)
    go_around_timeout_ = declare_parameter("go_around_timeout", 25.0);  // s — a go-around taking longer than this = stop + give up
    tag_timeout_      = declare_parameter("tag_timeout",      0.5);   // s — tag stale this long -> lost (triggers handoff)
    tag_alpha_        = declare_parameter("tag_alpha",        0.5);   // EMA on tag pose
    apriltag_steer_sign_ = declare_parameter("apriltag_steer_sign", 1.0);  // flip to -1.0 if FAR correction diverges

    // NEAR phase — IR balance (lateral @ the mouth).
    Ky_balance_     = declare_parameter("Ky_balance",     0.40);  // rad/s per unit balance (balance in [-1,1])
    max_wz_         = declare_parameter("max_wz",         0.50);  // rad/s clamp (both phases)
    balance_alpha_  = declare_parameter("balance_alpha",  0.25);  // EMA smoothing on the noisy per-cycle rate
    sum_min_        = declare_parameter("sum_min",        20.0);  // eps — below this (L+R) there's no usable beam

    // Signs / direction — pin empirically on first run.
    reverse_        = declare_parameter("reverse",        false);
    steer_sign_     = declare_parameter("steer_sign",     1.0);   // flip to -1.0 if NEAR (IR) correction diverges

    // Liveness / robustness
    ir_timeout_       = declare_parameter("ir_timeout",       1.0);   // s — IR-msg gap before pausing
    ir_hard_timeout_  = declare_parameter("ir_hard_timeout",  8.0);   // s — IR silent this long -> give up (rides ESP32 reconnects)
    range_hold_       = declare_parameter("range_hold",       0.5);   // s — drive on last VALID range across brief invalid flickers
    stuck_vel_thresh_ = declare_parameter("stuck_vel_threshold", 0.01); // m/s — below = physically stopped
    stuck_timeout_    = declare_parameter("stuck_timeout",    3.0);   // s — stalled this long WHILE DRIVING (near contact) = docked
    control_rate_     = declare_parameter("control_rate",     20.0);  // Hz

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // AprilTag pose on the dock (camera optical frame): x = lateral, z = distance.
    tag_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vision/marker_pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const double x = msg->pose.position.x;
        const double z = msg->pose.position.z;
        const rclcpp::Time t_now = this->now();
        if (!have_tag_) { tag_x_ = x; tag_z_ = z; tag_z_rate_ = 0.0; }
        else {
          // Vision-derived translation speed (tag z-rate). This is the "am I rolling" truth:
          // both odom twists are unusable (encoder twist ~0 while driving; EKF flickers).
          const double dt = (t_now - last_tag_time_).seconds();
          if (dt > 1e-3 && dt < 0.5) {
            const double raw = (z - tag_z_) / dt;
            tag_z_rate_ = 0.3 * raw + 0.7 * tag_z_rate_;
          }
          tag_x_ = tag_alpha_ * x + (1.0 - tag_alpha_) * tag_x_;
          tag_z_ = tag_alpha_ * z + (1.0 - tag_alpha_) * tag_z_;
        }
        last_tag_time_ = t_now;
        have_tag_      = true;
      });

    // IR heartbeat rate: [left_eps, right_eps] -> balance + sum (EMA-smoothed).
    ir_rate_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/dock/ir_rate", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        const double l = msg->data[0], r = msg->data[1];
        const double sum = l + r;
        const double bal = (sum > sum_min_) ? (l - r) / sum : 0.0;
        sum_ema_     = balance_alpha_ * sum + (1.0 - balance_alpha_) * sum_ema_;
        balance_ema_ = balance_alpha_ * bal + (1.0 - balance_alpha_) * balance_ema_;
        last_ir_time_ = this->now();
        have_ir_      = true;
        if (sum_ema_ > sum_min_) beam_ever_ = true;
      });

    range_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/dock/range", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (static_cast<int>(msg->data.size()) < R_SIZE) return;
        if (msg->data[R_VALID] > 0.5f) {
          dist_to_pogo_        = msg->data[R_DIST_TO_POGO];
          wall_angle_          = msg->data[R_WALL_ANGLE];
          last_valid_range_time_ = this->now();
          have_range_          = true;
        }
      });

    // Odom feeds ONLY the stall detector. Neither odom is a usable rolling signal (the ESP32's
    // encoder twist reads ~0.001 m/s while visibly driving; the EKF twist flickers) — the
    // steer-while-rolling gate instead uses the TAG's z-rate, which IS the translation truth.
    const std::string odom_topic =
      declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_linear_x_ = msg->twist.twist.linear.x;
        // Yaw (IMU-driven via the EKF) — the heading reference for the go-around's yaw-hold.
        const auto& q = msg->pose.pose.orientation;
        odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        have_yaw_ = true;
      });

    // Forward clearance from the raw scan — the couch guard for go-arounds. Laser frame: the
    // DOCK/rear is bearing 0 (base_laser mounted yaw=π), so the robot's FRONT is bearing π.
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
        constexpr double kHalf = 25.0 * M_PI / 180.0;
        double best = 1e9, a = scan->angle_min;
        for (const float r : scan->ranges) {
          const double off = std::atan2(std::sin(a - M_PI), std::cos(a - M_PI));
          if (std::fabs(off) <= kHalf && std::isfinite(r) && r > 0.05 && r < best) best = r;
          a += scan->angle_increment;
        }
        forward_clear_      = best;
        last_scan_time_     = this->now();
      });

    auto on_engage = [this](bool engage) {
      engaged_ = engage;
      if (engaged_) {
        reset_state();
        RCLCPP_INFO(get_logger(), "Docking engaged (%s).", reverse_ ? "reverse" : "forward");
      } else {
        publish_zero();
        RCLCPP_INFO(get_logger(), "Docking stopped.");
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

    // Live tuning — sweep speed/gains via `ros2 param set /dock_ir <name> <val>` without a relaunch.
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&DockIr::on_set_params, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "dock_ir (AprilTag far -> IR near) online. Engage: ros2 service call /dock_engage std_srvs/srv/SetBool \"{data: true}\"");
  }

private:
  // Live-update the tunable knobs (read every control cycle, so changes take effect immediately).
  rcl_interfaces::msg::SetParametersResult
  on_set_params(const std::vector<rclcpp::Parameter>& params) {
    for (const auto& p : params) {
      const std::string& n = p.get_name();
      if      (n == "approach_speed")      approach_speed_      = p.as_double();
      else if (n == "near_speed")          near_speed_          = p.as_double();
      else if (n == "contact_dist")        contact_dist_        = p.as_double();
      else if (n == "k_bearing")           k_bearing_           = p.as_double();
      else if (n == "k_bearing_d")         k_bearing_d_         = p.as_double();
      else if (n == "align_wz_min")        align_wz_min_        = p.as_double();
      else if (n == "bearing_deadband")    bearing_deadband_    = p.as_double();
      else if (n == "handoff_dist")        handoff_dist_        = p.as_double();
      else if (n == "near_commit_dist")    near_commit_dist_    = p.as_double();
      else if (n == "handoff_x_max")       handoff_x_max_       = p.as_double();
      else if (n == "handoff_angle_max")   handoff_angle_max_   = p.as_double();
      else if (n == "dock_x_ref")          dock_x_ref_          = p.as_double();
      else if (n == "dock_x_tol")          dock_x_tol_          = p.as_double();
      else if (n == "retry_dist")          retry_dist_          = p.as_double();
      else if (n == "go_steer_sign")       go_steer_sign_       = p.as_double();
      else if (n == "max_attempts")        max_attempts_        = static_cast<int>(p.as_int());
      else if (n == "apriltag_steer_sign") apriltag_steer_sign_ = p.as_double();
      else if (n == "Ky_balance")          Ky_balance_          = p.as_double();
      else if (n == "max_wz")              max_wz_              = p.as_double();
      else if (n == "steer_sign")          steer_sign_          = p.as_double();
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  void publish_zero() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  // Missed-approach: abandon this attempt and retreat to the retry pose. Bounded by max_attempts.
  void start_go_around(const char* reason) {
    publish_zero();
    if (attempts_ >= max_attempts_) {
      RCLCPP_ERROR(get_logger(),
        "GO-AROUND (%s) — but %d/%d attempts used: GIVING UP. Re-stage and re-engage.",
        reason, attempts_, max_attempts_);
      engaged_ = false;
      return;
    }
    attempts_++;
    go_around_        = true;
    go_around_start_  = this->now();
    committed_near_   = false;
    have_prev_bearing_ = false;
    bearing_rate_ema_  = 0.0;
    // Capture the heading NOW (IMU via EKF): the retreat HOLDS this yaw — camera stays on the
    // dock, no pursuit, no sign ambiguity (a yaw regulator is self-correcting by construction).
    go_yaw_ref_  = odom_yaw_;
    go_have_ref_ = have_yaw_;
    RCLCPP_WARN(get_logger(), "GO-AROUND %d/%d (%s) — retreating to %.2f m holding yaw %.1f deg.",
                attempts_, max_attempts_, reason, retry_dist_, go_yaw_ref_ * 57.2958);
  }

  // Drive FORWARD away from the dock, steering on the tag to arrive back centred at retry_dist,
  // then re-enter the FAR approach. The camera keeps facing the dock throughout.
  void go_around_step() {
    if ((this->now() - go_around_start_).seconds() > go_around_timeout_) {
      RCLCPP_ERROR(get_logger(), "Go-around timed out — stopping. Re-stage and re-engage.");
      publish_zero(); engaged_ = false; go_around_ = false;
      return;
    }

    // Couch guard: we are driving FORWARD, possibly blind — halt if anything is close ahead.
    const bool scan_fresh = (this->now() - last_scan_time_).seconds() < 0.5;
    if (scan_fresh && forward_clear_ < forward_clear_min_) {
      RCLCPP_ERROR(get_logger(),
        "Go-around BLOCKED — obstacle %.2f m ahead (limit %.2f). Stopping.",
        forward_clear_, forward_clear_min_);
      publish_zero(); engaged_ = false; go_around_ = false;
      return;
    }

    const bool tag_fresh =
      have_tag_ && (this->now() - last_tag_time_).seconds() < tag_timeout_;

    // Exit ONLY on the tag: retreated far enough with the tag in view, after a real retreat
    // (min 2s — prevents the instant-exit thrash that burned all attempts in 200ms). Lateral is
    // NOT required here — the retreat holds heading, so the offset persists by design; the next
    // FAR approach corrects it with full runway. No tag by go_around_timeout -> stop, never blind.
    const bool retreated_enough = (this->now() - go_around_start_).seconds() > 2.0;
    if (retreated_enough && tag_fresh && tag_z_ >= retry_dist_) {
      publish_zero();
      go_around_     = false;
      attempt_start_ = this->now();
      RCLCPP_INFO(get_logger(), "Go-around complete (z %.2f, x %+.3f) — RE-ATTEMPTING dock (attempt %d/%d).",
                  tag_z_, tag_x_, attempts_ + 1, max_attempts_ + 1);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = (reverse_ ? 1.0 : -1.0) * approach_speed_;  // AWAY from the dock
    // YAW-HOLD retreat: regulate heading to the captured reference (IMU/EKF). No tag pursuit —
    // holding the approach heading keeps the camera ON the dock so the tag never leaves view.
    if (go_have_ref_ && have_yaw_) {
      const double yaw_err = std::atan2(std::sin(go_yaw_ref_ - odom_yaw_),
                                        std::cos(go_yaw_ref_ - odom_yaw_));
      cmd.angular.z = std::clamp(0.8 * yaw_err, -max_wz_, max_wz_);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "GO_AROUND: z %.3f, x %+.3f, yaw-hold wz %+.2f", tag_z_, tag_x_, cmd.angular.z);
    cmd_pub_->publish(cmd);
  }

  // Post-dock verification off the tag: a straddle/miss reaches wall-contact distance like a
  // real dock (LiDAR can't tell), but the seated tag x gives the truth. Compare to the known
  // properly-seated signature (dock_x_ref) — outside tolerance means we're ON a rail, not in it.
  void verify_docked() {
    const bool tag_fresh =
      have_tag_ && (this->now() - last_tag_time_).seconds() < 1.0;
    if (!tag_fresh) {
      RCLCPP_WARN(get_logger(), "Docked but tag not visible — cannot verify seat position.");
      return;
    }
    const double err = tag_x_ - dock_x_ref_;
    if (std::fabs(err) <= dock_x_tol_) {
      RCLCPP_INFO(get_logger(), "DOCKED VERIFIED — seated on centre (tag x %+.3f, err %+.3f m).",
                  tag_x_, err);
    } else {
      RCLCPP_ERROR(get_logger(),
        "DOCK MISSED — seated %+.3f m off centre (tag x %+.3f): likely straddling a rail.",
        err, tag_x_);
      engaged_ = true;                 // stay engaged: ride back off the rail and retry
      start_go_around("verified dock miss");
    }
  }

  void reset_state() {
    stuck_since_      = this->now();
    engage_time_      = this->now();
    moving_confirmed_ = false;
    beam_ever_        = false;
    committed_near_   = false;
    tag_ever_seen_    = false;
    have_prev_bearing_ = false;
    bearing_rate_ema_  = 0.0;
    go_around_        = false;
    attempts_         = 0;
    attempt_start_    = this->now();
    tag_z_rate_       = 0.0;
    aligning_         = false;
  }

  // FAR phase: reverse at constant speed, steer to null the AprilTag bearing. Runs until the tag
  // gets close (z <= handoff_dist) or drops out — then we commit to the NEAR phase for good.
  // Returns true while it is driving (caller should return), false once it has committed to NEAR.
  bool apriltag_far_step() {
    if (committed_near_) return false;

    const bool tag_fresh =
      have_tag_ && (this->now() - last_tag_time_).seconds() < tag_timeout_;

    if (tag_fresh && tag_z_ > handoff_dist_) {
      tag_ever_seen_ = true;
      const double bearing = std::atan2(tag_x_, tag_z_);  // camera frame: x lateral, z forward

      // PD: the D term steers against the bearing's rate of change, easing the correction off
      // as the tag swings back toward centre — kills the left-right "wiggle" (underdamped ring)
      // that pure P shows on offset recoveries. Rate is EMA-filtered; first cycle has no rate.
      const rclcpp::Time t_now = this->now();
      double bearing_rate = 0.0;
      if (have_prev_bearing_) {
        const double dt = (t_now - prev_bearing_time_).seconds();
        if (dt > 1e-3) {
          const double raw_rate = (bearing - prev_bearing_) / dt;
          bearing_rate_ema_ = 0.3 * raw_rate + 0.7 * bearing_rate_ema_;
          bearing_rate = bearing_rate_ema_;
        } else {
          bearing_rate = bearing_rate_ema_;
        }
      }
      prev_bearing_      = bearing;
      prev_bearing_time_ = t_now;
      have_prev_bearing_ = true;

      // ALIGN-THEN-DRIVE (user insight: "speed is the killer — it's not letting the robot turn").
      // At drive speed the base cannot rotate a big bearing out; the view goes oblique and the
      // tag is lost. So: bearing large -> STOP and ROTATE IN PLACE (closed-loop on the tag, slow,
      // self-limiting — bearing shrinks, wz shrinks) until small again, THEN drive.
      if (!aligning_ && std::fabs(bearing) > 0.20) aligning_ = true;   // ~11 deg: stop & turn
      if (aligning_  && std::fabs(bearing) < 0.10) aligning_ = false;  // ~6 deg: resume driving

      geometry_msgs::msg::Twist cmd;
      double wz = 0.0;
      if (aligning_) {
        cmd.linear.x = 0.0;
        // Minimum rotation authority: a small PD output stalls against rotational stiction, the
        // wheel PIDs wind up, and the release is a violent lurch that sweeps the tag out of frame
        // (proven twice). If we turn at all, turn at >= align_wz_min — hysteresis exit (0.10 rad)
        // still bounds the overshoot.
        const double pd = apriltag_steer_sign_ * (k_bearing_ * bearing + k_bearing_d_ * bearing_rate);
        wz = std::clamp((pd >= 0 ? 1.0 : -1.0) * std::max(std::fabs(pd), align_wz_min_),
                        -0.45, 0.45);
      } else {
        // Driving: steer only while actually translating (vision-confirmed via tag z-rate) — a
        // stationary robot given wz while "driving" is stiction-pivot, not steering.
        const bool rolling = std::fabs(tag_z_rate_) > 0.015;
        cmd.linear.x = (reverse_ ? -1.0 : 1.0) * approach_speed_;
        if (rolling && (std::fabs(bearing) > bearing_deadband_ || std::fabs(bearing_rate) > 0.05))
          wz = std::clamp(apriltag_steer_sign_ * (k_bearing_ * bearing + k_bearing_d_ * bearing_rate),
                          -max_wz_, max_wz_);
      }
      cmd.angular.z = wz;
      cmd_pub_->publish(cmd);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "APRILTAG_FAR%s: z %.3f m, x %+.3f, bearing %+.3f rad (rate %+.2f), wz %+.2f",
        aligning_ ? " [ALIGN]" : "", tag_z_, tag_x_, bearing, bearing_rate, wz);

      // keep the near-phase stall timer fresh so it doesn't false-trip right after handoff
      stuck_since_ = this->now();
      return true;
    }

    // Commit to the NEAR (blind straight-in) phase ONLY if we are genuinely at the mouth —
    // tag close, or LiDAR confirms we're near the dock. Losing the tag FAR out (pivoted away,
    // occlusion, lighting) must ABORT, not drive blind from a metre away on a bad heading.
    const bool lidar_near =
      have_range_ && (this->now() - last_valid_range_time_).seconds() < range_hold_ &&
      dist_to_pogo_ <= near_commit_dist_;
    if (!tag_fresh && tag_ever_seen_ && !lidar_near) {
      // PATIENCE before declaring the tag lost: a brief dropout is usually MOTION BLUR (slow
      // webcam shutter while rotating). STOP — blur clears when stationary — and give the
      // detector time to re-lock before concluding the tag is truly gone. This also covers the
      // (re-)acquisition window right after engage or a go-around exit.
      const double gap = have_tag_ ? (this->now() - last_tag_time_).seconds()
                                   : (this->now() - attempt_start_).seconds();
      if (gap < 3.0) {
        publish_zero();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Tag dropout (%.1fs) — holding still to re-acquire...", gap);
        return true;
      }
      start_go_around("tag lost far from dock");
      return true;  // handled — this control cycle is done
    }

    // ===== DECISION POINT (the "decision height") — commit to the blind rails-final ONLY if: =====
    // (1) LATERAL: the caster (robot centreline) will enter BETWEEN the rails. A big lateral
    //     error here = straddle a rail = a false "Docked" the LiDAR cannot detect.
    //     Proven: a miss telegraphed itself with x=+0.111 at handoff (rode the left rail).
    // LAST-KNOWN x, fresh or not: a tag lost near the dock while off-centre must go around —
    // committing blind on a stale off-centre reading drove 28cm-off into the rails once.
    if (tag_ever_seen_ && std::fabs(tag_x_ - dock_x_ref_) > handoff_x_max_) {
      RCLCPP_WARN(get_logger(), "Decision point: OFF-CENTRE (%s tag x %+.3f, limit ±%.3f).",
                  tag_fresh ? "" : "last-known", tag_x_, handoff_x_max_);
      start_go_around("off-centre at decision point");
      return true;
    }
    // (2) HEADING: arriving square enough that the rail funnel can finish the job. The LiDAR
    //     wall angle is the heading truth (tag bearing conflates heading with lateral).
    if (lidar_near && std::fabs(wall_angle_) > handoff_angle_max_) {
      RCLCPP_WARN(get_logger(), "Decision point: TOO SKEWED (wall angle %+.1f deg, limit %.1f).",
                  wall_angle_ * 57.2958, handoff_angle_max_ * 57.2958);
      start_go_around("too skewed at decision point");
      return true;
    }

    committed_near_ = true;
    engage_time_    = this->now();     // fresh IR-liveness window for the near phase
    stuck_since_    = this->now();
    moving_confirmed_ = false;
    RCLCPP_INFO(get_logger(), "HANDOFF -> IR near phase (tag %s, last z %.3f m, x %+.3f).",
                (tag_fresh ? "close" : "lost/absent"), tag_z_, tag_x_);
    return false;
  }

  void control_step() {
    if (!engaged_) { publish_zero(); return; }

    // ===== Phase 0: GO-AROUND — missed approach; retreat, re-centre, re-attempt. =====
    if (go_around_) { go_around_step(); return; }

    // ===== Phase 1: FAR — AprilTag bearing null over the runway. =====
    if (apriltag_far_step()) return;

    // ===== Phase 2: NEAR — IR balance + LiDAR contact-stop + rails. =====
    // IR liveness. In pure-IR mode (no tag ever seen) IR is the ONLY lateral+liveness guide, so we
    // enforce it hard. After an AprilTag far-align, IR is a bonus fine-tune — never let its silence
    // abort a dock the tag already lined up (LiDAR still stops us at contact).
    const double ir_gap = have_ir_ ? (this->now() - last_ir_time_).seconds()
                                   : (this->now() - engage_time_).seconds();
    const bool ir_fresh = have_ir_ && ir_gap <= ir_timeout_;

    if (!tag_ever_seen_) {
      if (ir_gap > ir_hard_timeout_) {
        RCLCPP_WARN(get_logger(), "No IR signal for %.1fs — disengaging.", ir_gap);
        publish_zero(); engaged_ = false; return;
      }
      if (!ir_fresh) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for IR signal...");
        publish_zero(); stuck_since_ = this->now(); return;
      }
      // Beam not acquired yet — hold (place robot roughly facing the dock).
      if (!beam_ever_) {
        publish_zero();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for beam...");
        return;
      }
    }

    // --- LiDAR distance must be usable (fresh-enough valid reading) to drive: never blind.
    //     range_hold bridges brief INVALID flickers (drive on the last valid reading). ---
    const bool range_usable =
      have_range_ && (this->now() - last_valid_range_time_).seconds() < range_hold_;
    if (!range_usable) {
      publish_zero();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for valid LiDAR range...");
      stuck_since_ = this->now();
      return;
    }

    // Contact reached (LiDAR) — the primary stop.
    if (dist_to_pogo_ <= contact_dist_) {
      RCLCPP_INFO(get_logger(), "Reached contact distance (%.3f m) — stop.", dist_to_pogo_);
      publish_zero(); engaged_ = false;
      verify_docked();
      return;
    }

    geometry_msgs::msg::Twist cmd;
    // Drive: NEAR speed — slow, so the guide rails have time to thread the caster in.
    cmd.linear.x = (reverse_ ? -1.0 : 1.0) * near_speed_;
    // Steer: null the IR balance when it's live & resolving; else drive straight (the AprilTag far
    // phase already centred us and the rails square the last bit). Self-gating near the mouth.
    const bool use_balance = ir_fresh && beam_ever_;
    cmd.angular.z = use_balance
      ? std::clamp(steer_sign_ * Ky_balance_ * balance_ema_, -max_wz_, max_wz_)
      : 0.0;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "IR_NEAR: to-pogo %.3f m, v %.3f, balance %+.2f (sum %.0f)%s, wz %+.2f",
      dist_to_pogo_, cmd.linear.x, balance_ema_, sum_ema_,
      use_balance ? "" : " [straight]", cmd.angular.z);

    // Docked detection — a stall counts as contact ONLY near the dock; a stall far out is
    // a STUCK fault (drive can't sustain, or obstruction), never a false "docked".
    if (std::fabs(odom_linear_x_) > stuck_vel_thresh_) {
      moving_confirmed_ = true;
      stuck_since_      = this->now();
    }
    if (moving_confirmed_ && (this->now() - stuck_since_).seconds() > stuck_timeout_) {
      if (dist_to_pogo_ <= contact_dist_ + stall_band_) {
        RCLCPP_INFO(get_logger(), "Docked — stalled at contact (%.3f m to pogo).", dist_to_pogo_);
        publish_zero(); engaged_ = false;
        verify_docked();
        return;
      } else {
        RCLCPP_WARN(get_logger(),
          "STALLED %.3f m from dock — NOT docked. Raise approach_speed or clear obstruction.",
          dist_to_pogo_);
      }
      publish_zero(); engaged_ = false; return;
    }

    cmd_pub_->publish(cmd);
  }

  // params
  double approach_speed_, near_speed_, contact_dist_, stall_band_;
  double k_bearing_, k_bearing_d_, align_wz_min_, bearing_deadband_, handoff_dist_, near_commit_dist_, tag_timeout_, tag_alpha_, apriltag_steer_sign_;
  double handoff_x_max_, handoff_angle_max_, dock_x_ref_, dock_x_tol_;
  double retry_dist_, go_steer_sign_, go_around_timeout_, forward_clear_min_;
  int    max_attempts_{3};
  double Ky_balance_, max_wz_, balance_alpha_, sum_min_;
  bool   reverse_{false};
  double steer_sign_{1.0};
  double ir_timeout_, ir_hard_timeout_, range_hold_;
  double stuck_vel_thresh_, stuck_timeout_, control_rate_;

  // state
  double       tag_x_{0.0}, tag_z_{0.0}, tag_z_rate_{0.0};
  double       wall_angle_{0.0};
  double       prev_bearing_{0.0}, bearing_rate_ema_{0.0};
  bool         have_prev_bearing_{false};
  bool         aligning_{false};
  bool         go_around_{false};
  int          attempts_{0};
  double       forward_clear_{1e9};
  double       odom_yaw_{0.0}, go_yaw_ref_{0.0};
  bool         have_yaw_{false}, go_have_ref_{false};
  rclcpp::Time prev_bearing_time_, go_around_start_, attempt_start_, last_scan_time_;
  double       balance_ema_{0.0}, sum_ema_{0.0};
  double       dist_to_pogo_{0.0}, odom_linear_x_{0.0};
  bool         have_tag_{false}, have_ir_{false}, beam_ever_{false}, have_range_{false};
  bool         engaged_{false}, moving_confirmed_{false};
  bool         committed_near_{false}, tag_ever_seen_{false};
  rclcpp::Time last_tag_time_, last_ir_time_, last_valid_range_time_, stuck_since_, engage_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    tag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   ir_rate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   range_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr        scan_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                  engage_srv_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                engage_sub_;
  rclcpp::TimerBase::SharedPtr                                        timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr   param_cb_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockIr>());
  rclcpp::shutdown();
  return 0;
}
