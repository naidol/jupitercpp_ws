// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_aligner_v3 — SEGMENTED, POSITION-CONTROLLED reverse docking.
//
// WHY V3 EXISTS
// -------------
// V1 and V2 both steer by VELOCITY (publish Twist to /cmd_vel). Measured 2026-08-08: the ESP32
// velocity loop needs 4-9 s to reach a commanded wheel speed while a docking approach lasts ~7 s,
// so every steering correction expired before the firmware executed it. That is why the robot
// appeared to refuse to steer -- it was always still ramping.
//
// V3 commands DISTANCES instead. Steering is a differential distance:
//
//      dS = theta * WHEEL_SEPARATION
//
// which is geometry and does not care how slowly it happens. A slow, ugly velocity profile still
// lands the wheels on their target counts, so the turn comes out right. The velocity loop's
// settling time leaves the critical path entirely.
//
// STRUCTURE — nested loops, and this is the whole idea:
//   INNER (open):   one segment executed by the ESP32 as encoder counts. Precise, and immune to
//                   the velocity lag. Firmware owns stall/timeout/divergence guards.
//   OUTER (closed): after EVERY segment, re-read the reflector and decide the next one.
// Wheel slip and the ~2 deg caster scrub measured during in-place rotation are therefore absorbed
// between segments instead of accumulating over a whole approach. Each stop also settles the
// casters -- the long-wanted "pre-flip", for free.
//
// PRIORITY: CENTRE FIRST, square second. The throat/rails exist to absorb a residual few degrees
// of ANGLE; what they can never absorb is arriving OFF-CENTRE. Every pre-V3 scheme had this
// backwards -- they controlled squareness to the dock face and never targeted the dock centre.
//
// FLOW: ACQUIRE -> [ AIM in place | DRIVE straight ]* -> COMMIT (final straight) -> rails/prox
//
// V2 (velocity pure-pursuit) is deliberately kept intact alongside this; V3 is a separate node,
// not a mutation of it. Nav2 is unaffected: it drives /cmd_vel, V3 drives /wheel_move.
//
// START:  ros2 service call /dock/v3/align_start  std_srvs/srv/Trigger
// CANCEL: ros2 service call /dock/v3/align_cancel std_srvs/srv/Trigger
//
// *** STATUS: NEW, NEVER RUN. Every behaviour below is unverified. ***

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
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

// /wheel_move_state values published by the firmware.
constexpr uint8_t MV_IDLE = 0, MV_RUNNING = 1, MV_DONE = 2,
                  MV_STALL = 3, MV_TIMEOUT = 4, MV_REJECT = 5, MV_DIVERGE = 6;

double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

const char* move_state_name(uint8_t s) {
  switch (s) {
    case MV_IDLE: return "IDLE"; case MV_RUNNING: return "RUNNING"; case MV_DONE: return "DONE";
    case MV_STALL: return "STALL"; case MV_TIMEOUT: return "TIMEOUT"; case MV_REJECT: return "REJECT";
    case MV_DIVERGE: return "DIVERGE"; default: return "?";
  }
}
}  // namespace

class DockAlignerV3 : public rclcpp::Node {
public:
  DockAlignerV3() : Node("dock_aligner_v3") {
    reflector_topic_  = declare_parameter<std::string>("reflector_topic",  "/dock/reflector");
    confidence_topic_ = declare_parameter<std::string>("confidence_topic", "/dock/reflector_confidence");
    contact_topic_    = declare_parameter<std::string>("contact_topic",    "/dock/contact");
    move_cmd_topic_   = declare_parameter<std::string>("move_cmd_topic",   "/wheel_move");
    move_state_topic_ = declare_parameter<std::string>("move_state_topic", "/wheel_move_state");
    state_topic_      = declare_parameter<std::string>("state_topic",      "/dock/v3/aligner_state");

    // --- GEOMETRY: counts <-> metres/radians. Defaults derive from the CALIBRATED firmware
    // values (COUNTS_PER_REV 1290, WHEEL_RADIUS 0.050, WHEEL_SEPARATION 0.3586 measured
    // 2026-08-10). Keep these in step with jupiter_config.h or every segment is mis-scaled.
    counts_per_m_    = declare_parameter("counts_per_m",     4106.3);   // 1290 / (2*pi*0.050)
    wheel_separation_= declare_parameter("wheel_separation", 0.3586);   // CALIBRATED, not the old 0.355
    // counts per radian of robot rotation, PER WHEEL (wheels move opposite): (W/2) * counts_per_m
    counts_per_rad_  = declare_parameter("counts_per_rad", (0.3586 / 2.0) * 4106.3);  // ~736.3

    // --- segment shaping
    seg_len_m_        = declare_parameter("seg_len_m",        0.25);   // straight segment cap
    seg_rpm_          = declare_parameter("seg_rpm",          12);     // precision speed: overshoot scales with this
    aim_tol_          = declare_parameter("aim_tol_deg",      2.0) * M_PI / 180.0;
    // Flips the rotation convention wholesale. Derived value is +1 (see the AIM block); if the
    // first AIM segment makes the offset GROW instead of shrink, set this to -1.0 and re-test.
    aim_sign_         = declare_parameter("aim_sign",         1.0);
    commit_range_     = declare_parameter("commit_range",     0.40);   // below this: no more re-aim, drive in
    max_segments_     = declare_parameter("max_segments",     25);     // don't loop forever

    // --- target pose. MEASURED on a real seat (from dock_aligner_v2): the reflector reads this
    // when the robot is properly docked. The final segment drives (d - seated_range).
    seated_range_m_   = declare_parameter("seated_range_m",   0.1937);
    seat_settle_s_    = declare_parameter("seat_settle_s",    2.0);

    // --- gating / safety
    min_confidence_   = declare_parameter("min_confidence",   0.70);
    reflector_stale_s_= declare_parameter("reflector_stale_s",0.30);
    acquire_stable_s_ = declare_parameter("acquire_stable_s", 0.5);
    acquire_timeout_s_= declare_parameter("acquire_timeout_s",12.0);
    overall_timeout_s_= declare_parameter("overall_timeout_s",180.0);  // segmented = slower than a continuous run
    seg_ack_timeout_s_= declare_parameter("seg_ack_timeout_s",1.5);    // firmware must report RUNNING this fast
    max_stall_retries_= declare_parameter("max_stall_retries",2);
    // Safety envelope: refuse to start, or abort, if the dock is implausibly far off-axis.
    max_lateral_m_    = declare_parameter("max_lateral_m",    0.35);
    max_start_range_m_= declare_parameter("max_start_range_m",1.60);

    control_hz_       = declare_parameter("control_hz",       10.0);
    dry_run_          = declare_parameter("dry_run",          false);  // plan + log, publish NOTHING

    move_pub_  = create_publisher<std_msgs::msg::Int32MultiArray>(move_cmd_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(1).transient_local());

    refl_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      reflector_topic_, 10, std::bind(&DockAlignerV3::on_reflector, this, std::placeholders::_1));
    conf_sub_ = create_subscription<std_msgs::msg::Float32>(
      confidence_topic_, 10, std::bind(&DockAlignerV3::on_confidence, this, std::placeholders::_1));
    contact_sub_ = create_subscription<std_msgs::msg::UInt8>(
      contact_topic_, 10, std::bind(&DockAlignerV3::on_contact, this, std::placeholders::_1));
    move_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      move_state_topic_, 10, std::bind(&DockAlignerV3::on_move_state, this, std::placeholders::_1));

    start_srv_ = create_service<std_srvs::srv::Trigger>("/dock/v3/align_start",
      std::bind(&DockAlignerV3::on_start, this, std::placeholders::_1, std::placeholders::_2));
    cancel_srv_ = create_service<std_srvs::srv::Trigger>("/dock/v3/align_cancel",
      std::bind(&DockAlignerV3::on_cancel, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_hz_)),
      std::bind(&DockAlignerV3::on_tick, this));

    set_state(IDLE);
    RCLCPP_INFO(get_logger(),
      "dock_aligner_v3 (SEGMENTED/POSITION, UNTESTED) ready. seg=%.2fm @%drpm aim_tol=%.1fdeg "
      "commit=%.2fm seated_range=%.4fm counts/m=%.1f counts/rad=%.1f%s. Call /dock/v3/align_start.",
      seg_len_m_, static_cast<int>(seg_rpm_), aim_tol_ * 180.0 / M_PI, commit_range_,
      seated_range_m_, counts_per_m_, counts_per_rad_, dry_run_ ? "  [DRY RUN]" : "");
  }

private:
  enum State { IDLE, ACQUIRE, PLAN, WAIT_SEG, SEAT_WAIT, SEATED, ABORT };
  const char* state_name(State s) const {
    switch (s) { case IDLE: return "IDLE"; case ACQUIRE: return "ACQUIRE"; case PLAN: return "PLAN";
                 case WAIT_SEG: return "WAIT_SEG"; case SEAT_WAIT: return "SEAT_WAIT";
                 case SEATED: return "SEATED"; default: return "ABORT"; }
  }
  void set_state(State s) {
    state_ = s;
    std_msgs::msg::String m; m.data = state_name(s);
    state_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "state -> %s", state_name(s));
  }

  // ---- inputs ----
  void on_reflector(const std_msgs::msg::Float32MultiArray::SharedPtr m) {
    if (m->data.size() < 6) return;
    refl_valid_   = (m->data[D_VALID] > 0.5f);
    refl_along_   = m->data[D_ALONG];
    refl_lateral_ = m->data[D_LATERAL];
    refl_range_   = m->data[D_RANGE];
    refl_nyaw_    = wrap_pi(m->data[D_SKEW] + m->data[D_BEARING] + M_PI);
    last_refl_time_ = now();
  }
  void on_confidence(const std_msgs::msg::Float32::SharedPtr m) { confidence_ = m->data; }
  void on_contact(const std_msgs::msg::UInt8::SharedPtr m) { contact_mask_ = m->data; }
  void on_move_state(const std_msgs::msg::UInt8::SharedPtr m) { move_state_ = m->data; }

  bool refl_fresh() const {
    return last_refl_time_.nanoseconds() != 0 &&
           (now() - last_refl_time_).seconds() < reflector_stale_s_;
  }
  bool refl_usable() const { return refl_fresh() && refl_valid_ && confidence_ >= min_confidence_; }

  // ---- geometry ----
  // Offset from the dock's approach centreline (+ = robot is left of it).
  double offset_from_axis() const {
    return refl_along_ * std::sin(refl_nyaw_) - refl_lateral_ * std::cos(refl_nyaw_);
  }
  // Distance from the robot back to the dock face, measured ALONG the approach axis.
  double axis_distance() const {
    return -refl_along_ * std::cos(refl_nyaw_) - refl_lateral_ * std::sin(refl_nyaw_);
  }
  // Rear-aim error: bearing of a carrot ON the centreline, off dead-astern. Driving the REAR at a
  // point on the axis converges offset and heading TOGETHER -- centring is the act, squareness the
  // byproduct. Carrot sits one lookahead nearer the dock than the robot's own axis projection.
  double rear_aim_error() const {
    const double nx = std::cos(refl_nyaw_), ny = std::sin(refl_nyaw_);
    const double s_r = -(refl_along_ * nx + refl_lateral_ * ny);
    const double look = std::max(0.15, 0.5 * axis_distance());
    const double cx = refl_along_   + (s_r - look) * nx;
    const double cy = refl_lateral_ + (s_r - look) * ny;
    return wrap_pi(std::atan2(cy, cx) - M_PI);
  }

  // ---- segment issue ----
  void issue_segment(int32_t counts_l, int32_t counts_r, const char* what) {
    seg_issue_time_ = now();
    seg_saw_running_ = false;
    seg_desc_ = what;
    segments_++;
    if (dry_run_) {
      RCLCPP_WARN(get_logger(), "[DRY RUN] would issue %s: L=%+d R=%+d @%drpm",
                  what, counts_l, counts_r, static_cast<int>(seg_rpm_));
      set_state(PLAN);      // pretend it completed instantly
      return;
    }
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {counts_l, counts_r, static_cast<int32_t>(seg_rpm_)};
    move_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "seg %d: %s  L=%+d R=%+d @%drpm",
                segments_, what, counts_l, counts_r, static_cast<int>(seg_rpm_));
    set_state(WAIT_SEG);
  }

  void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (state_ != IDLE && state_ != SEATED && state_ != ABORT) {
      res->success = false; res->message = "already running"; return;
    }
    if (!refl_usable()) {
      res->success = false;
      res->message = "no usable dock lock (valid/fresh/confidence) — check the reflector";
      return;
    }
    if (refl_range_ > max_start_range_m_) {
      res->success = false; res->message = "dock too far — stage closer"; return;
    }
    if (std::fabs(offset_from_axis()) > max_lateral_m_) {
      res->success = false; res->message = "too far off the dock axis — restage"; return;
    }
    segments_ = 0; stall_retries_ = 0;
    start_time_ = now(); acquire_start_ = now();
    valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    set_state(ACQUIRE);
    res->success = true; res->message = "acquiring dock lock...";
  }
  void on_cancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    // A zero-length move supersedes any in-flight segment (last command wins in the firmware).
    if (!dry_run_) { std_msgs::msg::Int32MultiArray m; m.data = {0, 0, 5}; move_pub_->publish(m); }
    abort_reason_ = "cancelled by service"; set_state(ABORT);
    res->success = true; res->message = "cancelled — segment superseded";
  }

  void on_tick() {
    const rclcpp::Time t = now();
    switch (state_) {
      case IDLE: case SEATED: case ABORT: return;

      case ACQUIRE: {
        if (!refl_usable()) {
          valid_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          if ((t - acquire_start_).seconds() > acquire_timeout_s_) {
            abort_reason_ = "no stable dock lock"; set_state(ABORT);
          }
          return;
        }
        if (valid_since_.nanoseconds() == 0) valid_since_ = t;
        if ((t - valid_since_).seconds() >= acquire_stable_s_) {
          RCLCPP_INFO(get_logger(),
            "locked: range %.3f  axis_d %.3f  offset %+.3f  aim %+.1f deg  conf %.2f",
            refl_range_, axis_distance(), offset_from_axis(),
            rear_aim_error() * 180.0 / M_PI, confidence_);
          set_state(PLAN);
        }
        return;
      }

      case PLAN: {
        if ((t - start_time_).seconds() > overall_timeout_s_) {
          abort_reason_ = "overall timeout"; set_state(ABORT); return;
        }
        if (segments_ >= max_segments_) {
          abort_reason_ = "segment budget exhausted"; set_state(ABORT); return;
        }
        // Seated already? contact is the ground truth, not the reflector.
        if ((contact_mask_ & 0x03) == 0x03) {
          RCLCPP_INFO(get_logger(), "both prox seated (contact=3) — docked.");
          set_state(SEATED); return;
        }
        if (!refl_usable()) {
          // Close in, the strips may leave the field of view; if we are already committed the
          // rails own the last stretch, so treat a dropout there as "keep going", not failure.
          if (committed_) { set_state(SEAT_WAIT); seat_wait_start_ = t; return; }
          abort_reason_ = "lost dock lock while planning"; set_state(ABORT); return;
        }

        const double d      = axis_distance();
        const double offset = offset_from_axis();
        const double aim    = rear_aim_error();

        if (std::fabs(offset) > max_lateral_m_) {
          abort_reason_ = "drifted off the dock axis"; set_state(ABORT); return;
        }

        // --- COMMIT: inside this range stop re-aiming and drive the remaining distance in.
        //     The funnel rails absorb the residual angle; steering here would fight them.
        if (d <= commit_range_) {
          const double drive = d - seated_range_m_;
          if (drive <= 0.005) { seat_wait_start_ = t; set_state(SEAT_WAIT); return; }
          committed_ = true;
          const int32_t c = static_cast<int32_t>(-drive * counts_per_m_);   // reverse = negative
          RCLCPP_INFO(get_logger(), "COMMIT at d=%.3f offset=%+.3f aim=%+.1fdeg -> final drive %.3f m",
                      d, offset, aim * 180.0 / M_PI, drive);
          issue_segment(c, c, "COMMIT drive");
          return;
        }

        // --- AIM: rotate in place to point the rear at the centreline carrot.
        // SIGN (derived, then verify on hardware — this is the classic trap on this robot):
        //   pure rotation by theta:  s_L = -theta*W/2,  s_R = +theta*W/2
        //   firmware kinematics:     motor1(L) = vx - wz*W,  motor2(R) = vx + wz*W
        //   => CCW (+theta) is LEFT BACKWARD, RIGHT FORWARD, i.e. counts (-c, +c).
        // A rear-right carrot gives aim > 0 and needs CCW, so positive aim -> (-c, +c).
        // aim_sign flips the whole convention if the hardware disagrees; if the offset GROWS
        // instead of shrinking on the first AIM segment, set aim_sign:=-1.0.
        if (std::fabs(aim) > aim_tol_) {
          const int32_t c = static_cast<int32_t>(aim_sign_ * aim * counts_per_rad_);
          RCLCPP_INFO(get_logger(), "AIM %+.1f deg (d=%.3f offset=%+.3f)",
                      aim * 180.0 / M_PI, d, offset);
          issue_segment(-c, c, "AIM rotate");
          return;
        }

        // --- DRIVE: straight back along the axis, one capped segment at a time.
        const double drive = std::min(seg_len_m_, d - commit_range_ + 0.05);
        const int32_t c = static_cast<int32_t>(-drive * counts_per_m_);
        RCLCPP_INFO(get_logger(), "DRIVE %.3f m (d=%.3f offset=%+.3f aim=%+.1fdeg)",
                    drive, d, offset, aim * 180.0 / M_PI);
        issue_segment(c, c, "DRIVE straight");
        return;
      }

      case WAIT_SEG: {
        // Contact can happen mid-segment; it is the ground truth and outranks everything.
        if ((contact_mask_ & 0x03) == 0x03) {
          RCLCPP_INFO(get_logger(), "both prox seated mid-segment — docked.");
          set_state(SEATED); return;
        }
        // The firmware must acknowledge by going RUNNING, or the command never landed.
        if (!seg_saw_running_) {
          if (move_state_ == MV_RUNNING) { seg_saw_running_ = true; }
          else if ((t - seg_issue_time_).seconds() > seg_ack_timeout_s_) {
            abort_reason_ = "firmware never acknowledged the segment (/wheel_move not received?)";
            set_state(ABORT);
          }
          return;
        }
        if (move_state_ == MV_RUNNING) return;      // still executing

        // Terminal state reached.
        switch (move_state_) {
          case MV_DONE:
            RCLCPP_INFO(get_logger(), "seg %d (%s) DONE", segments_, seg_desc_.c_str());
            stall_retries_ = 0;
            if (committed_) { seat_wait_start_ = t; set_state(SEAT_WAIT); }
            else            { set_state(PLAN); }
            return;
          case MV_STALL:
            // A stall near the seat is usually ARRIVAL, not failure — the dock is stopping us.
            if (committed_ || refl_range_ < commit_range_) {
              RCLCPP_WARN(get_logger(), "stall while committed (range %.3f) — treating as arrival.",
                          refl_range_);
              seat_wait_start_ = t; set_state(SEAT_WAIT); return;
            }
            if (++stall_retries_ <= max_stall_retries_) {
              RCLCPP_WARN(get_logger(), "seg %d STALLED — re-planning (retry %d/%d)",
                          segments_, stall_retries_, static_cast<int>(max_stall_retries_));
              set_state(PLAN); return;
            }
            abort_reason_ = "repeated stalls — wheels blocked or floor too grippy";
            set_state(ABORT); return;
          case MV_DIVERGE:
            abort_reason_ = "firmware DIVERGE — wheel travelling the wrong way (encoder sign?)";
            set_state(ABORT); return;
          case MV_REJECT:
            abort_reason_ = "firmware REJECTED the segment (too long / malformed)";
            set_state(ABORT); return;
          case MV_TIMEOUT:
            abort_reason_ = "firmware segment TIMEOUT"; set_state(ABORT); return;
          default:
            RCLCPP_WARN(get_logger(), "unexpected move_state %s — re-planning",
                        move_state_name(move_state_));
            set_state(PLAN); return;
        }
      }

      case SEAT_WAIT: {
        // Final stretch: the rails and the firmware's seat reflex own it. Just watch the prox.
        if ((contact_mask_ & 0x03) == 0x03) {
          RCLCPP_INFO(get_logger(), "both prox seated (contact=3) — docked.");
          set_state(SEATED); return;
        }
        if ((t - seat_wait_start_).seconds() > seat_settle_s_) {
          if ((contact_mask_ & 0x03) != 0) {
            RCLCPP_WARN(get_logger(),
              "partial seat (contact=%u, left=%d right=%d) after %.1fs — accepting honestly.",
              contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0,
              seat_settle_s_);
            set_state(SEATED); return;
          }
          abort_reason_ = "reached the seat with NO prox contact — arrived off-centre";
          set_state(ABORT); return;
        }
        return;
      }
    }
  }

  // params
  std::string reflector_topic_, confidence_topic_, contact_topic_,
              move_cmd_topic_, move_state_topic_, state_topic_;
  double counts_per_m_, wheel_separation_, counts_per_rad_;
  double seg_len_m_, seg_rpm_, aim_tol_, aim_sign_, commit_range_, max_segments_;
  double seated_range_m_, seat_settle_s_;
  double min_confidence_, reflector_stale_s_, acquire_stable_s_, acquire_timeout_s_,
         overall_timeout_s_, seg_ack_timeout_s_, max_stall_retries_,
         max_lateral_m_, max_start_range_m_, control_hz_;
  bool   dry_run_;

  // live state
  bool    refl_valid_{false}, seg_saw_running_{false}, committed_{false};
  double  refl_along_{0}, refl_lateral_{0}, refl_range_{0}, refl_nyaw_{0}, confidence_{0};
  uint8_t contact_mask_{0}, move_state_{MV_IDLE};
  int     segments_{0}, stall_retries_{0};
  std::string seg_desc_, abort_reason_;
  rclcpp::Time last_refl_time_{0,0,RCL_ROS_TIME}, acquire_start_{0,0,RCL_ROS_TIME},
               valid_since_{0,0,RCL_ROS_TIME}, start_time_{0,0,RCL_ROS_TIME},
               seg_issue_time_{0,0,RCL_ROS_TIME}, seat_wait_start_{0,0,RCL_ROS_TIME};
  State state_{IDLE};

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr move_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr refl_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           conf_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr             contact_sub_, move_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, cancel_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockAlignerV3>());
  rclcpp::shutdown();
  return 0;
}
