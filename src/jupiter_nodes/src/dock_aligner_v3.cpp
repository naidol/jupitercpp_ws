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
    // SEGMENT LENGTH is the OUTER LOOP'S SAMPLING INTERVAL — the dock is only re-measured
    // BETWEEN segments (inside one, the firmware drives on encoders alone and is blind to the
    // dock). So a 0.25 m segment means the dock is sampled every 250 mm. Close in, that is far
    // too coarse: the whole final approach got ONE look.
    // Logan, 2026-08-11: stalls cluster on the FIRST move after a long rest, not on subsequent
    // ones — T2 ran 4 segments with zero stalls, and yesterday's runs stalled only on segment 1.
    // Strictly it is not momentum carrying through (the firmware BRAKES between segments, both
    // bridge inputs HIGH), it is that stiction has not re-set and the caster is still trailing
    // correctly after a ~200 ms pause. Either way short segments are cheap, so sample far more
    // often where it matters.
    seg_len_m_        = declare_parameter("seg_len_m",        0.25);   // segment cap, far out
    seg_len_close_m_  = declare_parameter("seg_len_close_m",  0.10);   // segment cap, close in
    seg_close_range_  = declare_parameter("seg_close_range",  0.60);   // below this range, use the short cap
    // STALL ESCALATION (2026-08-11). Cold-start stiction is the dominant failure: T1 and T3 both
    // stalled 3x on the FIRST move after the robot had been standing, while T2 -- run immediately
    // after the robot had been driven -- completed every segment. The retries were useless
    // because each one re-issued at the SAME rpm, so it was three identical attempts against the
    // same stiction. Each retry now escalates the commanded speed, which in this scheme escalates
    // the FORCE a stopped wheel pushes with (duty ~ K_P * rpm + FF).
    stall_rpm_boost_  = declare_parameter("stall_rpm_boost",  0.6);    // +60 % of base rpm per retry
    // 12 -> 25 (2026-08-11, measured). At 16 rpm EVERY arc segment stalled and every boosted
    // retry at 25 completed -- three of each, no ambiguity. 16 is simply below this robot's
    // breakaway threshold on this floor; arcs need more than straights because the differential
    // means one wheel is always doing less. At 25 the whole approach ran 4/4 with zero stalls.
    seg_rpm_          = declare_parameter("seg_rpm",          25);     // precision speed: overshoot scales with this
    aim_tol_          = declare_parameter("aim_tol_deg",      2.0) * M_PI / 180.0;
    // Flips the rotation convention wholesale. Derived value is +1 (see the AIM block); if the
    // first AIM segment makes the offset GROW instead of shrink, set this to -1.0 and re-test.
    aim_sign_         = declare_parameter("aim_sign",         1.0);
    // ARC steering (2026-08-10). In-place rotation is the WORST move this chassis can make: the
    // single rear caster sits 180 mm behind the drive axle, so a pivot demands it swivel 90 deg
    // from rest under load -- it behaves as a locked skid and the segment stalls. Measured: three
    // consecutive AIM segments (60/54/57 counts) stalled with the robot barely moving, while every
    // straight DRIVE completed. So turn WHILE MOVING: over an arc of radius R the caster only
    // swivels atan(0.180/R) -- about 5 deg at R=2 m, 13 deg at R=0.8 m. That keeps the robot in
    // the regime that demonstrably works, and folds AIM+DRIVE into one primitive.
    arc_gain_         = declare_parameter("arc_gain",         0.6);   // fraction of the aim error corrected per segment
    // HEADING TERM (Logan's observation, 2026-08-11). Pure pursuit converges POSITION but leaves
    // terminal HEADING to fall out of whatever path was taken -- a known property. Measured: a run
    // reached lateral -0.0007 m (0.7 mm off centre, essentially perfect) yet -2.2 deg of skew. The
    // rear prox sit +-150 mm from centre, so 2.2 deg puts one ~6 mm out, past an inductive sensor's
    // range. Being centred is no use if you arrive crooked.
    //   aim  = bearing to the carrot          -> POSITION error
    //   nyaw = heading vs the dock normal     -> HEADING error   (was never used for control)
    // theta = arc_gain * (aim + heading_gain * nyaw) makes this a POSE controller instead of a
    // pure-pursuit one. heading_gain ramps in as the robot closes: far out get CENTRED, close in
    // also get SQUARE. Sign convention matches aim -- rotating CCW by nyaw drives nyaw to zero.
    heading_gain_     = declare_parameter("heading_gain",     0.8);   // weight at/below heading_full_range
    heading_full_range_ = declare_parameter("heading_full_range", 0.45);  // m: full weight at/below this
    heading_zero_range_ = declare_parameter("heading_zero_range", 0.90);  // m: no weight at/above this
    min_turn_radius_  = declare_parameter("min_turn_radius",  0.80);  // m — caps curvature -> caps caster swivel
    pivot_fallback_   = declare_parameter("pivot_fallback_deg", 25.0) * M_PI / 180.0;
    // CARROT LOOKAHEAD. look = max(look_min, look_frac * axis_distance). The floor matters more
    // than it looks: with only 0.15 m the carrot sits very close as the robot closes in, so the
    // SAME lateral offset produces a much larger aim angle and the loop over-reacts exactly when
    // it should be settling. Measured 2026-08-10: aim swung +17.9 -> -18.4 deg between segments,
    // leaving +2.9 deg of skew at the seat and only one prox engaged.
    look_min_         = declare_parameter("look_min",         0.35);
    look_frac_        = declare_parameter("look_frac",        0.5);
    // COMMIT_RANGE is where the robot goes BLIND to the dock for good. The rails argument only
    // applies INSIDE the throat (~0.28 m) — committing at 0.40 left a 12 cm band with no sensor
    // AND no rails guiding anything, which is where the 2026-08-11 left-rail strike happened
    // (entered the commit at offset -0.019, came out 23 mm and 12 deg off). The detector is still
    // excellent down there — 127 points at 0.95 confidence at 0.196 m — so that was good data
    // being discarded at the moment it mattered most. Commit at the throat, not before it.
    commit_range_     = declare_parameter("commit_range",     0.30);   // below this: no more re-aim, drive in
    max_segments_     = declare_parameter("max_segments",     25);     // don't loop forever

    // --- target pose. RE-MEASURED 2026-08-11, robot hand-seated at contact=3, 6 samples:
    // along -0.19292, lateral -0.00476, skew -0.70 deg -> nyaw +0.712 deg, and the value V3
    // actually steers on, AXIS DISTANCE = 0.1930 m.
    //
    // The same capture the day before read lateral +0.0039; today -0.0048 -- the SIGN FLIPPED
    // between two valid seatings. So the dock has roughly +-5 mm of lateral play when seated,
    // and there is no precise lateral target worth aiming at. offset_from_axis at the seat came
    // out +2.36 mm; that is deliberately NOT used as a target, because it is inside the noise.
    // Only the axis distance is repeatable enough to calibrate on.
    seated_range_m_   = declare_parameter("seated_range_m",   0.1930);
    seat_settle_s_    = declare_parameter("seat_settle_s",    2.0);
    // FINAL PUSH (measured 2026-08-10): at 12 rpm (0.063 m/s) the robot stopped 12 mm short of
    // the seat with NO prox contact -- a gentle creep cannot overcome the funnel rails'
    // steel-on-PLA friction. V1 used 0.14 m/s for exactly this. And OVERDRIVE past the nominal
    // seated range: the firmware's prox reflex ends the move the instant both sensors confirm, so
    // aiming deliberately deep lets CONTACT be what stops us instead of a distance estimate. If
    // contact never comes, the move simply stalls -- which we already treat as arrival.
    // Commanded RPM sets BOTH the force a blocked wheel pushes with (duty ~ K_P * rpm) AND the
    // speed the robot enters the funnel at -- they are coupled in this scheme, which is the
    // tension here. 22 was too weak to close the last 10 mm. 40 seated once, then on 2026-08-11
    // carried the robot into the LEFT RAIL at ~0.21 m/s: it entered the commit at offset -0.019
    // and came out 23 mm and 12 deg off, having been deflected inside the throat. 30 keeps most
    // of the force while cutting the entry energy. Paired with MOVE_STALL_MS 1200 in firmware,
    // which stops the stall guard cutting the push off before the integral adds its share.
    commit_rpm_       = declare_parameter("commit_rpm",       30);
    commit_overdrive_m_ = declare_parameter("commit_overdrive_m", 0.030);
    // SEAT NUDGE. Arriving square but a few mm off centre leaves ONE rear corner off its plate
    // (measured 2026-08-10: skew +0.7 deg, lateral -0.008 -> contact=2, right seated). Pivot
    // about the SEATED corner to swing the open one in, while still easing inward. Unlike a
    // free-space pivot -- which stalls on this chassis because the caster must swivel 90 deg --
    // one corner is already against the dock, so the dock provides the pivot point.
    // Sign is DERIVED, matching V1: contact=1 (LEFT seated, right open) -> CW, i.e. theta < 0;
    // contact=2 (RIGHT seated, left open) -> CCW, theta > 0.
    nudge_deg_        = declare_parameter("nudge_deg",        3.0);
    nudge_push_m_     = declare_parameter("nudge_push_m",     0.015);
    nudge_rpm_        = declare_parameter("nudge_rpm",        30);
    // Nudging is DISABLED by default (was 3). Proven ineffective 2026-08-11: with one corner
    // seated and the tongue in the throat there is nothing to pivot about -- three nudges at
    // 20 rpm (~26 % duty) did not move the robot at all. The robot must be FREED before it can
    // be corrected, which is what the back-off retry below does.
    max_nudges_       = declare_parameter("max_nudges",       0);

    // BACK-OFF RETRY (Logan's call, 2026-08-11). On a partial seat, reverse OUT far enough to be
    // clear of the throat and above commit_range, then re-plan and re-approach. Rationale: the
    // residual error at the seat is HEADING, not position -- a run reached lateral -0.0007 m
    // (0.7 mm!) but 2.2 deg of skew, and 2.2 deg across the ~300 mm prox spacing is ~11 mm at the
    // corners, past an inductive sensor's ~8 mm range. That skew is geometrically coupled to the
    // residual offset through the carrot lookahead, so it cannot be nudged out in place -- but a
    // fresh approach from a freed position gets another attempt at nulling it.
    // TWO mechanisms, cheapest first (Logan, 2026-08-11):
    //   SHORT back-off (~50 mm) + straight re-push -> unjams and gives the funnel RAILS another
    //       go at squaring the robot on re-entry. No sensor involved, ~2 s.
    //   LONG back-off (past commit_range) + full re-approach -> sensor-guided correction, ~15 s.
    //       Only worth it if the short retry fails, and it is useless if the residual skew turns
    //       out to be SYSTEMATIC (the same approach would just reproduce it).
    short_backoff_m_    = declare_parameter("short_backoff_m",    0.05);
    // 2 -> 0: DISPROVEN 2026-08-11. Two consecutive short back-off + re-push cycles produced
    // IDENTICAL contact=2 both times -- the rails do not re-square the robot on re-entry.
    short_backoff_tries_= declare_parameter("short_backoff_tries", 0);
    max_seat_retries_ = declare_parameter("max_seat_retries", 3);     // total, short + long
    backoff_target_m_ = declare_parameter("backoff_target_m", 0.38);  // LONG back-off target range
    backoff_rpm_      = declare_parameter("backoff_rpm",      25);

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
      "dock_aligner_v3 (SEGMENTED/POSITION) ready. seg=%.2f/%.2fm @%drpm aim_tol=%.1fdeg "
      "commit=%.2fm seated_range=%.4fm counts/m=%.1f counts/rad=%.1f%s. Call /dock/v3/align_start.",
      seg_len_m_, seg_len_close_m_, static_cast<int>(seg_rpm_), aim_tol_ * 180.0 / M_PI, commit_range_,
      seated_range_m_, counts_per_m_, counts_per_rad_, dry_run_ ? "  [DRY RUN]" : "");
  }

private:
  enum State { IDLE, ACQUIRE, PLAN, WAIT_SEG, SEAT_WAIT, SEAT_NUDGE, SEATED, ABORT };
  const char* state_name(State s) const {
    switch (s) { case IDLE: return "IDLE"; case ACQUIRE: return "ACQUIRE"; case PLAN: return "PLAN";
                 case WAIT_SEG: return "WAIT_SEG"; case SEAT_WAIT: return "SEAT_WAIT";
                 case SEAT_NUDGE: return "SEAT_NUDGE";
                 case SEATED: return "SEATED"; default: return "ABORT"; }
  }
  void set_state(State s) {
    state_ = s;
    std_msgs::msg::String m; m.data = state_name(s);
    state_pub_->publish(m);
    // Always surface WHY we aborted. Storing the reason and never printing it made a live abort
    // undiagnosable (2026-08-10) — the run just ended with no explanation in the log.
    if (s == ABORT && !abort_reason_.empty()) {
      RCLCPP_ERROR(get_logger(), "state -> ABORT: %s  [contact=%u range=%.3f offset=%+.3f]",
                   abort_reason_.c_str(), contact_mask_, refl_range_, offset_from_axis());
    } else {
      RCLCPP_INFO(get_logger(), "state -> %s", state_name(s));
    }
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
    const double look = std::max(look_min_, look_frac_ * axis_distance());
    const double cx = refl_along_   + (s_r - look) * nx;
    const double cy = refl_lateral_ + (s_r - look) * ny;
    return wrap_pi(std::atan2(cy, cx) - M_PI);
  }

  // ---- segment issue ----
  void issue_segment(int32_t counts_l, int32_t counts_r, const char* what, double rpm) {
    seg_issue_time_ = now();
    seg_saw_running_ = false;
    seg_desc_ = what;
    segments_++;
    if (dry_run_) {
      RCLCPP_WARN(get_logger(), "[DRY RUN] would issue %s: L=%+d R=%+d @%drpm",
                  what, counts_l, counts_r, static_cast<int>(rpm));
      set_state(PLAN);      // pretend it completed instantly
      return;
    }
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {counts_l, counts_r, static_cast<int32_t>(rpm)};
    move_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "seg %d: %s  L=%+d R=%+d @%drpm",
                segments_, what, counts_l, counts_r, static_cast<int>(rpm));
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
    segments_ = 0; stall_retries_ = 0; nudges_ = 0; seat_retries_ = 0; committed_ = false;
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

        // Escalate speed (= force) on each stall retry; resets to base after any DONE segment.
        // Declared here so COMMIT, PIVOT and ARC can all use it.
        const double eff_rpm = std::min(60.0, seg_rpm_ * (1.0 + stall_rpm_boost_ * stall_retries_));

        // --- COMMIT: inside this range stop re-aiming and drive the remaining distance in.
        //     The funnel rails absorb the residual angle; steering here would fight them.
        if (d <= commit_range_) {
          // Aim PAST the nominal seat: the firmware's prox reflex ends the move the moment both
          // sensors confirm, so let CONTACT stop us rather than a distance estimate. Firmer rpm
          // too -- a gentle creep cannot beat the rails' friction.
          const double drive = d - seated_range_m_ + commit_overdrive_m_;
          if (drive <= 0.005) { seat_wait_start_ = t; set_state(SEAT_WAIT); return; }
          committed_ = true;
          const int32_t c = static_cast<int32_t>(-drive * counts_per_m_);   // reverse = negative
          RCLCPP_INFO(get_logger(),
            "COMMIT at d=%.3f offset=%+.3f aim=%+.1fdeg -> push %.3f m (incl %.3f overdrive) @%drpm",
            d, offset, aim * 180.0 / M_PI, drive, commit_overdrive_m_, static_cast<int>(commit_rpm_));
          issue_segment(c, c, "COMMIT drive", commit_rpm_);
          return;
        }

        // --- PIVOT FALLBACK: only for gross misalignment, where no arc can recover inside the
        //     remaining distance. Known to be unreliable on this chassis (90 deg caster swivel
        //     from rest) -- it is a last resort, not the normal path.
        //     SIGN (verified in dry run + on hardware): pure rotation is s_L = -theta*W/2,
        //     s_R = +theta*W/2; firmware has motor1(L) = vx - wz*W, so CCW is LEFT BACKWARD,
        //     RIGHT FORWARD = counts (-c, +c). A rear-right carrot gives aim > 0 and needs CCW.
        if (std::fabs(aim) > pivot_fallback_) {
          const int32_t c = static_cast<int32_t>(aim_sign_ * aim * counts_per_rad_);
          RCLCPP_WARN(get_logger(),
            "aim %+.1f deg exceeds arc authority — PIVOT fallback (may stall: caster must swivel)",
            aim * 180.0 / M_PI);
          issue_segment(-c, c, "PIVOT rotate", eff_rpm);
          return;
        }

        // --- ARC: turn WHILE reversing. One primitive replaces AIM+DRIVE.
        //     Centre travel is -s (reversing); rotation theta is applied across it:
        //         s_L = -s - theta*W/2      s_R = -s + theta*W/2
        //     which reduces to the pivot form above when s = 0, so the sign convention is shared.
        //     Curvature is capped by min_turn_radius so the caster is never asked for a large
        //     swivel: theta_max = s / R_min.
        // Sample the dock more often as we close in — segment length IS the outer loop's
        // sampling interval (see the seg_len note above).
        const double cap = (d <= seg_close_range_) ? seg_len_close_m_ : seg_len_m_;
        const double s_travel = std::min(cap, d - commit_range_ + 0.05);
        // Blend the heading term in as we close: 0 above heading_zero_range, full at/below
        // heading_full_range, linear between. Far out, chasing heading would fight the approach;
        // close in, it is the difference between contact=3 and a partial seat.
        double hw = 0.0;
        if (d <= heading_full_range_)      hw = 1.0;
        else if (d < heading_zero_range_)  hw = (heading_zero_range_ - d) /
                                                (heading_zero_range_ - heading_full_range_);
        const double heading_err = hw * heading_gain_ * refl_nyaw_;
        double theta = aim_sign_ * arc_gain_ * (aim + heading_err);
        const double theta_max = s_travel / min_turn_radius_;
        theta = clampd(theta, -theta_max, theta_max);

        const double half   = 0.5 * wheel_separation_;
        const int32_t cl = static_cast<int32_t>((-s_travel - theta * half) * counts_per_m_);
        const int32_t cr = static_cast<int32_t>((-s_travel + theta * half) * counts_per_m_);
        const double  radius = (std::fabs(theta) > 1e-6) ? (s_travel / std::fabs(theta)) : 999.0;
        RCLCPP_INFO(get_logger(),
          "ARC %.3f m turning %+.1f deg (R=%.2fm, caster swivel ~%.0f deg) @%drpm%s "
          "[d=%.3f offset=%+.3f aim=%+.1f nyaw=%+.1f hw=%.2f]",
          s_travel, theta * 180.0 / M_PI, radius,
          std::atan(0.180 / std::max(0.2, radius)) * 180.0 / M_PI,
          static_cast<int>(eff_rpm), stall_retries_ ? " [BOOSTED]" : "",
          d, offset, aim * 180.0 / M_PI, refl_nyaw_ * 180.0 / M_PI, hw);
        issue_segment(cl, cr, "ARC", eff_rpm);
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

        // BACK-OFF segments are routed on what the segment WAS, not on committed_. A back-off is
        // "committed" yet moving AWAY from the dock, so the generic "terminal while committed =
        // arrival" rule is exactly backwards for it -- 2026-08-11 a short back-off timed out and
        // was declared an arrival while the robot sat 50 mm OUT of the dock, so the re-push never
        // ran. Timeout/stall on a back-off is fine: it only has to get clear, not hit a target.
        if (seg_desc_ == "BACK-OFF short" &&
            (move_state_ == MV_DONE || move_state_ == MV_STALL || move_state_ == MV_TIMEOUT)) {
          const double push = short_backoff_m_ + commit_overdrive_m_;
          const int32_t c = static_cast<int32_t>(-push * counts_per_m_);
          RCLCPP_INFO(get_logger(), "back-off ended (%s) — re-pushing %.3f m so the rails can re-square",
                      move_state_name(move_state_), push);
          issue_segment(c, c, "RE-PUSH", commit_rpm_);
          return;
        }
        if (seg_desc_ == "BACK-OFF long" &&
            (move_state_ == MV_DONE || move_state_ == MV_STALL || move_state_ == MV_TIMEOUT)) {
          RCLCPP_INFO(get_logger(), "long back-off ended (%s) — re-planning a fresh approach",
                      move_state_name(move_state_));
          committed_ = false;
          set_state(PLAN);
          return;
        }

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
            // Only a COMMITTED segment may be treated as arrival. Being merely NEAR the dock is
            // not the same as having made the final push -- 2026-08-11 an ordinary arc happened
            // to end below commit_range, the old "|| refl_range_ < commit_range_" clause declared
            // arrival, and the COMMIT segment was never issued at all. The robot sat 64 mm short
            // with nothing driving it in. If we are close but not committed, re-plan: PLAN will
            // immediately issue the COMMIT because d <= commit_range.
            if (committed_) {
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
            // A timeout WHILE COMMITTED means the same as a stall while committed: we are at the
            // dock and the rails are what slowed us. Measured 2026-08-10: pushing through the
            // funnel the robot CREEPS, making >MOVE_STALL_MIN_COUNTS of progress inside every
            // stall window, so the stall guard never fires -- it runs out the segment budget
            // instead. Treating this as a hard abort skipped SEAT_WAIT entirely and meant the
            // seat nudge never ran, with contact=2 sitting right there.
            // Same rule as the stall case: only a COMMITTED segment counts as arrival.
            if (committed_) {
              RCLCPP_WARN(get_logger(),
                "segment timeout while committed (range %.3f, contact=%u) — treating as arrival.",
                refl_range_, contact_mask_);
              seat_wait_start_ = t; set_state(SEAT_WAIT); return;
            }
            // Not committed but a segment timed out near the dock -> re-plan so COMMIT can run.
            if (refl_range_ < commit_range_ + 0.05) {
              RCLCPP_WARN(get_logger(),
                "segment timeout at range %.3f (not yet committed) — re-planning to COMMIT.",
                refl_range_);
              set_state(PLAN); return;
            }
            abort_reason_ = "firmware segment TIMEOUT"; set_state(ABORT); return;
          default:
            RCLCPP_WARN(get_logger(), "unexpected move_state %s (%u) — re-planning",
                        move_state_name(move_state_), move_state_);
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
            if (nudges_ < static_cast<int>(max_nudges_)) {
              // ONE corner is down. Pivot about it to bring the other in.
              const bool left_seated = (contact_mask_ & 0x01) != 0;
              const double theta = (left_seated ? -1.0 : +1.0) * nudge_deg_ * M_PI / 180.0;
              const double half  = 0.5 * wheel_separation_;
              const int32_t cl = static_cast<int32_t>((-nudge_push_m_ - theta * half) * counts_per_m_);
              const int32_t cr = static_cast<int32_t>((-nudge_push_m_ + theta * half) * counts_per_m_);
              nudges_++;
              RCLCPP_WARN(get_logger(),
                "partial seat (contact=%u, left=%d right=%d) — NUDGE %d/%d: pivot %+.1f deg about the seated corner",
                contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0,
                nudges_, static_cast<int>(max_nudges_), theta * 180.0 / M_PI);
              issue_segment(cl, cr, "SEAT nudge", nudge_rpm_);
              return;
            }
            // BACK OFF and try the approach again from a position where the robot is FREE.
            if (seat_retries_ < static_cast<int>(max_seat_retries_)) {
              seat_retries_++;
              nudges_ = 0;
              const bool use_short = (seat_retries_ <= static_cast<int>(short_backoff_tries_));
              double back;
              if (use_short) {
                // SHORT: just unjam. committed_ STAYS true so the next PLAN drives straight back
                // in and the rails get another go at squaring it. No re-approach, no sensor.
                back = short_backoff_m_;
              } else {
                // LONG: clear the throat and get above commit_range so PLAN actually re-approaches.
                back = std::max(0.06, backoff_target_m_ - refl_range_);
                committed_ = false;
              }
              const int32_t c = static_cast<int32_t>(+back * counts_per_m_);   // POSITIVE = away
              RCLCPP_WARN(get_logger(),
                "partial seat (contact=%u, left=%d right=%d) — %s BACK-OFF %d/%d: out %.3f m",
                contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0,
                use_short ? "SHORT (rails re-square)" : "LONG (re-approach)",
                seat_retries_, static_cast<int>(max_seat_retries_), back);
              issue_segment(c, c, use_short ? "BACK-OFF short" : "BACK-OFF long", backoff_rpm_);
              return;
            }
            RCLCPP_WARN(get_logger(),
              "partial seat (contact=%u, left=%d right=%d) after %d back-off retries — accepting honestly.",
              contact_mask_, (contact_mask_ & 1) ? 1 : 0, (contact_mask_ & 2) ? 1 : 0, seat_retries_);
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
  double seg_len_m_, seg_len_close_m_, seg_close_range_, stall_rpm_boost_;
  double seg_rpm_, aim_tol_, aim_sign_, commit_range_, max_segments_;
  double arc_gain_, min_turn_radius_, pivot_fallback_, look_min_, look_frac_;
  double heading_gain_, heading_full_range_, heading_zero_range_;
  double seated_range_m_, seat_settle_s_, commit_rpm_, commit_overdrive_m_;
  double nudge_deg_, nudge_push_m_, nudge_rpm_, max_nudges_;
  double max_seat_retries_, backoff_target_m_, backoff_rpm_;
  double short_backoff_m_, short_backoff_tries_;
  double min_confidence_, reflector_stale_s_, acquire_stable_s_, acquire_timeout_s_,
         overall_timeout_s_, seg_ack_timeout_s_, max_stall_retries_,
         max_lateral_m_, max_start_range_m_, control_hz_;
  bool   dry_run_;

  // live state
  bool    refl_valid_{false}, seg_saw_running_{false}, committed_{false};
  double  refl_along_{0}, refl_lateral_{0}, refl_range_{0}, refl_nyaw_{0}, confidence_{0};
  uint8_t contact_mask_{0}, move_state_{MV_IDLE};
  int     segments_{0}, stall_retries_{0}, nudges_{0}, seat_retries_{0};
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
