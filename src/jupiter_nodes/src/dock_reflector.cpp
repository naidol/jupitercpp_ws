// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_reflector — retro-reflective dock detection from the S2E LiDAR (THREE-STRIP + fallback).
//
// WHY THREE STRIPS (2026-08 update): two side strips are enough to infer midpoint+normal, but
// close-range lateral bias can still accumulate. Adding a CENTER strip gives a direct midpoint
// observation, reducing ambiguity at the funnel mouth. This node now prefers a validated
// LEFT-CENTER-RIGHT triplet and falls back to side-pair geometry when only two strips are visible.
//
// STRIPS: left/right vertical retro strips ~250 mm centre-to-centre + one center strip at
// midpoint, all spanning the S2E scan-plane height.
//
// METHOD:
//   1. Keep scan points with intensity >= threshold (default 40) inside a range window.
//   2. Segment into angularly-contiguous clusters; keep every cluster with >= min_points.
//   3. Prefer a validated LEFT-CENTER-RIGHT triplet among strongest clusters.
//   4. VALIDATE triplet: left-right separation ~= baseline_expected, left-center and
//      center-right ~= baseline_expected/2, and center close to side-midpoint.
//   5. If no valid triplet, fall back to the best validated left-right pair.
//   6. Dock CENTRE = center strip centroid (triplet mode) or side-strip midpoint (fallback).
//      Dock NORMAL = perpendicular to left-right baseline, signed toward the lidar (dock->robot).
//   6. TF the centre+normal pose from base_laser into base_footprint.
//
// PUBLISHES (interface unchanged so the aligner is untouched):
//   geometry_msgs/PoseStamped  /dock/reflector_pose   dock face in base_footprint:
//        position = strip-pair midpoint; orientation yaw = outward normal (points at robot).
//   std_msgs/Float32MultiArray /dock/reflector        self-describing debug (echo-able):
//        [0] valid(1/0)  [1] along_x m (fore/aft)  [2] lateral_y m (+left)
//        [3] range m     [4] bearing rad (0=front,+left)  [5] skew rad (0=squared)
//        [6] baseline m (measured strip separation)  [7] baseline_err m  [8] n_points (both)
//
// All frames/topics/thresholds are parameters — no hardcoded config (house rule).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {
// /dock/reflector index convention (keep in sync with the header block above).
constexpr int D_VALID    = 0;
constexpr int D_ALONG    = 1;
constexpr int D_LATERAL  = 2;
constexpr int D_RANGE    = 3;
constexpr int D_BEARING  = 4;
constexpr int D_SKEW     = 5;
constexpr int D_BASELINE = 6;   // was strip_len — now measured strip separation
constexpr int D_BASE_ERR = 7;   // was fit_rms   — now |measured - expected| baseline
constexpr int D_N_POINTS = 8;
constexpr int D_SIZE     = 9;

double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }
}  // namespace

class DockReflector : public rclcpp::Node {
public:
  DockReflector() : Node("dock_reflector") {
    scan_topic_    = declare_parameter<std::string>("scan_topic",  "/scan");
    pose_topic_    = declare_parameter<std::string>("pose_topic",  "/dock/reflector_pose");
    debug_topic_   = declare_parameter<std::string>("debug_topic", "/dock/reflector");
    confidence_topic_ = declare_parameter<std::string>("confidence_topic", "/dock/reflector_confidence");
    target_frame_  = declare_parameter<std::string>("target_frame", "base_footprint");

    intensity_min_ = declare_parameter("intensity_min", 40.0);   // strip saturates 63; room <=33
    range_min_     = declare_parameter("range_min",     0.10);
    range_max_     = declare_parameter("range_max",     4.0);
    max_gap_deg_   = declare_parameter("max_gap_deg",   2.0);     // split clusters across a wider angular gap
    min_points_    = declare_parameter("min_points",    4);       // per cluster; narrow strips give few beams at range

    // Side-strip geometry validation (rigid, range-independent).
    baseline_expected_ = declare_parameter("baseline_expected", 0.25);   // m, centre-to-centre
    baseline_tol_      = declare_parameter("baseline_tol",      0.05);    // m accept window

    // Three-strip geometry validation.
    center_spacing_expected_ = declare_parameter("center_spacing_expected", baseline_expected_ * 0.5);
    center_spacing_tol_      = declare_parameter("center_spacing_tol", 0.035);
    center_midpoint_tol_     = declare_parameter("center_midpoint_tol", 0.03);
    triplet_search_top_k_    = declare_parameter("triplet_search_top_k", 5);

    // Confidence tuning.
    confidence_min_points_       = declare_parameter("confidence_min_points", 6);
    confidence_full_points_      = declare_parameter("confidence_full_points", 24);
    confidence_alpha_            = declare_parameter("confidence_alpha", 0.35);
    continuity_baseline_jump_m_  = declare_parameter("continuity_baseline_jump_m", 0.04);
    continuity_bearing_jump_deg_ = declare_parameter("continuity_bearing_jump_deg", 12.0);
    continuity_range_jump_m_     = declare_parameter("continuity_range_jump_m", 0.15);

    pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(debug_topic_, 10);
    confidence_pub_ = create_publisher<std_msgs::msg::Float32>(confidence_topic_, 10);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DockReflector::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "dock_reflector (THREE-STRIP aware): %s (I>=%.0f, %.2f-%.2f m) baseline %.0f+/-%.0f mm, "
      "center spacing %.0f+/-%.0f mm -> %s + %s + %s in %s.",
      scan_topic_.c_str(), intensity_min_, range_min_, range_max_,
      baseline_expected_ * 1000.0, baseline_tol_ * 1000.0,
      center_spacing_expected_ * 1000.0, center_spacing_tol_ * 1000.0,
      pose_topic_.c_str(), debug_topic_.c_str(), confidence_topic_.c_str(), target_frame_.c_str());
  }

private:
  void publish_confidence(float confidence) {
    std_msgs::msg::Float32 msg;
    msg.data = confidence;
    confidence_pub_->publish(msg);
  }

  void publish_invalid() {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label  = "valid,along,lateral,range,bearing,skew,baseline,base_err,n_points";
    msg.layout.dim[0].size   = D_SIZE;
    msg.layout.dim[0].stride = D_SIZE;
    msg.data.assign(D_SIZE, 0.0f);
    debug_pub_->publish(msg);

    filtered_confidence_ = 0.0;
    publish_confidence(0.0f);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    const bool have_intensities = (scan->intensities.size() == scan->ranges.size());
    if (!have_intensities) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "scan has no per-point intensities — reflector detection impossible.");
      publish_invalid();
      return;
    }

    // --- 1) above-threshold points, kept as (index, x, y) in the laser frame ---
    struct BrightPt { int idx; double x; double y; };
    std::vector<BrightPt> bright;
    bright.reserve(128);
    double ang = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i, ang += scan->angle_increment) {
      const float r = scan->ranges[i];
      if (scan->intensities[i] >= intensity_min_ && std::isfinite(r) &&
          r > range_min_ && r < range_max_) {
        bright.push_back({static_cast<int>(i), r * std::cos(ang), r * std::sin(ang)});
      }
    }
    if (static_cast<int>(bright.size()) < 2 * min_points_) {
      publish_invalid();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "no dock: only %zu bright points (need >= %d for two strips).",
        bright.size(), 2 * min_points_);
      return;
    }

    // --- 2) segment into angularly-contiguous clusters (each = one vertical strip) ---
    const int gap_bins = std::max(1, static_cast<int>(
        std::ceil((max_gap_deg_ * M_PI / 180.0) / scan->angle_increment)));
    struct Cluster { int count; double cx; double cy; };
    std::vector<Cluster> clusters;
    size_t run_lo = 0;
    for (size_t k = 1; k <= bright.size(); ++k) {
      const bool split = (k == bright.size()) ||
                         (bright[k].idx - bright[k - 1].idx > gap_bins);
      if (split) {
        const int cnt = static_cast<int>(k - run_lo);
        if (cnt >= min_points_) {
          double cx = 0.0, cy = 0.0;
          for (size_t j = run_lo; j < k; ++j) { cx += bright[j].x; cy += bright[j].y; }
          clusters.push_back({cnt, cx / cnt, cy / cnt});
        }
        run_lo = k;
      }
    }
    if (clusters.size() < 2) {
      publish_invalid();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "dock needs TWO strips — found %zu qualifying cluster(s).", clusters.size());
      return;
    }

    // --- 3/4) choose best geometry: validated L-C-R triplet first, else best validated L-R pair ---
    std::vector<int> sorted_idx(clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) sorted_idx[i] = static_cast<int>(i);
    std::sort(sorted_idx.begin(), sorted_idx.end(),
      [&](int a, int b) { return clusters[a].count > clusters[b].count; });

    const int top_k = std::min(triplet_search_top_k_, static_cast<int>(sorted_idx.size()));

    bool use_triplet = false;
    Cluster L{}, C{}, R{};
    double baseline = 0.0;
    double baseline_err = std::numeric_limits<double>::infinity();
    double center_mid_err = std::numeric_limits<double>::infinity();
    int n_points = 0;

    double best3_score = std::numeric_limits<double>::infinity();
    for (int a = 0; a < top_k; ++a) {
      for (int b = a + 1; b < top_k; ++b) {
        for (int c = b + 1; c < top_k; ++c) {
          std::array<int, 3> ids{sorted_idx[a], sorted_idx[b], sorted_idx[c]};
          std::sort(ids.begin(), ids.end(),
            [&](int i, int j) { return clusters[i].cy > clusters[j].cy; });  // left .. right

          const Cluster& l = clusters[ids[0]];
          const Cluster& m = clusters[ids[1]];
          const Cluster& r = clusters[ids[2]];

          const double d_lr = std::hypot(l.cx - r.cx, l.cy - r.cy);
          const double d_lc = std::hypot(l.cx - m.cx, l.cy - m.cy);
          const double d_cr = std::hypot(m.cx - r.cx, m.cy - r.cy);
          const double e_lr = std::fabs(d_lr - baseline_expected_);
          const double e_lc = std::fabs(d_lc - center_spacing_expected_);
          const double e_cr = std::fabs(d_cr - center_spacing_expected_);
          const double mx_lr = 0.5 * (l.cx + r.cx);
          const double my_lr = 0.5 * (l.cy + r.cy);
          const double e_mid = std::hypot(m.cx - mx_lr, m.cy - my_lr);

          if (e_lr > baseline_tol_ || e_lc > center_spacing_tol_ ||
              e_cr > center_spacing_tol_ || e_mid > center_midpoint_tol_) {
            continue;
          }

          const double score =
            (e_lr / std::max(1e-6, baseline_tol_)) +
            0.8 * (e_lc / std::max(1e-6, center_spacing_tol_)) +
            0.8 * (e_cr / std::max(1e-6, center_spacing_tol_)) +
            1.2 * (e_mid / std::max(1e-6, center_midpoint_tol_));

          if (score < best3_score) {
            best3_score = score;
            use_triplet = true;
            L = l;
            C = m;
            R = r;
            baseline = d_lr;
            baseline_err = e_lr;
            center_mid_err = e_mid;
            n_points = l.count + m.count + r.count;
          }
        }
      }
    }

    if (!use_triplet) {
      double best2_score = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < clusters.size(); ++i) {
        for (size_t j = i + 1; j < clusters.size(); ++j) {
          const Cluster& a = clusters[i];
          const Cluster& b = clusters[j];
          const Cluster& l = (a.cy >= b.cy) ? a : b;
          const Cluster& r = (a.cy >= b.cy) ? b : a;
          const double d_lr = std::hypot(l.cx - r.cx, l.cy - r.cy);
          const double e_lr = std::fabs(d_lr - baseline_expected_);
          if (e_lr > baseline_tol_) continue;

          const double score =
            (e_lr / std::max(1e-6, baseline_tol_)) -
            0.03 * static_cast<double>(l.count + r.count);
          if (score < best2_score) {
            best2_score = score;
            L = l;
            R = r;
            baseline = d_lr;
            baseline_err = e_lr;
            center_mid_err = 0.0;
            n_points = l.count + r.count;
          }
        }
      }
      if (!std::isfinite(best2_score)) {
        publish_invalid();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "no valid strip geometry: %zu clusters seen, no triplet or side-pair passed spacing checks.",
          clusters.size());
        return;
      }
    }

    // --- 5) dock centre + outward normal ---
    const double mx = use_triplet ? C.cx : 0.5 * (L.cx + R.cx);
    const double my = use_triplet ? C.cy : 0.5 * (L.cy + R.cy);
    const bool clean = true;

    const double bx = L.cx - R.cx, by = L.cy - R.cy;   // baseline vector R->L (along the dock face)
    const double dirx = bx / baseline, diry = by / baseline;   // face direction (unit)
    double nx = -diry, ny = dirx;                              // perpendicular to the face
    if (nx * (-mx) + ny * (-my) < 0.0) { nx = -nx; ny = -ny; } // sign it back toward the lidar

    // --- 6) build the dock-face pose in the laser frame, TF to target_frame ---
    geometry_msgs::msg::PoseStamped pose_laser;
    pose_laser.header = scan->header;                 // frame = base_laser, stamp = scan time
    pose_laser.pose.position.x = mx;
    pose_laser.pose.position.y = my;
    pose_laser.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(ny, nx));           // yaw = outward-normal heading
    pose_laser.pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseStamped pose_out;
    try {
      tf_buffer_->transform(pose_laser, pose_out, target_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s <- %s unavailable: %s", target_frame_.c_str(),
        scan->header.frame_id.c_str(), ex.what());
      publish_invalid();
      return;
    }

    // Derived scalars in the target (robot) frame.
    const double along   = pose_out.pose.position.x;             // fore/aft
    const double lateral = pose_out.pose.position.y;             // +left / -right
    const double range   = std::hypot(along, lateral);
    const double bearing = std::atan2(lateral, along);           // 0 = straight ahead
    double nyaw;
    { tf2::Quaternion qo; tf2::fromMsg(pose_out.pose.orientation, qo);
      double r_, p_; tf2::Matrix3x3(qo).getRPY(r_, p_, nyaw); }
    // Squared-up when the outward normal points straight back at the robot:
    // normal heading == (bearing_to_dock + pi). skew=0 => perfectly square.
    const double skew = wrap_pi(nyaw - (bearing + M_PI));

    // Confidence score [0..1].
    const double sep_score = clamp01(1.0 - (baseline_err / std::max(1e-6, baseline_tol_)));
    const double points_score = clamp01(
      (static_cast<double>(n_points - confidence_min_points_)) /
      std::max(1.0, static_cast<double>(confidence_full_points_ - confidence_min_points_)));

    double baseline_jump_score = 1.0;
    double bearing_jump_score  = 1.0;
    double range_jump_score    = 1.0;

    if (std::isfinite(last_baseline_)) {
      baseline_jump_score = clamp01(
        1.0 - std::fabs(baseline - last_baseline_) / std::max(1e-6, continuity_baseline_jump_m_));
    }
    if (std::isfinite(last_bearing_)) {
      const double max_jump = continuity_bearing_jump_deg_ * M_PI / 180.0;
      bearing_jump_score = clamp01(
        1.0 - std::fabs(wrap_pi(bearing - last_bearing_)) / std::max(1e-6, max_jump));
    }
    if (std::isfinite(last_range_)) {
      range_jump_score = clamp01(
        1.0 - std::fabs(range - last_range_) / std::max(1e-6, continuity_range_jump_m_));
    }

    const double center_score = use_triplet
      ? clamp01(1.0 - center_mid_err / std::max(1e-6, center_midpoint_tol_))
      : 0.55;

    const double raw_confidence =
      0.40 * sep_score +
      0.20 * points_score +
      0.15 * center_score +
      0.10 * baseline_jump_score +
      0.10 * bearing_jump_score +
      0.05 * range_jump_score;

    filtered_confidence_ =
      confidence_alpha_ * clamp01(raw_confidence) + (1.0 - confidence_alpha_) * filtered_confidence_;
    publish_confidence(static_cast<float>(filtered_confidence_));

    if (clean) {
      pose_pub_->publish(pose_out);
      last_baseline_ = baseline;
      last_bearing_  = bearing;
      last_range_    = range;
    }

    std_msgs::msg::Float32MultiArray dbg;
    dbg.layout.dim.resize(1);
    dbg.layout.dim[0].label  = "valid,along,lateral,range,bearing,skew,baseline,base_err,n_points";
    dbg.layout.dim[0].size   = D_SIZE;
    dbg.layout.dim[0].stride = D_SIZE;
    dbg.data.assign(D_SIZE, 0.0f);
    dbg.data[D_VALID]    = clean ? 1.0f : 0.0f;
    dbg.data[D_ALONG]    = static_cast<float>(along);
    dbg.data[D_LATERAL]  = static_cast<float>(lateral);
    dbg.data[D_RANGE]    = static_cast<float>(range);
    dbg.data[D_BEARING]  = static_cast<float>(bearing);
    dbg.data[D_SKEW]     = static_cast<float>(skew);
    dbg.data[D_BASELINE] = static_cast<float>(baseline);
    dbg.data[D_BASE_ERR] = static_cast<float>(baseline_err);
    dbg.data[D_N_POINTS] = static_cast<float>(n_points);
    debug_pub_->publish(dbg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "dock: range %.3f m | lateral %+.3f m | bearing %+.1f deg | skew %+.1f deg | "
      "baseline %.0f mm (err %+.0f) | %d pts | mode %s%s",
      range, lateral, bearing * 180.0 / M_PI, skew * 180.0 / M_PI,
      baseline * 1000.0, (baseline - baseline_expected_) * 1000.0, n_points,
      use_triplet ? "three-strip" : "two-strip-fallback",
      clean ? "" : "  [baseline reject]");
  }

  std::string scan_topic_, pose_topic_, debug_topic_, confidence_topic_, target_frame_;
  double intensity_min_, range_min_, range_max_, max_gap_deg_;
  double baseline_expected_, baseline_tol_;
  double center_spacing_expected_, center_spacing_tol_, center_midpoint_tol_;
  int triplet_search_top_k_;
  int    min_points_;

  int confidence_min_points_, confidence_full_points_;
  double confidence_alpha_, continuity_baseline_jump_m_, continuity_bearing_jump_deg_, continuity_range_jump_m_;
  double filtered_confidence_{0.0};
  double last_baseline_{std::numeric_limits<double>::quiet_NaN()};
  double last_bearing_{std::numeric_limits<double>::quiet_NaN()};
  double last_range_{std::numeric_limits<double>::quiet_NaN()};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr        pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr       debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                 confidence_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr         scan_sub_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockReflector>());
  rclcpp::shutdown();
  return 0;
}
