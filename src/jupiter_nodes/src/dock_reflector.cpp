// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_reflector — retro-reflective dock detection from the S2E LiDAR (TWO-STRIP).
//
// WHY TWO STRIPS (2026-08 redesign): a single horizontal strip gives a robust CENTROID
// (position) but a FRAGILE line-fit ANGLE that "balloons" below ~0.35 m — which stranded
// docking (the robot arrived cocked/off-centre and no funnel could rescue it). Two NARROW
// VERTICAL strips a known BASELINE apart fix this: each strip is a tight cluster whose
// centroid is a precise point; the dock CENTRE is their midpoint, and the dock ANGLE is the
// perpendicular to the baseline between them — well-conditioned and, unlike the single-strip
// PCA, it IMPROVES as you close in (more beams per strip). Vertical strips also make the
// lateral read height-invariant (immune to scan-plane pitch/vibration).
//
// STRIPS: two vertical retro strips, ~250 mm centre-to-centre, symmetric about the pogo
// centreline (so their midpoint = the dock centre), spanning the S2E scan-plane height.
//
// METHOD:
//   1. Keep scan points with intensity >= threshold (default 40) inside a range window.
//   2. Segment into angularly-contiguous clusters; keep every cluster with >= min_points.
//   3. Take the TWO largest clusters = the two strips; centroid each.
//   4. VALIDATE: their separation must be ~baseline_expected (rigid, range-independent) or
//      the pair is rejected — a free false-positive gate.
//   5. Dock CENTRE = midpoint of the two centroids. Dock NORMAL = perpendicular to the
//      baseline vector, signed toward the lidar (dock->robot).
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
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
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
}  // namespace

class DockReflector : public rclcpp::Node {
public:
  DockReflector() : Node("dock_reflector") {
    scan_topic_    = declare_parameter<std::string>("scan_topic",  "/scan");
    pose_topic_    = declare_parameter<std::string>("pose_topic",  "/dock/reflector_pose");
    debug_topic_   = declare_parameter<std::string>("debug_topic", "/dock/reflector");
    target_frame_  = declare_parameter<std::string>("target_frame", "base_footprint");

    intensity_min_ = declare_parameter("intensity_min", 40.0);   // strip saturates 63; room <=33
    range_min_     = declare_parameter("range_min",     0.10);
    range_max_     = declare_parameter("range_max",     4.0);
    max_gap_deg_   = declare_parameter("max_gap_deg",   2.0);     // split clusters across a wider angular gap
    min_points_    = declare_parameter("min_points",    4);       // per cluster; narrow strips give few beams at range

    // Two-strip geometry validation: the two centroids must be this far apart (rigid, range-
    // independent) or the pair is rejected as a false detection.
    baseline_expected_ = declare_parameter("baseline_expected", 0.25);   // m, centre-to-centre
    baseline_tol_      = declare_parameter("baseline_tol",      0.05);    // m accept window

    pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(debug_topic_, 10);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DockReflector::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "dock_reflector (TWO-STRIP): %s (I>=%.0f, %.2f-%.2f m) baseline %.0f+/-%.0f mm -> %s + %s in %s.",
      scan_topic_.c_str(), intensity_min_, range_min_, range_max_,
      baseline_expected_ * 1000.0, baseline_tol_ * 1000.0,
      pose_topic_.c_str(), debug_topic_.c_str(), target_frame_.c_str());
  }

private:
  void publish_invalid() {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label  = "valid,along,lateral,range,bearing,skew,baseline,base_err,n_points";
    msg.layout.dim[0].size   = D_SIZE;
    msg.layout.dim[0].stride = D_SIZE;
    msg.data.assign(D_SIZE, 0.0f);
    debug_pub_->publish(msg);
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

    // --- 3) the two largest clusters = the two strips ---
    int i0 = 0;
    for (size_t i = 1; i < clusters.size(); ++i)
      if (clusters[i].count > clusters[i0].count) i0 = static_cast<int>(i);
    int i1 = -1;
    for (size_t i = 0; i < clusters.size(); ++i) {
      if (static_cast<int>(i) == i0) continue;
      if (i1 < 0 || clusters[i].count > clusters[i1].count) i1 = static_cast<int>(i);
    }
    // Order left (higher y = +left) vs right so the baseline vector has a consistent sign.
    const Cluster& A = clusters[i0];
    const Cluster& B = clusters[i1];
    const Cluster& L = (A.cy >= B.cy) ? A : B;
    const Cluster& R = (A.cy >= B.cy) ? B : A;

    // --- 4) validate the baseline separation (rigid physical distance, range-independent) ---
    const double bx = L.cx - R.cx, by = L.cy - R.cy;   // baseline vector R->L (along the dock face)
    const double baseline = std::hypot(bx, by);
    const double baseline_err = std::fabs(baseline - baseline_expected_);
    const bool clean = (baseline_err <= baseline_tol_);

    // --- 5) dock centre = midpoint; outward normal = perpendicular to baseline, toward robot ---
    const double mx = 0.5 * (L.cx + R.cx);
    const double my = 0.5 * (L.cy + R.cy);
    const double dirx = bx / baseline, diry = by / baseline;   // face direction (unit)
    double nx = -diry, ny = dirx;                              // perpendicular to the face
    if (nx * (-mx) + ny * (-my) < 0.0) { nx = -nx; ny = -ny; } // sign it back toward the lidar
    const int n_points = A.count + B.count;

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

    if (clean) pose_pub_->publish(pose_out);

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
      "baseline %.0f mm (err %+.0f) | %d pts%s",
      range, lateral, bearing * 180.0 / M_PI, skew * 180.0 / M_PI,
      baseline * 1000.0, (baseline - baseline_expected_) * 1000.0, n_points,
      clean ? "" : "  [baseline reject]");
  }

  std::string scan_topic_, pose_topic_, debug_topic_, target_frame_;
  double intensity_min_, range_min_, range_max_, max_gap_deg_;
  double baseline_expected_, baseline_tol_;
  int    min_points_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr        pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr       debug_pub_;
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
