// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_reflector — retro-reflective dock detection from the S2E LiDAR.
//
// WHY THIS EXISTS (2026-07 pivot away from AprilTag): the robot must be placeable
// ANYWHERE, then find its dock, square itself to it, and reverse in — Dreame-style.
// A single AprilTag gave good (x,z) but noisy yaw, so the robot arrived angled. The
// dock now carries a retro-reflective strip: on the 6-bit S2E intensity scale (0-63)
// it SATURATES at 63 while the whole rest of the room stays <=33 (bench-confirmed
// 2026-07-24), so a simple intensity threshold isolates it cleanly and survives a
// MOVED dock — the dock is re-found by its own return every cycle, never assumed.
//
// dock_range.cpp fits the flat WALL and by its own admission CANNOT measure lateral
// offset (a featureless wall has no landmark along it). The strip is that landmark:
// its centroid gives LATERAL, and a line-fit through its bright points gives the dock
// FACE ANGLE (the strip is seen sloped in range across its width -> orientation).
//
// METHOD:
//   1. Keep scan points with intensity >= threshold (default 40) inside a range window.
//   2. Segment them into angularly-contiguous clusters; take the largest (rejects a
//      stray specular glint). This is the strip.
//   3. Convert to (x,y) in the laser frame; PCA (2x2 closed form) -> centroid, strip
//      long-axis, perpendicular RMS (flatness), and the OUTWARD NORMAL (dock->robot).
//   4. TF the centroid+normal pose from base_laser into base_footprint so the yaw=pi
//      S2E mount is handled by the tree, not hardcoded.
//
// PUBLISHES:
//   geometry_msgs/PoseStamped  /dock/reflector_pose   dock face in base_footprint:
//        position = strip centroid; orientation yaw = outward normal (points at robot).
//        The aligner squares up when the robot->dock line is anti-parallel to this.
//   std_msgs/Float32MultiArray /dock/reflector        self-describing debug (echo-able):
//        [0] valid(1/0)  [1] along_x m (fore/aft)  [2] lateral_y m (+left)
//        [3] range m     [4] bearing rad (0=front,+left)  [5] skew rad (0=squared)
//        [6] strip_len m [7] fit_rms m  [8] n_points
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
constexpr int D_VALID     = 0;
constexpr int D_ALONG     = 1;
constexpr int D_LATERAL   = 2;
constexpr int D_RANGE     = 3;
constexpr int D_BEARING   = 4;
constexpr int D_SKEW      = 5;
constexpr int D_STRIP_LEN = 6;
constexpr int D_FIT_RMS   = 7;
constexpr int D_N_POINTS  = 8;
constexpr int D_SIZE      = 9;

double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

class DockReflector : public rclcpp::Node {
public:
  DockReflector() : Node("dock_reflector") {
    scan_topic_    = declare_parameter<std::string>("scan_topic",  "/scan");
    pose_topic_    = declare_parameter<std::string>("pose_topic",  "/dock/reflector_pose");
    debug_topic_   = declare_parameter<std::string>("debug_topic", "/dock/reflector");
    target_frame_  = declare_parameter<std::string>("target_frame", "base_footprint");

    intensity_min_ = declare_parameter("intensity_min", 40.0);   // strip saturates 63; room <=33 -> 40 is safely between
    range_min_     = declare_parameter("range_min",     0.10);   // m — ignore self/near junk
    range_max_     = declare_parameter("range_max",     4.0);    // m — a pre-dock is ~1 m; reject far glints
    max_gap_deg_   = declare_parameter("max_gap_deg",   2.0);    // split clusters across an angular gap wider than this
    min_points_    = declare_parameter("min_points",    6);      // fewer than this = not the strip
    max_fit_rms_   = declare_parameter("max_fit_rms",   0.03);   // m — perpendicular RMS above this = not a clean strip

    pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(debug_topic_, 10);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DockReflector::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "dock_reflector: %s (I>=%.0f, %.2f-%.2f m) -> %s + %s in frame %s.",
      scan_topic_.c_str(), intensity_min_, range_min_, range_max_,
      pose_topic_.c_str(), debug_topic_.c_str(), target_frame_.c_str());
  }

private:
  void publish_invalid() {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label  = "valid,along,lateral,range,bearing,skew,strip_len,fit_rms,n_points";
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
    if (static_cast<int>(bright.size()) < min_points_) {
      publish_invalid();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "no reflector: only %zu bright points (need %d).", bright.size(), min_points_);
      return;
    }

    // --- 2) segment into angularly-contiguous clusters; keep the largest ---
    const int gap_bins = std::max(1, static_cast<int>(
        std::ceil((max_gap_deg_ * M_PI / 180.0) / scan->angle_increment)));
    size_t best_lo = 0, best_hi = 0, run_lo = 0;   // [lo, hi) half-open
    for (size_t k = 1; k <= bright.size(); ++k) {
      const bool split = (k == bright.size()) ||
                         (bright[k].idx - bright[k - 1].idx > gap_bins);
      if (split) {
        if ((k - run_lo) > (best_hi - best_lo)) { best_lo = run_lo; best_hi = k; }
        run_lo = k;
      }
    }
    const int n = static_cast<int>(best_hi - best_lo);
    if (n < min_points_) {
      publish_invalid();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "reflector cluster too small: %d points (need %d).", n, min_points_);
      return;
    }

    // --- 3) PCA on the cluster (2x2 closed form) ---
    double mx = 0.0, my = 0.0;
    for (size_t k = best_lo; k < best_hi; ++k) { mx += bright[k].x; my += bright[k].y; }
    mx /= n; my /= n;
    double Sxx = 0.0, Sxy = 0.0, Syy = 0.0;
    for (size_t k = best_lo; k < best_hi; ++k) {
      const double dx = bright[k].x - mx, dy = bright[k].y - my;
      Sxx += dx * dx; Sxy += dx * dy; Syy += dy * dy;
    }
    Sxx /= n; Sxy /= n; Syy /= n;
    const double tr = Sxx + Syy;
    const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - (Sxx * Syy - Sxy * Sxy)));
    const double l_max = tr / 2.0 + disc;   // variance along the strip
    const double l_min = tr / 2.0 - disc;   // variance across it (flatness)

    // Long-axis eigenvector (for l_max); guard the Sxy~0 degenerate case.
    double dirx, diry;
    if (std::fabs(Sxy) > 1e-12) { dirx = l_max - Syy; diry = Sxy; }
    else if (Sxx >= Syy)        { dirx = 1.0; diry = 0.0; }
    else                        { dirx = 0.0; diry = 1.0; }
    const double dn = std::hypot(dirx, diry);
    dirx /= dn; diry /= dn;

    // Outward normal = perpendicular to long axis, signed toward the lidar (origin),
    // i.e. pointing from the dock face back at the robot (the approach reference).
    double nx = -diry, ny = dirx;
    if (nx * (-mx) + ny * (-my) < 0.0) { nx = -nx; ny = -ny; }

    // Strip length = extent of the cluster projected onto the long axis.
    double pmin = 1e9, pmax = -1e9;
    for (size_t k = best_lo; k < best_hi; ++k) {
      const double p = (bright[k].x - mx) * dirx + (bright[k].y - my) * diry;
      pmin = std::min(pmin, p); pmax = std::max(pmax, p);
    }
    const double strip_len = pmax - pmin;
    const double fit_rms   = std::sqrt(std::max(0.0, l_min));

    // --- 4) build the dock-face pose in the laser frame, TF to target_frame ---
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

    const bool clean = (fit_rms <= max_fit_rms_);

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

    pose_pub_->publish(pose_out);

    std_msgs::msg::Float32MultiArray dbg;
    dbg.layout.dim.resize(1);
    dbg.layout.dim[0].label  = "valid,along,lateral,range,bearing,skew,strip_len,fit_rms,n_points";
    dbg.layout.dim[0].size   = D_SIZE;
    dbg.layout.dim[0].stride = D_SIZE;
    dbg.data.assign(D_SIZE, 0.0f);
    dbg.data[D_VALID]     = clean ? 1.0f : 0.0f;
    dbg.data[D_ALONG]     = static_cast<float>(along);
    dbg.data[D_LATERAL]   = static_cast<float>(lateral);
    dbg.data[D_RANGE]     = static_cast<float>(range);
    dbg.data[D_BEARING]   = static_cast<float>(bearing);
    dbg.data[D_SKEW]      = static_cast<float>(skew);
    dbg.data[D_STRIP_LEN] = static_cast<float>(strip_len);
    dbg.data[D_FIT_RMS]   = static_cast<float>(fit_rms);
    dbg.data[D_N_POINTS]  = static_cast<float>(n);
    debug_pub_->publish(dbg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "dock: range %.3f m | lateral %+.3f m | bearing %+.1f deg | skew %+.1f deg | "
      "len %.0f mm | fit +/-%.0f mm | %d pts%s",
      range, lateral, bearing * 180.0 / M_PI, skew * 180.0 / M_PI,
      strip_len * 1000.0, fit_rms * 1000.0, n,
      clean ? "" : "  [noisy fit]");
  }

  std::string scan_topic_, pose_topic_, debug_topic_, target_frame_;
  double intensity_min_, range_min_, range_max_, max_gap_deg_, max_fit_rms_;
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
