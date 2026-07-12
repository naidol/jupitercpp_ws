// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_range — LiDAR wall-ranging for IR docking.
//
// The IR beacon gives SIDE (left/right of centre) but no distance; the robot was
// "flying blind" on range. This node fills that gap: it ranges the flat WALL behind
// the dock and reports (a) how far to drive and (b) how square we are — the two
// things a 2-D LiDAR CAN measure against a featureless wall. (It CANNOT measure
// lateral offset along the wall — that stays the IR beam's job. See dock_ir.cpp.)
//
// Method: take the rear-facing sector of the scan, convert to (x, y) in the laser
// frame, and least-squares fit a line  x = m*y + c  (x ~ constant for a wall roughly
// perpendicular to the approach — well-conditioned near square, which is where we
// dock). Then:
//     wall_dist  = c                     perpendicular distance straight back (m)
//     wall_angle = atan(m)               0 = square-on; sign found empirically (rad)
//     fit_rms    = RMS of x about line   flatness check — reject clutter/doorways
//     dist_to_pogo = wall_dist - dock_depth
//
// REAR BEARING (subtle — see navigation_s2e.launch.py):
//   S2E  /scan      frame base_laser  yaw=pi -> robot REAR is at scan-bearing   0 deg
//   LD20 /scan_low  frame ld20_laser  yaw=0  -> robot REAR is at scan-bearing 180 deg
//   Default rear_bearing_deg=0.0 is correct for the S2E /scan (primary).
//
// Publishes std_msgs/Float32MultiArray on /dock/range (layout labelled, so
// `ros2 topic echo /dock/range` is self-describing). Index convention:
//   [0] valid (1/0)  [1] dist_to_pogo m  [2] wall_dist m  [3] wall_angle rad
//   [4] fit_rms m     [5] n_points

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace {
// /dock/range index convention — keep in sync with dock_ir.cpp.
constexpr int R_VALID        = 0;
constexpr int R_DIST_TO_POGO = 1;
constexpr int R_WALL_DIST    = 2;
constexpr int R_WALL_ANGLE   = 3;
constexpr int R_FIT_RMS      = 4;
constexpr int R_N_POINTS     = 5;
constexpr int R_SIZE         = 6;

double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

class DockRange : public rclcpp::Node {
public:
  DockRange() : Node("dock_range") {
    scan_topic_        = declare_parameter<std::string>("scan_topic",     "/scan");
    range_topic_       = declare_parameter<std::string>("range_topic",    "/dock/range");
    rear_bearing_deg_  = declare_parameter("rear_bearing_deg",   0.0);   // S2E rear=0; LD20 /scan_low=180
    sector_half_deg_   = declare_parameter("sector_half_deg",    10.0);  // rear sector half-width (±10° validated on the 0.8 m box: fit ±2 mm; ±20° overshot the box edges → ±52 mm)
    range_min_         = declare_parameter("range_min",          0.05);  // m — ignore closer returns
    range_max_         = declare_parameter("range_max",          4.0);   // m — ignore far junk
    dock_depth_        = declare_parameter("dock_depth",         0.0);   // m — wall -> pogo plane (MEASURE)
    max_fit_rms_       = declare_parameter("max_fit_rms",        0.03);  // m — above this = not a clean wall
    min_points_        = declare_parameter("min_points",         8);     // fewer than this = invalid

    center_ = rear_bearing_deg_ * M_PI / 180.0;
    half_   = sector_half_deg_  * M_PI / 180.0;

    range_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(range_topic_, 10);
    scan_sub_  = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DockRange::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "dock_range: %s rear sector %.0f+/-%.0f deg -> %s (dock_depth %.3f m).",
      scan_topic_.c_str(), rear_bearing_deg_, sector_half_deg_, range_topic_.c_str(), dock_depth_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Gather rear-sector points as (x, y) in the laser frame.
    std::vector<double> xs, ys;
    xs.reserve(64);
    ys.reserve(64);
    double ang = scan->angle_min;
    for (const float r : scan->ranges) {
      if (std::isfinite(r) && r > range_min_ && r < range_max_ &&
          std::fabs(wrap_pi(ang - center_)) <= half_) {
        xs.push_back(r * std::cos(ang));
        ys.push_back(r * std::sin(ang));
      }
      ang += scan->angle_increment;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label  = "valid,dist_to_pogo,wall_dist,wall_angle,fit_rms,n_points";
    msg.layout.dim[0].size   = R_SIZE;
    msg.layout.dim[0].stride = R_SIZE;
    msg.data.assign(R_SIZE, 0.0f);

    const int n = static_cast<int>(xs.size());
    msg.data[R_N_POINTS] = static_cast<float>(n);
    if (n < min_points_) {
      range_pub_->publish(msg);   // valid stays 0
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rear sector: only %d pts — LiDAR blind here / out of range?", n);
      return;
    }

    // Least-squares line  x = m*y + c  (minimise residual in x; well-conditioned near square).
    double sy = 0.0, sx = 0.0;
    for (int i = 0; i < n; ++i) { sy += ys[i]; sx += xs[i]; }
    const double my = sy / n, mx = sx / n;
    double syy = 0.0, sxy = 0.0;
    for (int i = 0; i < n; ++i) {
      const double dy = ys[i] - my;
      syy += dy * dy;
      sxy += (xs[i] - mx) * dy;
    }
    const double m = (syy > 1e-9) ? (sxy / syy) : 0.0;
    const double c = mx - m * my;                       // x at y=0 = straight-back distance
    double ss = 0.0;
    for (int i = 0; i < n; ++i) {
      const double e = xs[i] - (m * ys[i] + c);
      ss += e * e;
    }
    const double fit_rms = std::sqrt(ss / n);

    const bool valid = (fit_rms <= max_fit_rms_) && (c > 0.0);
    msg.data[R_VALID]        = valid ? 1.0f : 0.0f;
    msg.data[R_WALL_DIST]    = static_cast<float>(c);
    msg.data[R_DIST_TO_POGO] = static_cast<float>(c - dock_depth_);
    msg.data[R_WALL_ANGLE]   = static_cast<float>(std::atan(m));
    msg.data[R_FIT_RMS]      = static_cast<float>(fit_rms);
    range_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "wall %.3f m | to-pogo %.3f m | angle %+.1f deg | fit +/-%.0f mm | %d pts%s",
      c, c - dock_depth_, std::atan(m) * 180.0 / M_PI, fit_rms * 1000.0, n,
      valid ? "" : "  [INVALID: sector not a clean wall]");
  }

  std::string scan_topic_, range_topic_;
  double rear_bearing_deg_, sector_half_deg_, range_min_, range_max_, dock_depth_, max_fit_rms_;
  int    min_points_;
  double center_, half_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr range_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockRange>());
  rclcpp::shutdown();
  return 0;
}
