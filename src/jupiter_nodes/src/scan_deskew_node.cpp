// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// scan_deskew_node — motion-compensates ("deskews") a 2D LaserScan.
//
// WHY THIS EXISTS (measured 2026-06-07):
//   The LD20 builds one 360 deg sweep over scan_time ~= 166 ms (6 Hz), but slam_toolbox
//   stamps all ~667 rays at a single instant. During a turn the robot rotates ~5 deg across
//   that one sweep (at 30 deg/s), so a straight wall is recorded as a ~5 deg arc — a ~0.44 m
//   smear at 5 m. That intra-scan distortion is the confirmed cause of the rotational "haze"
//   (transport latency was measured at ~0.1 ms and ruled out; the CP2102 link is clean).
//   The driver DOES populate time_increment (~249 us/ray), so we can recover each ray's exact
//   firing time and un-rotate/-translate it back to a single reference instant before SLAM.
//
// WHAT IT DOES:
//   For each ray i fired at time t_i = (ref + offset_i), convert (range_i, bearing_i) to a 2D
//   point in the laser frame, apply the rigid-body motion the robot underwent between t_i and a
//   common reference time (constant body twist vx,vy,omega from the fused EKF odometry over the
//   short sweep), then re-bin the corrected point back into the fixed LaserScan angle grid.
//
// Output is a normal LaserScan (so slam_toolbox/AMCL consume it unchanged) with scan_time and
// time_increment zeroed (it now represents a single instant). Point this node's output topic at
// slam_toolbox's scan_topic.
//
// PARAMETERS (all overridable, no hardcoded behaviour):
//   input_scan_topic   (string, "/scan")            raw scan in
//   output_scan_topic  (string, "/scan/deskewed")   corrected scan out
//   odom_topic         (string, "/odometry/filtered") fused twist source (vx,vy,omega)
//   stamp_position     (string, "end")               where header.stamp sits in the sweep:
//                                                     "start" | "mid" | "end". Our LD driver
//                                                     stamps at software-publish (~end), but this
//                                                     is A/B-testable if the map worsens.
//   enable_deskew      (bool,   true)                false = passthrough (same plumbing, for A/B)
//   max_twist_age      (double, 0.2 s)               ignore odom older than this (else passthrough)

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace
{
constexpr double kOmegaEps = 1e-5;  // rad/s below which we treat rotation as zero
}

class ScanDeskewNode : public rclcpp::Node
{
public:
  ScanDeskewNode()
  : rclcpp::Node("scan_deskew_node")
  {
    input_scan_topic_  = declare_parameter<std::string>("input_scan_topic", "/scan");
    output_scan_topic_ = declare_parameter<std::string>("output_scan_topic", "/scan/deskewed");
    odom_topic_        = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    stamp_position_    = declare_parameter<std::string>("stamp_position", "end");
    enable_deskew_     = declare_parameter<bool>("enable_deskew", true);
    max_twist_age_     = declare_parameter<double>("max_twist_age", 0.2);

    // Sensor data QoS: best-effort, depth 5 — matches typical LiDAR driver publishers.
    auto scan_qos = rclcpp::SensorDataQoS();

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_scan_topic_, scan_qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_vx_    = msg->twist.twist.linear.x;
        last_vy_    = msg->twist.twist.linear.y;
        last_omega_ = msg->twist.twist.angular.z;
        last_twist_stamp_ = rclcpp::Time(msg->header.stamp);
        have_twist_ = true;
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_scan_topic_, scan_qos,
      std::bind(&ScanDeskewNode::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "scan_deskew: '%s' -> '%s' | twist from '%s' | stamp_position=%s | deskew=%s",
      input_scan_topic_.c_str(), output_scan_topic_.c_str(), odom_topic_.c_str(),
      stamp_position_.c_str(), enable_deskew_ ? "ON" : "OFF (passthrough)");
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr in)
  {
    const size_t n = in->ranges.size();

    // Passthrough conditions: disabled, no usable per-ray timing, or stale/absent twist.
    const bool have_fresh_twist =
      have_twist_ &&
      (rclcpp::Time(in->header.stamp) - last_twist_stamp_).seconds() < max_twist_age_;

    if (!enable_deskew_ || in->time_increment == 0.0f || n < 2 || !have_fresh_twist) {
      if (enable_deskew_ && in->time_increment == 0.0f) {
        RCLCPP_WARN_ONCE(get_logger(),
          "time_increment is 0 -> cannot deskew (driver gives no per-ray timing); passing through");
      }
      pub_->publish(*in);
      return;
    }

    const double dt   = in->time_increment;          // seconds between consecutive rays
    const double span = dt * static_cast<double>(n - 1);  // first->last ray duration

    // Time offset of ray 0 relative to the reference instant (header.stamp), per convention.
    // ray i fires at t_i = ref + (off0 + i*dt).
    double off0;
    if (stamp_position_ == "start")      off0 = 0.0;
    else if (stamp_position_ == "mid")   off0 = -span * 0.5;
    else                                 off0 = -span;   // "end" (default)

    const double vx = last_vx_, vy = last_vy_, w = last_omega_;

    // Build the output scan: identical grid, ranges reset to "no return" (inf), then re-binned.
    auto out = std::make_shared<sensor_msgs::msg::LaserScan>(*in);
    const float no_ret = std::numeric_limits<float>::infinity();
    std::fill(out->ranges.begin(), out->ranges.end(), no_ret);
    const bool has_int = (in->intensities.size() == n);
    if (has_int) std::fill(out->intensities.begin(), out->intensities.end(), 0.0f);
    out->time_increment = 0.0f;   // corrected scan represents a single instant
    out->scan_time = 0.0f;

    for (size_t i = 0; i < n; ++i) {
      const float r = in->ranges[i];
      if (!std::isfinite(r) || r < in->range_min || r > in->range_max) continue;

      const double bearing = in->angle_min + static_cast<double>(i) * in->angle_increment;
      const double px = r * std::cos(bearing);   // point in laser frame at ray i's capture time
      const double py = r * std::sin(bearing);

      // tau = time to advance ray i forward to the reference instant.
      const double tau = -(off0 + static_cast<double>(i) * dt);  // ref - t_i

      // Body displacement over tau under constant twist (SE(2) exp map). This is the pose of
      // base@ref expressed in base@t_i; the world-fixed point is then moved by its inverse.
      const double dtheta = w * tau;
      double tx, ty;
      if (std::fabs(w) > kOmegaEps) {
        const double s = std::sin(dtheta), c = std::cos(dtheta);
        tx = (vx * s + vy * (c - 1.0)) / w;
        ty = (vx * (1.0 - c) + vy * s) / w;
      } else {
        tx = vx * tau;
        ty = vy * tau;
      }

      // p_ref = T_{t_i->ref}^{-1} * p_i  =  R(-dtheta) * (p_i - t)
      const double dx = px - tx, dy = py - ty;
      const double cn = std::cos(-dtheta), sn = std::sin(-dtheta);
      const double rx = cn * dx - sn * dy;
      const double ry = sn * dx + cn * dy;

      const double r_corr = std::hypot(rx, ry);
      const double b_corr = std::atan2(ry, rx);

      // Re-bin into the fixed output grid (nearest angle cell).
      const long idx = std::lround((b_corr - in->angle_min) / in->angle_increment);
      if (idx < 0 || idx >= static_cast<long>(n)) continue;
      auto & slot = out->ranges[static_cast<size_t>(idx)];
      if (static_cast<float>(r_corr) < slot) {     // keep nearest on collision
        slot = static_cast<float>(r_corr);
        if (has_int) out->intensities[static_cast<size_t>(idx)] = in->intensities[i];
      }
    }

    pub_->publish(*out);
  }

  // params
  std::string input_scan_topic_, output_scan_topic_, odom_topic_, stamp_position_;
  bool enable_deskew_{true};
  double max_twist_age_{0.2};

  // latest fused twist
  double last_vx_{0.0}, last_vy_{0.0}, last_omega_{0.0};
  rclcpp::Time last_twist_stamp_{0, 0, RCL_ROS_TIME};
  bool have_twist_{false};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
