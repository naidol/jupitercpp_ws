// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Delays nvblox ESDF slice messages by 250 ms before forwarding to the Nav2
// NvbloxCostmapLayer.
//
// Problem: nvblox stamps its static_map_slice with rclcpp::now() (the publication
// time). cuVSLAM publishes TF at the camera frame timestamp, which is 13–50 ms
// behind ROS clock due to processing latency. In RGBD mode (6 fps depth) the next
// cuVSLAM frame arrives ~167 ms after the previous one. So when NvbloxCostmapLayer
// tries to look up TF at the slice stamp (= now), the latest TF is 13–50 ms behind
// and the lookup throws "extrapolation into future."
//
// Fix: delay the slice by 200 ms. With tracking_mode=1 (VIO), cuVSLAM
// publishes TF at the IMU rate (200 Hz = 5 ms interval). A 200 ms delay
// guarantees ≥40 TF frames have been published after the slice timestamp,
// so NvbloxCostmapLayer's tf2 lookup always interpolates successfully.
// At ≤0.5 m/s the 200 ms latency ≤10 cm of travel — negligible.

#include <chrono>
#include <memory>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <nvblox_msgs/msg/distance_map_slice.hpp>

using DistanceMapSlice = nvblox_msgs::msg::DistanceMapSlice;
using namespace std::chrono_literals;

class NvbloxSliceRelay : public rclcpp::Node
{
public:
  NvbloxSliceRelay()
  : Node("nvblox_slice_relay"),
    delay_(400ms)
  {
    sub_ = create_subscription<DistanceMapSlice>(
      "/nvblox_node/static_map_slice", 5,
      [this](DistanceMapSlice::UniquePtr msg) {
        auto due = now() + delay_;
        queue_.push({due, std::move(msg)});
      });

    pub_ = create_publisher<DistanceMapSlice>(
      "/nvblox_node/static_map_slice_delayed", 5);

    // Drain the queue at 20 Hz — fine-grained enough relative to the 5 Hz slice rate.
    timer_ = create_wall_timer(50ms, [this]() {
      auto t = now();
      while (!queue_.empty() && queue_.front().due <= t) {
        auto msg = std::move(queue_.front().msg);
        // DO NOT re-stamp the message. By keeping the original timestamp from nvblox
        // and delaying the publication, we allow the lagging cuVSLAM VIO data to 
        // arrive in the TF buffer before Nav2 attempts the coordinate lookup.
        pub_->publish(std::move(msg));
        queue_.pop();
      }
    });
  }

private:
  struct Entry {
    rclcpp::Time due;
    DistanceMapSlice::UniquePtr msg;
  };

  const rclcpp::Duration delay_;
  std::queue<Entry> queue_;

  rclcpp::Subscription<DistanceMapSlice>::SharedPtr sub_;
  rclcpp::Publisher<DistanceMapSlice>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NvbloxSliceRelay>());
  rclcpp::shutdown();
  return 0;
}
