// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// BNO055 NDOF mode — typical covariance values
static constexpr double IMU_ORIENTATION_VARIANCE     = 0.01;   // ~5.7 deg std dev
static constexpr double IMU_ANGULAR_VELOCITY_VARIANCE = 0.001;  // rad²/s²
static constexpr double IMU_LINEAR_ACCEL_VARIANCE     = 0.1;    // m²/s⁴

class ImuCovarianceFixer : public rclcpp::Node {
public:
    explicit ImuCovarianceFixer() : Node("imu_covariance_fixer") {
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data/corrected", 10);

        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ImuCovarianceFixer::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU covariance fixer online");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto out = *msg;

        // Replace -1 (unknown) covariances with calibrated BNO055 NDOF values
        if (msg->orientation_covariance[0] < 0.0) {
            out.orientation_covariance = {
                IMU_ORIENTATION_VARIANCE, 0.0, 0.0,
                0.0, IMU_ORIENTATION_VARIANCE, 0.0,
                0.0, 0.0, IMU_ORIENTATION_VARIANCE
            };
        }

        if (msg->angular_velocity_covariance[0] < 0.0) {
            out.angular_velocity_covariance = {
                IMU_ANGULAR_VELOCITY_VARIANCE, 0.0, 0.0,
                0.0, IMU_ANGULAR_VELOCITY_VARIANCE, 0.0,
                0.0, 0.0, IMU_ANGULAR_VELOCITY_VARIANCE
            };
        }

        if (msg->linear_acceleration_covariance[0] < 0.0) {
            out.linear_acceleration_covariance = {
                IMU_LINEAR_ACCEL_VARIANCE, 0.0, 0.0,
                0.0, IMU_LINEAR_ACCEL_VARIANCE, 0.0,
                0.0, 0.0, IMU_LINEAR_ACCEL_VARIANCE
            };
        }

        pub_->publish(out);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCovarianceFixer>());
    rclcpp::shutdown();
    return 0;
}
