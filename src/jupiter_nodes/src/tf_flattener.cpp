// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

/**
 * @brief TF Flattener Node
 * 
 * Intercepts a 3D transform (e.g., from cuVSLAM) and publishes a 2D version 
 * (zero roll, pitch, and optionally Z) to satisfy Nav2's NvbloxCostmapLayer 
 * requirements and prevent costmap freezes due to minor IMU drift.
 */
class TfFlattener : public rclcpp::Node {
public:
    TfFlattener() : Node("tf_flattener") {
        this->declare_parameter("input_frame", "map_raw");
        this->declare_parameter("output_frame", "map");
        this->declare_parameter("child_frame", "odom");
        this->declare_parameter("flatten_z", true);

        input_frame_ = this->get_parameter("input_frame").as_string();
        output_frame_ = this->get_parameter("output_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();
        flatten_z_ = this->get_parameter("flatten_z").as_bool();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Run at 100Hz to ensure high-fidelity TF bridging for the navigation stack
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TfFlattener::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "TF Flattener online: %s->%s (input) => %s->%s (flat)",
                    input_frame_.c_str(), child_frame_.c_str(),
                    output_frame_.c_str(), child_frame_.c_str());
    }

private:
    void timer_callback() {
        geometry_msgs::msg::TransformStamped ts;
        try {
            ts = tf_buffer_->lookupTransform(input_frame_, child_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return; // Normal during cuVSLAM initialization
        }

        // Convert message to TF2 transform: T(map_raw -> odom)
        tf2::Transform T_raw;
        T_raw.setOrigin(tf2::Vector3(ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z));
        T_raw.setRotation(tf2::Quaternion(ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w));

        tf2::Matrix3x3 m_rot(T_raw.getRotation());
        double roll, pitch, yaw;
        m_rot.getRPY(roll, pitch, yaw);

        // Create the "Ideal" flat version of the transform: T(map_flat -> odom)
        tf2::Transform T_flat;
        T_flat.setOrigin(tf2::Vector3(T_raw.getOrigin().x(), T_raw.getOrigin().y(), flatten_z_ ? 0.0 : T_raw.getOrigin().z()));
        tf2::Quaternion q_flat;
        q_flat.setRPY(0.0, 0.0, yaw);
        T_flat.setRotation(q_flat);

        // Publish map -> map_raw correction at WALL-CLOCK time.
        //
        // TF chain:  map --(this node, wall-clock)--> map_raw --(cuVSLAM, camera-clock)--> odom
        //
        // Why wall-clock here: nvblox slices are stamped at wall-clock; the
        // nvblox_slice_relay delays them 400 ms so that when Nav2 looks up TF at the
        // slice timestamp T, the camera-clock TF buffer already has data past T
        // (camera lag ~250 ms < 400 ms delay → margin of ~150 ms).  Publishing
        // map->map_raw at wall-clock means the first leg of the chain is always
        // available at any wall-clock timestamp Nav2 requests.
        //
        // odom MUST NOT have two parents: cuVSLAM owns map_raw->odom.  Publishing
        // map->odom here instead would give odom two parents and split the TF tree.
        tf2::Transform T_corr = T_flat * T_raw.inverse();

        geometry_msgs::msg::TransformStamped ts_corr;
        ts_corr.header.stamp = this->get_clock()->now();  // WALL CLOCK
        ts_corr.header.frame_id = output_frame_; // "map"
        ts_corr.child_frame_id = input_frame_;   // "map_raw"

        ts_corr.transform.translation.x = T_corr.getOrigin().x();
        ts_corr.transform.translation.y = T_corr.getOrigin().y();
        ts_corr.transform.translation.z = T_corr.getOrigin().z();
        tf2::Quaternion q_corr = T_corr.getRotation();
        ts_corr.transform.rotation.x = q_corr.x();
        ts_corr.transform.rotation.y = q_corr.y();
        ts_corr.transform.rotation.z = q_corr.z();
        ts_corr.transform.rotation.w = q_corr.w();

        tf_broadcaster_->sendTransform(ts_corr);
    }

    std::string input_frame_, output_frame_, child_frame_;
    bool flatten_z_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfFlattener>());
    rclcpp::shutdown();
    return 0;
}