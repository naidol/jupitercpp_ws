// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter Planner — Phase 1
//
// Subscribes:  /jupiter/goal        (geometry_msgs/PoseStamped)  — where to go
//              /odometry/filtered   (nav_msgs/Odometry)          — where we are
// Publishes:   /jupiter/path        (nav_msgs/Path)              — A* waypoints
//              TF: map → odom       (static offset set at startup)
//
// Dead-reckoning only in Phase 1.  ICP localizer (Phase 3) will correct drift
// by updating the map→odom transform dynamically.

#include "jupiter_nav/astar.hpp"
#include "jupiter_nav/occupancy_map.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>

class JupiterPlannerNode : public rclcpp::Node {
public:
    JupiterPlannerNode() : Node("jupiter_planner") {
        declare_parameter("map_yaml",         "");
        declare_parameter("inflation_radius", 0.32);
        declare_parameter("initial_map_x",    0.0);
        declare_parameter("initial_map_y",    0.0);
        declare_parameter("initial_map_yaw",  0.0);

        const std::string map_yaml = get_parameter("map_yaml").as_string();
        const double inflation_r   = get_parameter("inflation_radius").as_double();
        initial_x_   = get_parameter("initial_map_x").as_double();
        initial_y_   = get_parameter("initial_map_y").as_double();
        initial_yaw_ = get_parameter("initial_map_yaw").as_double();

        if (map_yaml.empty()) {
            RCLCPP_FATAL(get_logger(), "map_yaml parameter not set");
            rclcpp::shutdown();
            return;
        }
        if (!map_.load(map_yaml)) {
            RCLCPP_FATAL(get_logger(), "Failed to load map: %s", map_yaml.c_str());
            rclcpp::shutdown();
            return;
        }
        map_.inflate(inflation_r);
        RCLCPP_INFO(get_logger(), "Map loaded: %dx%d cells @ %.3fm, inflated %.2fm",
                    map_.width(), map_.height(), map_.resolution(), inflation_r);

        // Publish static map→odom transform.
        // At launch the robot is at (initial_x_, initial_y_, initial_yaw_) in the
        // map frame, while odom reports (0,0,0).  The map→odom TF is exactly that
        // initial pose.  Phase 3 (ICP) will update this dynamically to correct drift.
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_map_to_odom_tf();

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/jupiter/goal", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_goal(msg); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/jupiter/path", 10);

        // Publish map as OccupancyGrid with transient-local QoS so RViz2 receives
        // it even when subscribing after the map was first published.
        rclcpp::QoS map_qos(1);
        map_qos.transient_local();
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        publish_map_msg();

        RCLCPP_INFO(get_logger(),
                    "Planner ready. Robot starts at map (%.2f, %.2f, %.2f°). "
                    "Send a goal to /jupiter/goal",
                    initial_x_, initial_y_, initial_yaw_ * 180.0 / M_PI);
    }

private:
    // ── Odometry callback ─────────────────────────────────────────────────────
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!odom_ready_) {
            odom_start_x_ = msg->pose.pose.position.x;
            odom_start_y_ = msg->pose.pose.position.y;
            odom_ready_   = true;
        }
        // Robot position in map frame = initial pose + rotated odom displacement
        const double dx = msg->pose.pose.position.x - odom_start_x_;
        const double dy = msg->pose.pose.position.y - odom_start_y_;
        const double c  = std::cos(initial_yaw_);
        const double s  = std::sin(initial_yaw_);
        robot_x_ = initial_x_ + c * dx - s * dy;
        robot_y_ = initial_y_ + s * dx + c * dy;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        robot_yaw_ = yaw + initial_yaw_;
    }

    // ── Goal callback ─────────────────────────────────────────────────────────
    void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!odom_ready_) {
            RCLCPP_WARN(get_logger(), "No odometry yet — ignoring goal");
            return;
        }

        const double goal_x = msg->pose.position.x;
        const double goal_y = msg->pose.position.y;

        const Cell start = map_.world_to_cell(robot_x_, robot_y_);
        const Cell goal  = map_.world_to_cell(goal_x,   goal_y);

        RCLCPP_INFO(get_logger(),
                    "Planning: (%.2f, %.2f) → (%.2f, %.2f)",
                    robot_x_, robot_y_, goal_x, goal_y);

        if (!map_.is_free(start.row, start.col))
            RCLCPP_WARN(get_logger(), "Start cell (%d,%d) is inside an obstacle — "
                        "check initial pose or inflate radius", start.row, start.col);

        if (!map_.is_free(goal.row, goal.col)) {
            RCLCPP_ERROR(get_logger(), "Goal cell (%d,%d) is inside an obstacle",
                         goal.row, goal.col);
            return;
        }

        AStar planner;
        auto result = planner.search(start, goal, map_);
        if (!result.found) {
            RCLCPP_ERROR(get_logger(), "No path found to goal (%.2f, %.2f)", goal_x, goal_y);
            return;
        }

        // String-pull to replace staircase grid path with straight segments
        auto smoothed = AStar::smooth(result.path, map_);
        RCLCPP_INFO(get_logger(), "Path: %zu raw → %zu smoothed waypoints",
                    result.path.size(), smoothed.size());

        publish_path(smoothed);
    }

    // ── Path publisher ────────────────────────────────────────────────────────
    void publish_path(const std::vector<Cell>& cells) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp    = now();
        path_msg.header.frame_id = "map";

        for (const auto& cell : cells) {
            double wx, wy;
            map_.cell_to_world(cell.row, cell.col, wx, wy);
            geometry_msgs::msg::PoseStamped pose;
            pose.header          = path_msg.header;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        // Point each waypoint's orientation toward the next — cosmetic for RViz arrows
        for (size_t i = 0; i + 1 < path_msg.poses.size(); ++i) {
            const double dx = path_msg.poses[i+1].pose.position.x
                            - path_msg.poses[i].pose.position.x;
            const double dy = path_msg.poses[i+1].pose.position.y
                            - path_msg.poses[i].pose.position.y;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, std::atan2(dy, dx));
            path_msg.poses[i].pose.orientation = tf2::toMsg(q);
        }
        if (path_msg.poses.size() > 1)
            path_msg.poses.back().pose.orientation =
                path_msg.poses[path_msg.poses.size() - 2].pose.orientation;

        path_pub_->publish(path_msg);
    }

    // ── Map publisher ─────────────────────────────────────────────────────────
    void publish_map_msg() {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp            = now();
        grid.header.frame_id         = "map";
        grid.info.resolution         = static_cast<float>(map_.resolution());
        grid.info.width              = map_.width();
        grid.info.height             = map_.height();
        grid.info.origin.position.x  = map_.origin_x();
        grid.info.origin.position.y  = map_.origin_y();
        grid.info.origin.orientation.w = 1.0;

        const auto& free = map_.free_cells();
        grid.data.resize(free.size());
        for (size_t i = 0; i < free.size(); ++i)
            grid.data[i] = free[i] ? 0 : 100;

        map_pub_->publish(grid);
        RCLCPP_INFO(get_logger(), "Map published to /map (%dx%d, transient-local)",
                    map_.width(), map_.height());
    }

    // ── TF broadcaster ────────────────────────────────────────────────────────
    void publish_map_to_odom_tf() {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp            = now();
        tf.header.frame_id         = "map";
        tf.child_frame_id          = "odom";
        tf.transform.translation.x = initial_x_;
        tf.transform.translation.y = initial_y_;
        tf.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, initial_yaw_);
        tf.transform.rotation = tf2::toMsg(q);
        static_tf_broadcaster_->sendTransform(tf);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    OccupancyMap map_;

    double initial_x_{0.0}, initial_y_{0.0}, initial_yaw_{0.0};

    bool   odom_ready_{false};
    double odom_start_x_{0.0}, odom_start_y_{0.0};
    double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr         map_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>               static_tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JupiterPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
