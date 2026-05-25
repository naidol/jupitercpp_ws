# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Isaac ROS cuVSLAM — Orbbec Gemini 336 RGBD + IMU
#
# Replaces both AMCL and the EKF localiser for Nav2 navigation mode.
# Publishes the full TF chain: map → odom → base_footprint
#
# Prerequisites — camera must be launched with:
#   enable_depth=true, enable_accel=true, enable_gyro=true,
#   enable_sync_output_accel_gyro=true, align_mode=SW, align_target_stream=COLOR
#
# Camera-mounting static TF (base_footprint → camera_link):
#   Measure your physical camera position and update the arguments below.
#   Defaults: 10 cm forward, centred, 25 cm above ground, no rotation.

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Static TF: base_footprint → camera_link
    # Orbbec 336 physical mounting: 10 cm forward, centred, 49 cm above ground, no tilt.
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x',     '0.10',
            '--y',     '0.0',
            '--z',     '0.49',
            '--roll',  '0',
            '--pitch', '0',
            '--yaw',   '0',
            '--frame-id',       'base_footprint',
            '--child-frame-id', 'camera_link',
        ],
    )

    # cuVSLAM — RGBD + IMU mode using the Orbbec Gemini 336.
    # tracking_mode 2 = RGBD (depth + colour image pair, optional IMU fusion).
    # enable_localization_n_mapping — full visual SLAM:
    #   publishes map→odom TF (replaces AMCL) AND odom→base_footprint TF (replaces EKF).
    # depth_scale_factor 1000 — Orbbec depth is in millimetres; cuVSLAM wants metres.
    # rectified_images false — Orbbec publishes raw (unrectified) colour images.
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'tracking_mode':                     2,
            'depth_scale_factor':                1000.0,
            'enable_image_denoising':            False,
            'rectified_images':                  False,
            'image_jitter_threshold_ms':         100.0,
            'sync_matching_threshold_ms':        40.0,
            'base_frame':                        'base_footprint',
            'map_frame':                         'map',
            'odom_frame':                        'odom',
            'imu_frame':                         'camera_accel_gyro_optical_frame',
            'enable_localization_n_mapping':     True,
            'num_cameras':                       1,
            'depth_camera_id':                   0,
            'min_num_images':                    1,
            'camera_optical_frames':             ['camera_color_optical_frame'],
            'enable_slam_visualization':         True,
            'enable_landmarks_view':             True,
            'enable_observations_view':          True,
            'enable_imu_fusion':                  True,
            'gyro_noise_density':                0.000244,
            'gyro_random_walk':                  0.000019393,
            'accel_noise_density':               0.001862,
            'accel_random_walk':                 0.003,
            'calibration_frequency':             200.0,
            'enable_ground_constraint_in_odometry': True,
            'enable_ground_constraint_in_slam':  True,
        }],
        remappings=[
            ('visual_slam/image_0',       '/camera/color/image_raw'),
            ('visual_slam/camera_info_0', '/camera/color/camera_info'),
            ('visual_slam/depth_0',       '/camera/depth/image_raw'),
            ('visual_slam/imu',           '/camera/gyro_accel/sample'),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([camera_link_tf, visual_slam_container])
