# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Isaac ROS cuVSLAM — Orbbec Gemini 336 RGBD + IMU
#
# Replaces both AMCL and the EKF localiser for Nav2 navigation mode.
# Publishes the full TF chain: map → odom → base_footprint
#
# Prerequisites — camera must be launched with:
#   enable_left_ir=true, enable_right_ir=true, enable_laser=false (projector OFF),
#   enable_accel=true, enable_gyro=true, enable_sync_output_accel_gyro=true
#   NOTE: projector must be OFF — its pattern moves with the camera and causes cuVSLAM
#   to track "moving features", producing tracking drift on every robot movement.
#
# Camera-mounting static TF (base_footprint → camera_link):
#   Measure your physical camera position and update the arguments below.
#   Defaults: 10 cm forward, centred, 25 cm above ground, no rotation.

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Static TF: base_footprint → camera_link
    # Orbbec 336 physical mounting: 10 cm forward, centred, 49 cm above ground, level.
    # Projector is off — camera should face level to maximise wall/furniture features
    # in FOV. Upward tilt would show more featureless ceiling and reduce tracking.
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

    # cuVSLAM — nav mode: stereo IR + IMU.
    # tracking_mode 1 (VIO) with 2 IR cameras gives stereo depth and the IR structured-
    # light dot pattern provides trackable features on white walls and reflective floors
    # where colour fails. Depth stream kept active solely to keep the IR laser projector
    # on — cuVSLAM does not consume depth data.
    # Publishes map→odom TF (global localisation); EKF owns odom→base_footprint.
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # tracking_mode 1 = VIO: stereo IR + IMU fusion.
            # IMU at 200 Hz keeps map→odom gravity-aligned between camera frames.
            'tracking_mode':                     1,
            'depth_scale_factor':                1000.0,
            'enable_image_denoising':            False,
            'rectified_images':                  False,
            'image_jitter_threshold_ms':         1000.0,
            'sync_matching_threshold_ms':        200.0,
            'base_frame':                        'base_footprint',
            'map_frame':                         'map',
            'odom_frame':                        'odom',
            'imu_frame':                         'camera_accel_gyro_optical_frame',
            'enable_localization_n_mapping':     True,
            'num_cameras':                       2,
            'min_num_images':                    2,
            'camera_optical_frames':             ['camera_left_ir_optical_frame', 'camera_right_ir_optical_frame'],
            'enable_slam_visualization':         True,
            'enable_landmarks_view':             True,
            'enable_observations_view':          True,
            'gyro_noise_density':                0.000244,
            'gyro_random_walk':                  0.000019393,
            'accel_noise_density':               0.001862,
            'accel_random_walk':                 0.003,
            'calibration_frequency':             200.0,
            'enable_ground_constraint_in_odometry': True,
            'enable_ground_constraint_in_slam':  True,
            # EKF (robot_localization) owns odom→base_footprint using wheel odometry.
            # map→odom is a static identity TF in navigation.launch.py (map frame = odom frame).
            # Disabling cuVSLAM's map→odom prevents VIO divergence in featureless corridors
            # from jumping the robot's map position and crashing the global planner.
            'publish_odom_to_base_tf':           False,
            'publish_map_to_odom_tf':            True,
        }],
        remappings=[
            ('visual_slam/image_0',       '/camera/left_ir/image_raw'),
            ('visual_slam/camera_info_0', '/camera/left_ir/camera_info'),
            ('visual_slam/image_1',       '/camera/right_ir/image_raw'),
            ('visual_slam/camera_info_1', '/camera/right_ir/camera_info'),
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
