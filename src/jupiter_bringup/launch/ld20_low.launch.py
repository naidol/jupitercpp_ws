# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# LD20 (LD19-compatible) LOW lidar — the second, low obstacle layer (~13 cm) that catches chair
# legs / feet / low furniture under the S2E's 0.515 m plane. Frees the camera from nav duty.
#
# Distinct topic (/scan_low) + frame (ld20_laser) so it coexists with the S2E (which owns /scan +
# base_laser). Mounting from the old navigation.launch.py: 6 cm forward, centred, 13 cm above ground.
#
# *** STANDALONE BRING-UP for verification first ***: run this, view /scan_low in RViz (Fixed Frame
# base_footprint or ld20_laser) and check (a) it publishes, (b) the 4 corner-upright blind sectors
# (4 close returns) and (c) walls land in the right place (if 180-deg off, change the TF yaw 0 -> pi).
# The 4 upright sectors will be angularly MASKED before this feeds the costmap.
#
# Run:
#   ros2 launch jupiter_bringup ld20_low.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # LD20 driver (LD19 profile) -> /scan_low in frame ld20_laser
        Node(
            package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='LD19_low',
            output='screen',
            parameters=[
                {'product_name':          'LDLiDAR_LD19'},
                {'topic_name':            'scan_low'},
                {'frame_id':              'ld20_laser'},
                {'port_name':             '/dev/jupiter_lidar'},
                {'port_baudrate':         230400},
                {'laser_scan_dir':        True},
                {'enable_angle_crop_func': False},
            ],
        ),

        # LD20 mount TF: 6 cm forward, centred, 13 cm above ground. Yaw=0 (the value that ran before;
        # the stale comment said pi). VERIFY vs reality in RViz; flip to 3.14159 if the scan is reversed.
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_ld20_laser',
            arguments=['0.06', '0', '0.1475', '0', '0', '0', 'base_footprint', 'ld20_laser'],  # +17.5mm: 100mm AGV wheels
        ),
    ])
