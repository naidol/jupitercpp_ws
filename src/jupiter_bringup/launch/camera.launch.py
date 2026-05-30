# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Orbbec Gemini 336 — persistent camera launch (all streams).
#
# Run this ONCE in a separate terminal before any bringup launch.
# Never kill this process — the camera firmware stays warm, giving
# sub-2s init on every subsequent bringup restart.
#
# Streams enabled:
#   Color:     640x480 @ 15fps MJPG  → face recognition
#   Left IR:   640x480 @ 15fps       → cuVSLAM (nav mode)
#   Right IR:  640x480 @ 15fps       → cuVSLAM (nav mode)
#   IMU:       100Hz accel + gyro    → EKF odometry
#   Depth:     disabled              → not needed by any current subsystem
#   Laser:     OFF                   → mandatory for cuVSLAM passive IR tracking
#
# Usage:
#   ros2 launch jupiter_bringup camera.launch.py
#
# See config/orbbec_gemini336_profiles.md for valid stream profiles.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('orbbec_camera')
                + '/launch/gemini_330_series.launch.py'
            ),
            launch_arguments={
                'serial_number':                   'CP9KB53000HP',
                'usb_port':                        '2-1',
                # Color — face recognition
                'enable_color':                    'true',
                'color_width':                     '640',
                'color_height':                    '480',
                'color_fps':                       '15',
                'color_format':                    'MJPG',
                # IR stereo — cuVSLAM passive tracking (nav mode)
                'enable_left_ir':                  'true',
                'enable_right_ir':                 'true',
                'left_ir_width':                   '640',
                'left_ir_height':                  '480',
                'left_ir_fps':                     '15',
                'right_ir_width':                  '640',
                'right_ir_height':                 '480',
                'right_ir_fps':                    '15',
                # Laser projector OFF — mandatory for cuVSLAM passive IR
                'enable_laser':                    'false',
                # Depth off — not needed, saves DMA bandwidth
                'enable_depth':                    'false',
                'enable_point_cloud':              'false',
                'enable_colored_point_cloud':      'false',
                # IMU — EKF odometry fusion
                'enable_accel':                    'true',
                'enable_gyro':                     'true',
                'enable_sync_output_accel_gyro':   'true',
                'accel_rate':                      '100hz',
                'gyro_rate':                       '100hz',
            }.items(),
        ),
    ])
