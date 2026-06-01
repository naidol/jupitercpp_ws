# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Orbbec Gemini 336 — AI/registration test camera (COLOR ONLY).
#
# Use this instead of camera.launch.py when testing voice/face/registration
# WITHOUT navigation. Dropping the two IR stereo streams frees tegra-xusb DMA
# bandwidth so the ReSpeaker USB audio capture is not starved — fixes the
# pw-record "short buffer / underrun" failures seen during registration.
#
# Trade-off: color-only triggers a ~20s Orbbec SDK cold-start timeout (the SDK
# waits for depth HW). Acceptable for a persistent camera started once.
#
# Run ONCE in its own terminal, never kill:
#   ros2 launch jupiter_bringup camera_ai.launch.py
#
# (For full nav + AI, use camera.launch.py which adds IR stereo for cuVSLAM.)

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
                # Color only — face recognition. No IR (saves DMA for ReSpeaker).
                'enable_color':                    'true',
                'color_width':                     '640',
                'color_height':                    '480',
                'color_fps':                       '15',
                'color_format':                    'MJPG',
                'enable_left_ir':                  'false',
                'enable_right_ir':                 'false',
                'enable_depth':                    'false',
                'enable_laser':                    'false',
                'enable_point_cloud':              'false',
                'enable_colored_point_cloud':      'false',
                # IMU kept on — negligible bandwidth, harmless.
                'enable_accel':                    'true',
                'enable_gyro':                     'true',
                'enable_sync_output_accel_gyro':   'true',
                'accel_rate':                      '100hz',
                'gyro_rate':                       '100hz',
            }.items(),
        ),
    ])
