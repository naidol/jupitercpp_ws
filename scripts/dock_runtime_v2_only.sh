#!/usr/bin/env bash
set -eo pipefail

# Starts shared docking runtime on Thor and ensures only dock_aligner_v2 is active.
ssh jupiter@192.168.0.8 '
set -eo pipefail
mkdir -p ~/logs
source /opt/ros/jazzy/setup.bash
source ~/jupitercpp_ws/install/setup.bash

# Keep one long-lived micro-ROS agent.
pgrep -a -x micro_ros_agent >/dev/null || \
  nohup bash -lc "source /opt/ros/jazzy/setup.bash; source $HOME/microros_ws/install/local_setup.bash; exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/jupiter_esp32 -b 460800" > ~/logs/micro_ros_agent_session.log 2>&1 &

# S2E LiDAR over UDP.
pgrep -a -x sllidar_node >/dev/null || \
  nohup bash -lc "source /opt/ros/jazzy/setup.bash; source $HOME/jupitercpp_ws/install/setup.bash; exec ros2 run sllidar_ros2 sllidar_node --ros-args -p channel_type:=udp -p udp_ip:=192.168.11.2 -p udp_port:=8089 -p frame_id:=base_laser -p scan_mode:=Sensitivity" > ~/logs/s2e_lidar_session.log 2>&1 &

# Rear-facing laser transform.
pgrep -a -x static_transform_publisher >/dev/null || \
  nohup bash -lc "source /opt/ros/jazzy/setup.bash; exec ros2 run tf2_ros static_transform_publisher --x 0.035 --y 0 --z 0.518 --yaw 3.14159265 --frame-id base_footprint --child-frame-id base_laser" > ~/logs/base_laser_tf_session.log 2>&1 &

# Reflector detector.
pgrep -a -x dock_reflector >/dev/null || \
  nohup bash -lc "source /opt/ros/jazzy/setup.bash; source $HOME/jupitercpp_ws/install/setup.bash; exec ros2 run jupiter_nodes dock_reflector" > ~/logs/dock_reflector.log 2>&1 &

# Force-stop legacy v1 aligner to prevent dual /cmd_vel publishers.
pkill -x dock_aligner || true

# Start v2 aligner only.
pgrep -a -x dock_aligner_v2 >/dev/null || \
  nohup bash -lc "source /opt/ros/jazzy/setup.bash; source $HOME/jupitercpp_ws/install/setup.bash; exec ros2 run jupiter_nodes dock_aligner_v2" > ~/logs/dock_aligner_v2.log 2>&1 &

echo "runtime launch checks complete (v2 only)"
pgrep -a -x dock_aligner || true
pgrep -a -x dock_aligner_v2 || true
'
