#!/usr/bin/env python3
"""
drive_pulse.py -- publish a bounded /cmd_vel pulse, then STOP. Drive-isolation test.

Commands linear.x = <vx> (and optional angular.z = <wz>) for <dur> seconds, then
publishes zero hard. Use to measure the robot's real translation/rotation against
the LiDAR, decoupled from any controller. KILL dock_ir first (it publishes zeros
when idle and will fight this).

Usage (on Thor, ROS env sourced):
    python3 scripts/drive_pulse.py <vx> <dur> [wz]
    python3 scripts/drive_pulse.py -0.5 0.4          # reverse 0.5 m/s for 0.4 s
    python3 scripts/drive_pulse.py -0.5 0.4 0.0
Keep dur*|vx| well under the remaining distance to anything. Hand near the stop.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

vx  = float(sys.argv[1]) if len(sys.argv) > 1 else -0.3
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
wz  = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

rclpy.init()
node = rclpy.create_node("drive_pulse")
pub = node.create_publisher(Twist, "/cmd_vel", 10)
time.sleep(0.3)  # let discovery match subscribers before the first publish

cmd = Twist()
cmd.linear.x = vx
cmd.angular.z = wz
t_end = time.time() + dur
while time.time() < t_end:
    pub.publish(cmd)
    time.sleep(0.02)

# Hard stop: hammer zero for ~0.8 s so the motors definitely see it.
stop = Twist()
for _ in range(40):
    pub.publish(stop)
    time.sleep(0.02)

node.destroy_node()
rclpy.shutdown()
print(f"pulse done: vx={vx} m/s, wz={wz} rad/s for {dur}s, then STOP")
