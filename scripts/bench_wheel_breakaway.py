#!/usr/bin/env python3
"""Bench test: measure per-wheel breakaway and differential steering authority.

Open-floor test, no dock. Commands /cmd_vel steps and records actual wheel RPM
from /wheel_speeds (ESP32 encoders). Each reverse step starts from rest after a
forward return move, so casters are re-cocked = realistic worst case.
Needs ~1 m clear behind the robot and ~0.5 m in front.
"""
import csv
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

WHEEL_CIRC = 0.3142       # 100 mm AGV wheel circumference (m)
HALF_SEP = 0.355 / 2.0    # WHEEL_SEPARATION 0.355 (jupiter_config.h)
STEP_S = 3.0              # duration of each measured step
SETTLE_S = 2.0            # rest between steps (stiction re-engages)
RETURN_V = 0.15           # forward return speed (known-good)
MOVING_RPM = 2.0          # |RPM| above this counts as "moving"

# Phase A: straight reverse, breakaway from rest
PHASE_A = [0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12]
# Phase B: differential while reversing (v, az). First pair = trial-4 saturated case.
PHASE_B = [(0.06, 0.10), (0.06, -0.10), (0.08, 0.15), (0.08, -0.15),
           (0.10, 0.20), (0.10, -0.20), (0.12, 0.22), (0.12, -0.22)]


def mps_to_rpm(mps):
    return mps / WHEEL_CIRC * 60.0


class Bench(Node):
    def __init__(self):
        super().__init__('bench_wheel_breakaway')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Float32MultiArray, '/wheel_speeds', self.on_speed, 10)
        self.samples = []          # (t, rpm1, rpm2) during current capture
        self.capturing = False

    def on_speed(self, msg):
        if self.capturing and len(msg.data) >= 2:
            self.samples.append((time.monotonic(), msg.data[0], msg.data[1]))

    def command(self, vx, az, duration, capture=False):
        if capture:
            self.samples = []
            self.capturing = True
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = az
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.capturing = False
        self.stop()

    def stop(self):
        self.pub.publish(Twist())

    def rest(self):
        end = time.monotonic() + SETTLE_S
        while time.monotonic() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def analyze(self, vx, az):
        if not self.samples:
            return None
        t0 = self.samples[0][0]
        # steady state: last 1.0 s of the step
        t_end = self.samples[-1][0]
        steady = [(r1, r2) for (t, r1, r2) in self.samples if t >= t_end - 1.0]
        mean1 = sum(r for r, _ in steady) / len(steady)
        mean2 = sum(r for _, r in steady) / len(steady)
        move1 = next((t - t0 for (t, r1, _) in self.samples if abs(r1) > MOVING_RPM), None)
        move2 = next((t - t0 for (t, _, r2) in self.samples if abs(r2) > MOVING_RPM), None)
        return mean1, mean2, move1, move2


def main():
    rclpy.init()
    node = Bench()
    signal.signal(signal.SIGINT, lambda *_: (node.stop(), sys.exit(1)))

    rows = []

    def run_step(label, vx, az):
        # commanded per-wheel m/s per diff-drive kinematics
        cmd_l = vx - az * HALF_SEP
        cmd_r = vx + az * HALF_SEP
        node.command(vx, az, STEP_S, capture=True)
        result = node.analyze(vx, az)
        if result is None:
            print(f"{label}: NO /wheel_speeds DATA — check micro-ROS agent")
            node.rest()
            return
        m1, m2, t1, t2 = result
        rows.append([label, vx, az, cmd_l, cmd_r, mps_to_rpm(cmd_l), mps_to_rpm(cmd_r),
                     m1, m2, t1, t2])
        print(f"{label}: cmd L/R {mps_to_rpm(cmd_l):+6.1f}/{mps_to_rpm(cmd_r):+6.1f} RPM | "
              f"actual m1/m2 {m1:+6.1f}/{m2:+6.1f} RPM | first-motion {t1 if t1 is not None else 'NEVER'} / "
              f"{t2 if t2 is not None else 'NEVER'} s")
        node.rest()
        # return to start (forward, same distance), then rest so casters re-cock
        node.command(RETURN_V, 0.0, abs(vx) * STEP_S / RETURN_V)
        node.rest()

    print("=== PHASE A: straight reverse breakaway from rest ===")
    for v in PHASE_A:
        run_step(f"A v=-{v:.2f}", -v, 0.0)

    print("=== PHASE B: differential authority while reversing ===")
    for v, az in PHASE_B:
        run_step(f"B v=-{v:.2f} az={az:+.2f}", -v, az)

    node.stop()
    out = '/tmp/bench_wheel_breakaway.csv'
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['label', 'vx', 'az', 'cmd_l_mps', 'cmd_r_mps', 'cmd_l_rpm', 'cmd_r_rpm',
                    'actual_m1_rpm', 'actual_m2_rpm', 'first_motion_m1_s', 'first_motion_m2_s'])
        w.writerows(rows)
    print(f"CSV: {out}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
