#!/usr/bin/env python3
"""Bench test: does the ESP32 velocity PID EVENTUALLY reach commanded wheel speed?

WHY THIS EXISTS
---------------
bench_wheel_breakaway.py (2026-08-08) measured 50-70 % wheel-speed tracking and
reported it as steady-state, concluding the firmware PID is structurally broken.
But that script used STEP_S = 3.0 s and averaged only the LAST 1.0 s of it, so it
sampled t = 2-3 s. With K_I = 5 and a ~4 RPM error at ~20.8 Hz the integral adds
only ~20 PWM/s (PWM_MAX = 1023, not 255) — it needs roughly 7 s to build the
~150 PWM a wheel wants to overcome low-speed friction. So that measurement is a
TRANSIENT, not a steady state, and cannot distinguish:

    (a) UNDER-GAINED  — the loop converges, just far too slowly for docking
                        -> fix is a K_I / K_P retune (possibly speed-scheduled)
    (b) STRUCTURAL    — the loop never converges at all
                        -> fix is the coupled velocity + yaw-rate rewrite

This script holds each speed long enough to tell them apart, and reports the
tracking ratio in successive time WINDOWS so the convergence curve is visible
rather than a single endpoint.

READ THE RESULT
---------------
    tracking % rising across the windows  -> (a) UNDER-GAINED. Retune, no rewrite.
    tracking % flat across the windows    -> (b) STRUCTURAL. Rewrite justified.

STRAIGHT-LINE ONLY. The differential (Phase B) test is deliberately NOT included:
if authority IS restored, az = 0.22 held for 10 s would sweep ~126 deg of arc.

SPACE / SAFETY
--------------
Longest single reverse = 0.12 m/s x 10 s = 1.2 m. Needs ~1.5 m clear BEHIND
(the rear/S2E end — that is the direction it travels) and ~2 m clear in FRONT
(return moves). Open floor, NO dock in the lane. Run on the SAME floor surface as
the dock approach — this test is friction-dependent and will not transfer between
tile and laminate. Warm teleop ready as e-stop. Ctrl-C stops immediately.

Battery voltage is logged: motor torque per unit PWM scales with pack voltage, so
a low pack can masquerade as poor tracking. Compare runs at similar voltage.

    ros2 run  --  (plain python3, needs ROS 2 sourced)
    python3 bench_wheel_tracking_long.py [--step-s 10] [--speeds 0.04,0.06,0.08,0.12]
"""
import argparse
import csv
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import BatteryState

WHEEL_CIRC = 0.3142       # 100 mm AGV wheel circumference (m)
RETURN_V = 0.15           # forward return speed
SETTLE_S = 2.0            # rest between moves (let stiction re-engage)
PUBLISH_HZ = 20.0

# Windows (seconds from step start) the tracking ratio is averaged over.
# 2-3 s reproduces the original script's measurement for direct comparison.
WINDOWS = [(0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (4.0, 6.0), (6.0, 8.0), (8.0, 10.0)]


def mps_to_rpm(mps):
    return mps / WHEEL_CIRC * 60.0


class Bench(Node):
    def __init__(self):
        super().__init__('bench_wheel_tracking_long')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/wheel_speeds', self.on_speed, 10)
        self.create_subscription(BatteryState, '/battery/state', self.on_batt, 10)
        self.samples = []
        self.capturing = False
        self.voltage = float('nan')
        self.rpm = (0.0, 0.0)

    def on_speed(self, msg):
        if len(msg.data) >= 2:
            self.rpm = (msg.data[0], msg.data[1])
            if self.capturing:
                self.samples.append((time.monotonic(), msg.data[0], msg.data[1]))

    def on_batt(self, msg):
        self.voltage = msg.voltage

    def drive(self, vx, duration, capture=False):
        """Publish a constant twist for `duration`, optionally recording wheel speeds."""
        if capture:
            self.samples = []
            self.capturing = True
        cmd = Twist()
        cmd.linear.x = vx
        end = time.monotonic() + duration
        period = 1.0 / PUBLISH_HZ
        while time.monotonic() < end:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=period)
        self.capturing = False
        self.stop()

    def stop(self):
        for _ in range(3):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def rest(self, seconds=SETTLE_S):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def travelled_m(self):
        """Integrate measured wheel speed over the captured step -> distance actually moved.

        The original script assumed the commanded distance was achieved and returned
        that far forward; since actual travel is only ~60 % of commanded, the robot
        crept forward every cycle. Integrating the encoders removes that drift.
        """
        dist = 0.0
        for i in range(1, len(self.samples)):
            dt = self.samples[i][0] - self.samples[i - 1][0]
            mean_rpm = (abs(self.samples[i][1]) + abs(self.samples[i][2])) / 2.0
            dist += mean_rpm / 60.0 * WHEEL_CIRC * dt
        return dist

    def windows(self, cmd_rpm):
        """Mean tracking ratio (actual/commanded) per time window."""
        if not self.samples:
            return []
        t0 = self.samples[0][0]
        out = []
        for lo, hi in WINDOWS:
            pts = [(r1, r2) for (t, r1, r2) in self.samples if lo <= (t - t0) < hi]
            if not pts:
                out.append((lo, hi, None, None, None))
                continue
            m1 = sum(abs(a) for a, _ in pts) / len(pts)
            m2 = sum(abs(b) for _, b in pts) / len(pts)
            trk = (m1 + m2) / 2.0 / abs(cmd_rpm) * 100.0 if cmd_rpm else 0.0
            out.append((lo, hi, m1, m2, trk))
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--step-s', type=float, default=10.0, help='dwell per speed (s)')
    ap.add_argument('--speeds', type=str, default='0.04,0.06,0.08,0.12',
                    help='comma-separated reverse speeds (m/s)')
    ap.add_argument('--csv', type=str, default='/tmp/bench_wheel_tracking_long.csv')
    args = ap.parse_args()
    speeds = [float(s) for s in args.speeds.split(',')]

    rclpy.init()
    node = Bench()
    signal.signal(signal.SIGINT, lambda *_: (node.stop(), sys.exit(1)))

    # let battery/wheel topics arrive
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"battery at start: {node.voltage:.2f} V")
    print(f"dwell {args.step_s:.0f} s per speed, speeds {speeds}")
    print(f"longest reverse this run: {max(speeds) * args.step_s:.2f} m\n")

    rows = []
    for v in speeds:
        cmd_rpm = mps_to_rpm(v)
        print(f"--- v = -{v:.2f} m/s  (commanded {cmd_rpm:.1f} RPM/wheel) ---")
        node.drive(-v, args.step_s, capture=True)
        wins = node.windows(cmd_rpm)
        for lo, hi, m1, m2, trk in wins:
            if trk is None:
                print(f"    {lo:4.1f}-{hi:4.1f}s   (no samples)")
                continue
            print(f"    {lo:4.1f}-{hi:4.1f}s   m1 {m1:5.1f}  m2 {m2:5.1f} RPM   tracking {trk:5.1f}%")
            rows.append([v, cmd_rpm, lo, hi, m1, m2, trk, node.voltage])
        first = next((t for (lo, hi, m1, m2, t) in wins if t is not None), None)
        last = next((t for (lo, hi, m1, m2, t) in reversed(wins) if t is not None), None)
        if first is not None and last is not None:
            print(f"    => {first:.0f}% -> {last:.0f}%   "
                  f"({'RISING (under-gained)' if last - first > 8 else 'FLAT (structural)'})")
        travelled = node.travelled_m()
        node.rest()
        node.drive(RETURN_V, travelled / RETURN_V)   # return the distance ACTUALLY travelled
        node.rest()
        print()

    node.stop()
    with open(args.csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['vx', 'cmd_rpm', 'win_lo_s', 'win_hi_s', 'm1_rpm', 'm2_rpm',
                    'tracking_pct', 'battery_v'])
        w.writerows(rows)
    print(f"battery at end: {node.voltage:.2f} V")
    print(f"CSV: {args.csv}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
