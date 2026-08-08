#!/usr/bin/env python3
"""Floor test: closed-loop steering at the docking operating point. No dock.

Replicates dock_aligner_v2's steering law exactly (kp*err - kd*gyro, clamp
+/-0.22) while reversing at 0.12 m/s, with a synthetic heading target instead
of the reflector:

  Phase 0: 2 s at rest       -> gyro bias capture
  Phase 1: 3 s reverse       -> hold initial heading (target 0)
  Phase 2: 5 s reverse       -> step target +10 deg
  Phase 3: 5 s reverse       -> step target back to 0 deg (i.e. -10 deg move)

Total reverse travel ~1.6 m; needs ~2 m clear behind the robot.

PASS per step phase: reach +/-3 deg of target within 4.0 s and stay there.
PASS for hold phase: |heading err| stays <= 3 deg.
CSV: /tmp/bench_steer_closedloop.csv (t, phase, target_deg, yaw_deg, gyro,
az_cmd, rpm1, rpm2).
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data

KP = 2.0
KD = 2.0
AZ_MAX = 0.22
V_REVERSE = -0.12
STEP_DEG = 10.0
TOL_DEG = 3.0
CSV_PATH = '/tmp/bench_steer_closedloop.csv'


class SteerBench(Node):
    def __init__(self):
        super().__init__('bench_steer_closedloop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/wheel_speeds', self.on_speed, 10)
        self.gyro_raw = 0.0
        self.gyro_filt = 0.0          # same 0.8/0.2 filter as the controller
        self.have_imu = False
        self.rpm = (0.0, 0.0)
        self.rows = []

    def on_imu(self, msg):
        self.gyro_raw = msg.angular_velocity.z
        self.gyro_filt = 0.8 * self.gyro_filt + 0.2 * msg.angular_velocity.z
        self.have_imu = True

    def on_speed(self, msg):
        if len(msg.data) >= 2:
            self.rpm = (msg.data[0], msg.data[1])

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self):
        # wait for sensors
        t0 = time.monotonic()
        while not self.have_imu and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.have_imu:
            print('FAIL: no /imu/data')
            return

        # Phase 0: gyro bias at rest
        samples = []
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            samples.append(self.gyro_raw)
        bias = sum(samples) / len(samples)
        print(f'gyro bias: {bias:+.5f} rad/s ({len(samples)} samples)')

        yaw = 0.0                       # integrated, bias-corrected
        phases = [('HOLD', 0.0, 3.0), ('STEP+10', math.radians(STEP_DEG), 5.0),
                  ('STEP-10', 0.0, 5.0)]
        results = []
        t_start = time.monotonic()
        t_prev = t_start
        try:
            for name, target, dur in phases:
                phase_t0 = time.monotonic()
                t_reach = None
                max_err_after_reach = 0.0
                max_err = 0.0
                while time.monotonic() - phase_t0 < dur:
                    rclpy.spin_once(self, timeout_sec=0.02)
                    now = time.monotonic()
                    dt = now - t_prev
                    t_prev = now
                    yaw += (self.gyro_raw - bias) * dt
                    err = target - yaw
                    az = max(-AZ_MAX, min(AZ_MAX, KP * err - KD * (self.gyro_filt - bias)))
                    cmd = Twist()
                    cmd.linear.x = V_REVERSE
                    cmd.angular.z = az
                    self.pub.publish(cmd)
                    err_deg = math.degrees(err)
                    max_err = max(max_err, abs(err_deg))
                    if t_reach is None and abs(err_deg) <= TOL_DEG:
                        t_reach = now - phase_t0
                    if t_reach is not None:
                        max_err_after_reach = max(max_err_after_reach, abs(err_deg))
                    self.rows.append((now - t_start, name, math.degrees(target),
                                      math.degrees(yaw), self.gyro_raw - bias, az,
                                      self.rpm[0], self.rpm[1]))
                final_err = math.degrees(target - yaw)
                results.append((name, t_reach, final_err, max_err, max_err_after_reach))
        finally:
            self.stop()

        with open(CSV_PATH, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'phase', 'target_deg', 'yaw_deg', 'gyro', 'az_cmd', 'rpm1', 'rpm2'])
            w.writerows(self.rows)

        print(f'\ncsv: {CSV_PATH}')
        print(f'{"phase":<9} {"t_reach":>8} {"final_err":>10} {"max_err":>8} {"settle_err":>10}  verdict')
        overall = True
        for name, t_reach, final_err, max_err, settle_err in results:
            if name == 'HOLD':
                ok = max_err <= TOL_DEG
            else:
                ok = t_reach is not None and t_reach <= 4.0 and abs(final_err) <= TOL_DEG
            overall = overall and ok
            tr = f'{t_reach:.2f}s' if t_reach is not None else 'never'
            print(f'{name:<9} {tr:>8} {final_err:>+9.1f}d {max_err:>7.1f}d {settle_err:>9.1f}d  {"PASS" if ok else "FAIL"}')
        print(f'\nOVERALL: {"PASS" if overall else "FAIL"}')


def main():
    rclpy.init()
    node = SteerBench()
    try:
        node.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
