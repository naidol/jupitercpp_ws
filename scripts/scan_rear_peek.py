#!/usr/bin/env python3
"""
scan_rear_peek.py -- print the LiDAR range profile across the rear sector, to see
what dock_range is actually fitting (clean flat wall vs a distance step from dock
structure). Prints one scan: (bearing_deg, range_m) sampled across ±HALF around the
rear bearing. A clean wall = ranges rise smoothly + symmetrically; a step/jump = an
object (dock structure) in front of the wall.

Usage (on Thor): python3 scripts/scan_rear_peek.py [--bearing 0] [--half 25]
"""
import argparse, math, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bearing", type=float, default=0.0)
    ap.add_argument("--half", type=float, default=25.0)
    ap.add_argument("--topic", default="/scan")
    args = ap.parse_args()
    c = math.radians(args.bearing); h = math.radians(args.half)

    rclpy.init()
    node = rclpy.create_node("scan_rear_peek")
    done = {"v": False}

    def cb(msg):
        if done["v"]:
            return
        done["v"] = True
        pts = []
        a = msg.angle_min
        for r in msg.ranges:
            if abs(wrap(a - c)) <= h and math.isfinite(r) and r > 0.05:
                pts.append((math.degrees(wrap(a - c)), r))
            a += msg.angle_increment
        pts.sort()
        print(f"  rear sector {args.bearing:+.0f}±{args.half:.0f}°, {len(pts)} pts:")
        step = max(1, len(pts) // 24)
        for i in range(0, len(pts), step):
            d, r = pts[i]
            bar = "#" * int((r - 0.3) * 40) if r > 0.3 else ""
            print(f"    {d:+5.1f}°  {r:5.3f} m  {bar}")

    node.create_subscription(LaserScan, args.topic, cb, qos_profile_sensor_data)
    t = time.time()
    while not done["v"] and time.time() - t < 6:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
