#!/usr/bin/env python3
"""
dock_range_monitor.py -- live wall-distance + squareness gauge for LiDAR docking.

The IR beacon tells the robot which SIDE of centre it is on, but gives NO range.
This monitor is the LiDAR half: it ranges the flat WALL behind the dock and prints
(a) perpendicular distance straight back to the wall, and (b) the wall angle
(squareness). Run it BEFORE trusting the numbers in the controller -- like
ir_monitor.py, it is a read-only diagnostic.

How it works: it takes the rear-facing sector of a LaserScan, converts the points
to (x, y) in the laser frame, and least-squares fits a line  x = m*y + c :
    c  = perpendicular distance straight back to the wall   (metres)
    m  = wall slope  ->  wall angle = atan(m)                (deg; 0 = square-on)
A line fit over many points is far steadier than two raw rays and, as a bonus,
reports the fit residual so you can see when the "wall" isn't actually flat
(clutter, an open doorway, the dock body poking into the plane, etc.).

REAR-DIRECTION BEARING (this is the subtle bit -- see navigation_s2e.launch.py):
  * S2E  /scan       frame base_laser  yaw = pi  -> robot REAR is at bearing   0 deg
  * LD20 /scan_low   frame ld20_laser  yaw = 0   -> robot REAR is at bearing 180 deg
The default (--bearing 0) is correct for the S2E /scan. For /scan_low pass
--bearing 180. Confirm the sector really points at the wall by watching the
distance change as you move the robot.

Usage (on Thor, robot's ROS env sourced, a LiDAR publishing):
    python3 scripts/dock_range_monitor.py                       # S2E /scan
    python3 scripts/dock_range_monitor.py --topic /scan_low --bearing 180
    python3 scripts/dock_range_monitor.py --half-width 25 --dock-depth 0.12
Ctrl-C to stop.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wrap_pi(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class DockRangeMonitor(Node):
    def __init__(self, args):
        super().__init__("dock_range_monitor")
        self.center = math.radians(args.bearing)      # rear-ward bearing in the laser frame
        self.half = math.radians(args.half_width)     # sector half-width
        self.dock_depth = args.dock_depth             # wall -> pogo-contact plane (m)
        self.rmin = args.range_min
        self.period = args.period
        self.last_print = 0.0
        # BEST_EFFORT sensor QoS -- matches how LiDAR drivers publish /scan.
        self.create_subscription(LaserScan, args.topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f"dock_range_monitor on {args.topic}: rear bearing {args.bearing:.0f}deg "
            f"+/-{args.half_width:.0f}deg, dock_depth {self.dock_depth:.3f} m. Ctrl-C to stop.")

    def on_scan(self, msg: LaserScan):
        now = time.monotonic()
        if now - self.last_print < self.period:
            return
        self.last_print = now

        # Collect (x, y) points whose bearing lies within the rear sector.
        xs, ys, rs = [], [], []
        ang = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > self.rmin and abs(wrap_pi(ang - self.center)) <= self.half:
                xs.append(r * math.cos(ang))
                ys.append(r * math.sin(ang))
                rs.append(r)
            ang += msg.angle_increment

        n = len(xs)
        if n < 5:
            print(f"  [rear sector] only {n} valid pts -- LiDAR blind here / out of range?")
            return

        # Least-squares fit x = m*y + c over the sector (y is the lateral axis).
        my = sum(ys) / n
        mx = sum(xs) / n
        syy = sum((y - my) ** 2 for y in ys)
        sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
        m = sxy / syy if syy > 1e-9 else 0.0     # slope: dx/dy
        c = mx - m * my                          # x at y=0 = straight-back distance
        wall_deg = math.degrees(math.atan(m))    # 0 = square-on to the wall
        # RMS residual of x about the fitted line -- how "flat wall" the sector really is.
        resid = math.sqrt(sum(((xs[i] - (m * ys[i] + c)) ** 2) for i in range(n)) / n)

        to_pogo = c - self.dock_depth
        square = "SQUARE" if abs(wall_deg) < 2.0 else ("wall left " if wall_deg > 0 else "wall right")
        print(f"  wall {c:5.3f} m | to-pogo {to_pogo:6.3f} m | "
              f"angle {wall_deg:+5.1f} deg ({square}) | "
              f"fit +/-{resid*1000:4.0f} mm | {n:3d} pts | near {min(rs):.3f} m")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/scan", help="LaserScan topic (default /scan = S2E)")
    ap.add_argument("--bearing", type=float, default=0.0,
                    help="rear-ward bearing in the laser frame, deg (S2E=0, LD20/scan_low=180)")
    ap.add_argument("--half-width", type=float, default=20.0,
                    help="sector half-width around the rear bearing, deg (default 20)")
    ap.add_argument("--dock-depth", type=float, default=0.0,
                    help="wall -> pogo-contact distance, m (measure it; subtracted from wall dist)")
    ap.add_argument("--range-min", type=float, default=0.05, help="ignore returns closer than this, m")
    ap.add_argument("--period", type=float, default=0.25, help="print period, s")
    args = ap.parse_args()

    rclpy.init()
    node = DockRangeMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
