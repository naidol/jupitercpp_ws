#!/usr/bin/env python3
# Reads one /scan_low message and prints the min range per 10-deg sector, to locate the 4 corner-
# upright blind sectors (very-close returns) and confirm the scan is healthy.
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSectors(Node):
    def __init__(self):
        super().__init__('scan_sectors')
        self.create_subscription(LaserScan, '/scan_low', self.cb, 1)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        r = np.array(msg.ranges, dtype=float)
        ang = np.degrees(msg.angle_min + np.arange(len(r)) * msg.angle_increment)
        ang = (ang + 180.0) % 360.0 - 180.0          # wrap to [-180, 180)
        valid = np.isfinite(r) & (r > 0.001)
        print(f'\n=== /scan_low: {len(r)} beams, {math.degrees(msg.angle_min):.0f}..{math.degrees(msg.angle_max):.0f} deg, '
              f'range {msg.range_min:.2f}..{msg.range_max:.2f} m, valid {int(valid.sum())}/{len(r)} ===')
        print(' sector(deg)   min(m)  median(m)  n_close(<0.30m)   flag')
        for lo in range(-180, 180, 10):
            hi = lo + 10
            m = (ang >= lo) & (ang < hi) & valid
            if m.sum() == 0:
                print(f'  [{lo:+4d},{hi:+4d})   (no valid returns)')
                continue
            rr = r[m]
            nclose = int((rr < 0.30).sum())
            flag = '  <-- UPRIGHT/structure' if (rr.min() < 0.30 and nclose >= 2) else ''
            print(f'  [{lo:+4d},{hi:+4d})   {rr.min():.2f}    {np.median(rr):.2f}      {nclose:3d}            {flag}')
        self.done = True
        rclpy.shutdown()


rclpy.init()
rclpy.spin(ScanSectors())
