#!/usr/bin/env python3
# One-shot floor-plane pitch check. Grabs /camera/depth/points, transforms to base_footprint with the
# CURRENT camera TF applied, isolates the floor strip in front of the robot, fits a plane, and reports
# the residual forward slope (dz/dx). Floor is correctly calibrated when that slope ~ 0 deg.
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

CURRENT_PITCH = -0.0995  # rad, the value now in the launches (5.70 deg nose-up)

class FloorCheck(Node):
    def __init__(self):
        super().__init__('floor_pitch_check')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(PointCloud2, '/camera/depth/points', self.cb, 1)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        try:
            t = self.buf.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'waiting for TF base_footprint <- {msg.header.frame_id}')
            return
        q, tr = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                      [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
        T = np.array([tr.x, tr.y, tr.z])
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        pb = (R @ pts.T).T + T                          # points in base_footprint
        m = (pb[:, 0] > 1.1) & (pb[:, 0] < 1.8) & (np.abs(pb[:, 1]) < 0.35) & (pb[:, 2] < 0.10)
        floor = pb[m]
        if floor.shape[0] < 200:
            self.get_logger().warn(f'only {floor.shape[0]} floor pts in the strip — aim at clear flat floor')
            return
        A = np.column_stack([floor[:, 0], floor[:, 1], np.ones(floor.shape[0])])
        (a, b, c), *_ = np.linalg.lstsq(A, floor[:, 2], rcond=None)   # z = a*x + b*y + c
        slope_deg = math.degrees(math.atan(a))
        suggested = CURRENT_PITCH + a                  # best-estimate; verify by re-running
        print('\n================ FLOOR PITCH CHECK ================')
        print(f' floor points fitted : {floor.shape[0]}')
        print(f' mean floor height z : {floor[:,2].mean():+.3f} m   (want ~0)')
        print(f' forward slope dz/dx : {a:+.4f}  =  {slope_deg:+.2f} deg   (want ~0)')
        print(f' lateral slope dz/dy : {b:+.4f}')
        print(f' current TF pitch    : {CURRENT_PITCH:+.4f} rad ({math.degrees(CURRENT_PITCH):+.2f} deg)')
        print(f' SUGGESTED new pitch : {suggested:+.4f} rad ({math.degrees(suggested):+.2f} deg)')
        print('   (apply, relaunch, re-run this — slope should shrink toward 0; if it grows, flip the sign)')
        print('===================================================\n')
        self.done = True
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(FloorCheck())

if __name__ == '__main__':
    main()
