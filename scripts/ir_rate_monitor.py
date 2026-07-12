#!/usr/bin/env python3
"""
ir_rate_monitor.py -- smoothed per-side IR burst-RATE + balance, for characterising
the graded alignment signal (the receiver "heartbeat").

Subscribes to /dock/ir_rate (std_msgs/Float32MultiArray, [left_eps, right_eps] from
the ESP32). The raw rate is quantised by beacon-packet phase, so this keeps a rolling
time-average and prints:

    L<avg>  R<avg>  sum<L+R>  balance<(L-R)/(L+R)>   [====|====] bar

  * balance  = 0   -> both sides equal  -> centred / square (the target)
  * balance  > 0   -> LEFT stronger     -> robot offset toward the left beam
  * balance  < 0   -> RIGHT stronger
  * sum              -> proximity / signal strength (higher = closer/better aligned)

SWEEP TEST: hold the robot square at the mouth distance and slide it left<->right;
watch whether balance moves smoothly through 0 (usable steering gradient) or snaps
(cliff = only good near centre). Repeat at 1.0 m / 0.5 m / at the mouth.

Usage (on Thor, ROS env sourced):
    python3 scripts/ir_rate_monitor.py [--window SEC] [--rate HZ]
"""
import argparse
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray


class IrRateMonitor(Node):
    def __init__(self, window, hz):
        super().__init__("ir_rate_monitor")
        self.window = window
        self.samples = deque()               # (t, left, right)
        self.create_subscription(Float32MultiArray, "/dock/ir_rate",
                                 self.on_rate, qos_profile_sensor_data)
        self.create_timer(1.0 / hz, self.report)
        self.get_logger().info(
            f"ir_rate_monitor: {window:.1f}s rolling average. Sweep the robot L<->R "
            f"and watch 'balance' cross 0 at centre. Ctrl-C to stop.")

    def on_rate(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        now = time.monotonic()
        self.samples.append((now, float(msg.data[0]), float(msg.data[1])))
        cutoff = now - self.window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def report(self):
        if not self.samples:
            print("  (no /dock/ir_rate yet)")
            return
        n = len(self.samples)
        left = sum(s[1] for s in self.samples) / n
        right = sum(s[2] for s in self.samples) / n
        total = left + right
        balance = (left - right) / total if total > 1e-6 else 0.0
        # bar: 21 cells, centre = balanced
        pos = int(round((balance + 1.0) * 10))          # 0..20
        pos = max(0, min(20, pos))
        bar = "".join("|" if i == 10 else ("#" if i == pos else "-") for i in range(21))
        side = "centre" if abs(balance) < 0.05 else ("LEFT " if balance > 0 else "RIGHT")
        print(f"  L {left:6.1f}  R {right:6.1f}  sum {total:6.1f}  "
              f"balance {balance:+.3f} ({side})  [{bar}]")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--window", type=float, default=1.5, help="rolling-average window, s")
    ap.add_argument("--rate", type=float, default=4.0, help="print rate, Hz")
    args = ap.parse_args()

    rclpy.init()
    node = IrRateMonitor(args.window, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
