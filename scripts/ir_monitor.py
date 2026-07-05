#!/usr/bin/env python3
"""
ir_monitor.py -- live monitor + NONE-gap gauge for the IR dock beacon.

Subscribes to /dock/ir (std_msgs/UInt8, ~50 Hz from the ESP32 via micro-ROS):
    0 = NONE   1 = LEFT only   2 = RIGHT only   3 = BOTH (centred)

Two views at once:
  * transition events  -- a line each time the state changes (the old behaviour)
  * rolling summary    -- once a second: sample rate, per-state %, and the key
                          metric for judging the 90/10 ms burst firmware:
                          how many NONE gaps and the longest NONE gap (ms).

A NONE gap = a contiguous run of 0s. When the robot is sitting in the beam, a
healthy beacon+firmware should keep %NONE low and the longest gap short
(the dock_ir controller holds last state until beam_lost_timeout = 2.0 s).

Usage (on Thor, with the robot's ROS env sourced):
    python3 scripts/ir_monitor.py [--window SECONDS] [--no-events]
Ctrl-C to stop -- prints a cumulative session summary.
"""
import argparse
import time
from collections import deque, Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import UInt8

LABEL = {0: "--- NONE ---", 1: "<<< LEFT only", 2: "RIGHT only >>>", 3: "=== BOTH (centred) ==="}
SHORT = {0: "NONE", 1: "LEFT", 2: "RIGHT", 3: "BOTH"}


class Monitor(Node):
    def __init__(self, window, show_events):
        super().__init__("ir_monitor")
        self.window = window
        self.show_events = show_events
        self.samples = deque()          # (t, value) within the rolling window
        self.total = Counter()          # cumulative over the whole run
        self.total_n = 0
        self.last = None
        self.t0 = time.monotonic()
        # BEST_EFFORT sub is compatible with both reliable and best-effort
        # publishers -- safest for a micro-ROS sensor stream.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(UInt8, "/dock/ir", self.cb, qos)
        self.create_timer(1.0, self.report)
        print(f"ir_monitor: watching /dock/ir  (rolling {window:.0f}s window)  Ctrl-C to stop\n")

    def cb(self, msg):
        now = time.monotonic()
        v = int(msg.data)
        self.samples.append((now, v))
        self.total[v] += 1
        self.total_n += 1
        if self.show_events and v != self.last:
            print(f"[{now - self.t0:7.1f}s] {LABEL.get(v, '?? %d' % v)}")
            self.last = v

    @staticmethod
    def _gap_stats(samples):
        """longest contiguous NONE run (s) and number of NONE runs."""
        longest = 0.0
        runs = 0
        run_start = None
        prev = None
        for t, v in samples:
            if v == 0 and prev != 0:
                run_start = t
                runs += 1
            elif v != 0 and prev == 0 and run_start is not None:
                longest = max(longest, t - run_start)
            prev = v
        if prev == 0 and run_start is not None:      # window ends mid-gap
            longest = max(longest, samples[-1][0] - run_start)
        return longest, runs

    def report(self):
        now = time.monotonic()
        while self.samples and now - self.samples[0][0] > self.window:
            self.samples.popleft()
        n = len(self.samples)
        if n == 0:
            print("  (no /dock/ir msgs in window -- beacon off / robot out of range / stack down?)")
            return
        counts = Counter(v for _, v in self.samples)
        span = self.samples[-1][0] - self.samples[0][0]
        hz = (n - 1) / span if span > 0 else 0.0
        longest, runs = self._gap_stats(self.samples)
        bar = "  ".join(f"{SHORT[k]} {100.0 * counts.get(k, 0) / n:4.0f}%" for k in (0, 1, 2, 3))
        print(f"  ~{hz:4.1f}Hz n={n:3d} | {bar} | NONE gaps:{runs:2d} longest {longest * 1000:5.0f}ms")

    def summary(self):
        if self.total_n == 0:
            print("no samples received.")
            return
        dur = time.monotonic() - self.t0
        bar = "  ".join(f"{SHORT[k]} {100.0 * self.total.get(k, 0) / self.total_n:4.0f}%" for k in (0, 1, 2, 3))
        print(f"\n=== session summary ({dur:.0f}s, {self.total_n} samples) ===")
        print(f"  {bar}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=float, default=5.0, help="rolling window seconds (default 5)")
    ap.add_argument("--no-events", action="store_true", help="summary only, hide transition lines")
    a = ap.parse_args()
    rclpy.init()
    node = Monitor(a.window, not a.no_events)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
