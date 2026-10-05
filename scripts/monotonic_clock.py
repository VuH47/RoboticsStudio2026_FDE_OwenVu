#!/usr/bin/env python3
"""Republish Gazebo's clock and drop any sample that steps backwards.

tf2 and RViz clear their whole buffer on a negative /clock step. That is
the "Detected jump back in time. Resetting RViz." blank. This node must
be started with use_sim_time false: it is the publisher of /clock.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def _nsec(msg: Clock) -> int:
    return int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)


class MonotonicClock(Node):
    def __init__(self) -> None:
        super().__init__('monotonic_clock')
        # Same profile rclcpp uses to subscribe to /clock (best effort, depth 1).
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._last = -1
        self._dropped = 0
        self._pub = self.create_publisher(Clock, '/clock', qos)
        self.create_subscription(Clock, '/clock_gz', self._on_clock, qos)

    def _on_clock(self, msg: Clock) -> None:
        stamp = _nsec(msg)
        if stamp < self._last:
            self._dropped += 1
            self.get_logger().warn(
                'Dropped a backwards clock step (%d so far).' % self._dropped,
                throttle_duration_sec=5.0,
            )
            return
        self._last = stamp
        self._pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MonotonicClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
