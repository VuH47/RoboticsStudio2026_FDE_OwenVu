#!/usr/bin/env python3
"""Inject a bumper obstacle scan when the robot touches something solid.

Nav2 only sees the 2D costmap. Gazebo can stop the chassis on a collision
the scan has not marked yet, and the controller then keeps commanding
forward until the progress checker times out. This node is that missing
bumper: a non-ground contact publishes a short forward scan return on
`bumper_scan` (0.70 m ahead of base_link) so both local and global costmaps
instantly mark lethal obstacle cost in front of the robot. This stops forward
trajectories naturally through DWB's BaseObstacle critic and allows Nav2's
native Behavior Tree recovery (<BackUp>) to back away without command contention.
"""

from typing import Iterable

# Just in front of the 0.5 m footprint, so the costmap marks the cell ahead
# of the robot rather than the cell under it.
BUMPER_RANGE_M = 0.70
CLEAR_RANGE_M = 5.0


def _own_body(name: str) -> bool:
    text = str(name).lower()
    return 'wheel' in text or 'base_link' in text or 'bumper' in text


def contact_blocks(names: Iterable[str]) -> bool:
    """True when the robot is touching something that is not itself or the ground.

    Wheel-on-snow contacts are constant and must not stop the drive. A wheel
    or the base against a rock or a bush must.
    """
    cleaned = [str(name) for name in names if str(name).strip()]
    if len(cleaned) < 2:
        return False
    if all(_own_body(name) or 'ground' in name.lower() for name in cleaned):
        return False
    return True


def bumper_range(blocking: bool) -> float:
    return BUMPER_RANGE_M if blocking else CLEAR_RANGE_M


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover - unit test loads only the head
    Contacts = None


class HuskyBumperStop(Node):
    def __init__(self) -> None:
        super().__init__('husky_bumper_stop')
        self.declare_parameter('hold_seconds', 0.4)
        self.declare_parameter('frame_id', 'base_link')
        self._blocking = False
        self._last_block_s = None
        self._scan_pub = self.create_publisher(LaserScan, 'bumper_scan', 10)
        if Contacts is None:
            self.get_logger().error(
                'ros_gz_interfaces Contacts is missing. Bumper stop is idle.')
        else:
            for topic in (
                'bumper_contact',
                'front_left_contact',
                'front_right_contact',
                'rear_left_contact',
                'rear_right_contact',
            ):
                self.create_subscription(Contacts, topic, self._on_contact, 10)
        self.create_timer(0.1, self._publish_scan)
        self.get_logger().info(
            'Bumper stop: wheel or base contact injects lethal cost on bumper_scan.')

    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _on_contact(self, msg) -> None:
        hit = False
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if contact_blocks(names):
                hit = True
                break
        if hit:
            self._blocking = True
            self._last_block_s = self._now()

    def _still_blocking(self) -> bool:
        if not self._blocking or self._last_block_s is None:
            return False
        hold = float(self.get_parameter('hold_seconds').value)
        if self._now() - self._last_block_s > hold:
            self._blocking = False
            return False
        return True

    def _publish_scan(self) -> None:
        blocking = self._still_blocking()
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = str(self.get_parameter('frame_id').value)
        scan.angle_min = -0.2
        scan.angle_max = 0.2
        scan.angle_increment = 0.1
        scan.range_min = 0.05
        scan.range_max = CLEAR_RANGE_M
        distance = bumper_range(blocking)
        scan.ranges = [distance, distance, distance, distance, distance]
        self._scan_pub.publish(scan)


def main() -> None:
    rclpy.init()
    node = HuskyBumperStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
