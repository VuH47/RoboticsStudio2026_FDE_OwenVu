#!/usr/bin/env python3
"""Publish an alpine 'path preview' ribbon from Nav2 plans.

This is the visualisation language students associate with consumer
full-self-driving UIs: a wide, slightly transparent cyan carpet on the
ground that shows where the robot intends to drive *before* it gets
there. It is not a Tesla clone. There is no Autopilot chrome, no
wordmark, and no copied HUD layout. The ribbon is original 4SROS
styling for the alpine trail MVP.

Typical topics (node is launched in /husky1 or /parrot1):

  in:  plan, local_plan, received_global_plan   (nav_msgs/Path)
  in:  /operator/state                          (std_msgs/String)
  in:  /mission/hazard_markers                  (visualization_msgs/MarkerArray)
  out: path_preview/markers                     (visualization_msgs/MarkerArray)
  out: path_preview/path                        (nav_msgs/Path)

Add a MarkerArray display in RViz on ``path_preview/markers`` and a Path
display on ``path_preview/path``. Send a Nav2 Goal and the ribbon appears.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


# Palette is original to 4SROS (ice / amber / hazard), not a vendor UI.
COLOURS = {
    'idle':        (0.55, 0.62, 0.70, 0.25),
    'await':       (0.96, 0.76, 0.29, 0.45),   # amber: waiting for operator
    'navigating':  (0.24, 0.78, 1.00, 0.50),   # alpine cyan: committed path
    'hold':        (0.95, 0.55, 0.20, 0.45),
    'abort':       (1.00, 0.23, 0.31, 0.50),
    'done':        (0.24, 0.86, 0.59, 0.40),
    'local':       (0.55, 0.92, 1.00, 0.70),
    'center':      (0.85, 0.97, 1.00, 0.95),
}


def _state_key(raw: str) -> str:
    text = (raw or '').strip().lower()
    if 'await' in text or 'approv' in text:
        return 'await'
    if 'nav' in text or 'dispatch' in text:
        return 'navigating'
    if 'hold' in text or 'pause' in text:
        return 'hold'
    if 'abort' in text or 'return' in text or 'rtb' in text:
        return 'abort'
    if 'done' in text or 'complete' in text:
        return 'done'
    return 'idle'


def _colour(key: str) -> ColorRGBA:
    r, g, b, a = COLOURS.get(key, COLOURS['idle'])
    msg = ColorRGBA()
    msg.r, msg.g, msg.b, msg.a = r, g, b, a
    return msg


def _point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _path_xy(path: Path) -> List[Tuple[float, float, float]]:
    pts: List[Tuple[float, float, float]] = []
    last = None
    for pose in path.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        if last is not None:
            dx, dy = x - last[0], y - last[1]
            if dx * dx + dy * dy < 0.04:
                continue
        pts.append((x, y, z))
        last = (x, y, z)
    return pts


def _ribbon_triangles(
    pts: Sequence[Tuple[float, float, float]],
    half_width: float,
    lift: float,
) -> List[Point]:
    """Build a triangle-list ribbon sitting just above the snow."""
    if len(pts) < 2:
        return []
    left: List[Point] = []
    right: List[Point] = []
    n = len(pts)
    for i, (x, y, z) in enumerate(pts):
        if i == 0:
            dx, dy = pts[1][0] - x, pts[1][1] - y
        elif i == n - 1:
            dx, dy = x - pts[i - 1][0], y - pts[i - 1][1]
        else:
            dx, dy = pts[i + 1][0] - pts[i - 1][0], pts[i + 1][1] - pts[i - 1][1]
        length = math.hypot(dx, dy) or 1.0
        # Taper: a little wider near the robot (first points) like a guidance carpet.
        taper = 1.0 - 0.35 * (i / max(n - 1, 1))
        nx, ny = (-dy / length) * half_width * taper, (dx / length) * half_width * taper
        zz = z + lift
        left.append(_point(x + nx, y + ny, zz))
        right.append(_point(x - nx, y - ny, zz))

    triangles: List[Point] = []
    for i in range(n - 1):
        triangles.extend([left[i], right[i], left[i + 1]])
        triangles.extend([right[i], right[i + 1], left[i + 1]])
    return triangles


class PathPreviewViz(Node):
    """Turn Nav2 paths into a ground ribbon the operator can trust."""

    def __init__(self) -> None:
        super().__init__('path_preview_viz')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('global_plan_topic', 'plan')
        self.declare_parameter('received_plan_topic', 'received_global_plan')
        self.declare_parameter('local_plan_topic', 'local_plan')
        self.declare_parameter('marker_topic', 'path_preview/markers')
        self.declare_parameter('styled_path_topic', 'path_preview/path')
        self.declare_parameter('state_topic', '/operator/state')
        self.declare_parameter('hazard_topic', '/mission/hazard_markers')
        self.declare_parameter('ribbon_width', 1.6)
        self.declare_parameter('local_width', 0.7)
        self.declare_parameter('lift', 0.08)
        self.declare_parameter('lifetime_sec', 1.0)

        self.robot_name = self._resolve_robot_name()
        self.map_frame = f'{self.robot_name}_map'
        self.global_pts: List[Tuple[float, float, float]] = []
        self.local_pts: List[Tuple[float, float, float]] = []
        self.global_path: Optional[Path] = None
        self.state = 'IDLE'
        self.hazards = MarkerArray()

        plan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensorish_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Path, self.get_parameter('global_plan_topic').value,
                                 self._on_global, plan_qos)
        self.create_subscription(Path, self.get_parameter('received_plan_topic').value,
                                 self._on_global, sensorish_qos)
        self.create_subscription(Path, self.get_parameter('local_plan_topic').value,
                                 self._on_local, sensorish_qos)
        self.create_subscription(String, self.get_parameter('state_topic').value,
                                 self._on_state, 10)
        self.create_subscription(MarkerArray, self.get_parameter('hazard_topic').value,
                                 self._on_hazards, 10)

        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('styled_path_topic').value, 10)
        self.create_timer(0.1, self._publish)

        self.get_logger().info(
            f'Path preview ready for {self.robot_name} in frame {self.map_frame}. '
            'Send a Nav2 Goal and the cyan ribbon will appear on the snow.'
        )

    def _resolve_robot_name(self) -> str:
        configured = str(self.get_parameter('robot_name').value).strip().strip('/')
        if configured:
            return configured
        namespace = self.get_namespace().strip('/')
        return namespace or 'husky1'

    def _on_global(self, msg: Path) -> None:
        self.global_path = msg
        self.global_pts = _path_xy(msg)

    def _on_local(self, msg: Path) -> None:
        self.local_pts = _path_xy(msg)

    def _on_state(self, msg: String) -> None:
        self.state = msg.data.strip() or 'IDLE'

    def _on_hazards(self, msg: MarkerArray) -> None:
        self.hazards = msg

    def _base_marker(self, mid: int, ns: str, mtype: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = mid
        marker.type = mtype
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        lifetime = float(self.get_parameter('lifetime_sec').value)
        marker.lifetime.sec = int(lifetime)
        marker.lifetime.nanosec = int((lifetime % 1.0) * 1e9)
        return marker

    def _publish(self) -> None:
        key = _state_key(self.state)
        markers = MarkerArray()

        ribbon_pts = _ribbon_triangles(
            self.global_pts,
            half_width=float(self.get_parameter('ribbon_width').value) * 0.5,
            lift=float(self.get_parameter('lift').value),
        )
        if ribbon_pts:
            ribbon = self._base_marker(1, 'path_preview', Marker.TRIANGLE_LIST)
            ribbon.scale.x = ribbon.scale.y = ribbon.scale.z = 1.0
            ribbon.color = _colour(key)
            ribbon.points = ribbon_pts
            markers.markers.append(ribbon)

            centre = self._base_marker(2, 'path_preview', Marker.LINE_STRIP)
            centre.scale.x = 0.08
            centre.color = _colour('center')
            lift = float(self.get_parameter('lift').value) + 0.02
            centre.points = [_point(x, y, z + lift) for x, y, z in self.global_pts]
            markers.markers.append(centre)

            goal = self._base_marker(3, 'path_preview', Marker.SPHERE)
            gx, gy, gz = self.global_pts[-1]
            goal.pose.position.x = gx
            goal.pose.position.y = gy
            goal.pose.position.z = gz + 0.35
            goal.scale.x = goal.scale.y = goal.scale.z = 0.55
            goal.color = _colour(key)
            goal.color.a = 0.95
            markers.markers.append(goal)

        local_pts = _ribbon_triangles(
            self.local_pts,
            half_width=float(self.get_parameter('local_width').value) * 0.5,
            lift=float(self.get_parameter('lift').value) + 0.04,
        )
        if local_pts:
            local = self._base_marker(4, 'path_preview', Marker.TRIANGLE_LIST)
            local.scale.x = local.scale.y = local.scale.z = 1.0
            local.color = _colour('local')
            local.points = local_pts
            markers.markers.append(local)

        text = self._base_marker(5, 'path_preview', Marker.TEXT_VIEW_FACING)
        if self.global_pts:
            x, y, z = self.global_pts[min(2, len(self.global_pts) - 1)]
        else:
            x, y, z = 0.0, 0.0, 0.0
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = z + 1.4
        text.scale.z = 0.45
        text.color = _colour(key)
        text.color.a = 1.0
        text.text = f'{self.robot_name.upper()}  {self.state or "IDLE"}'
        markers.markers.append(text)

        # Pass through hazard markers so one RViz display shows path + hazards.
        for i, src in enumerate(self.hazards.markers):
            copied = Marker()
            copied = src
            if not copied.header.frame_id:
                copied.header.frame_id = self.map_frame
            if copied.id < 100:
                copied.id = 100 + i
            markers.markers.append(copied)

        self.marker_pub.publish(markers)

        if self.global_path is not None:
            styled = Path()
            styled.header.stamp = self.get_clock().now().to_msg()
            styled.header.frame_id = self.global_path.header.frame_id or self.map_frame
            styled.poses = self.global_path.poses
            self.path_pub.publish(styled)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPreviewViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
