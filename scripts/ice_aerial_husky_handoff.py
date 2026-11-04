#!/usr/bin/env python3
"""Quick dual-robot ice demo: Parrot patrol → detect → hover; Husky Nav2 to corridor.

Prereq (both robots, Husky Nav2+SLAM, Parrot manual cmd_vel — no parrot Nav2):
  ros2 launch 41068_ignition_bringup 41068_ignition_dual_ice.launch.py

Wait until Husky map is usable, then:
  ros2 launch 41068_ignition_bringup ice_aerial_husky_handoff.launch.py

Detection for this quick test reads ice ground truth (~/.ros/ice_ground_truth.json)
written by ice_scenario_manager. Replace with RGB/thermal later.
"""

from __future__ import annotations

import json
import math
import sys
import time
from enum import Enum, auto
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    DEPLOY_ROOF_Z,
    DEPLOY_STATION_XY,
    husky_corridor_goal_for_ice_x,
    parrot_full_cable_tour_waypoints,
)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

Waypoint = Tuple[str, float, float, float, float]


def _yaw_from_quat(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def _yaw_err(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class Phase(Enum):
    WAIT_TRUTH = auto()
    PATROL = auto()
    HOVER = auto()
    HUSKY_NAV = auto()
    DONE = auto()


class IceAerialHuskyHandoff(Node):
    def __init__(self) -> None:
        super().__init__('ice_aerial_husky_handoff')

        self.declare_parameter('parrot_namespace', 'parrot1')
        self.declare_parameter('husky_namespace', 'husky1')
        self.declare_parameter('truth_path', '')
        self.declare_parameter('truth_wait_seconds', 120.0)
        self.declare_parameter('detect_range_m', 18.0)
        self.declare_parameter('use_truth_detection', True)
        self.declare_parameter('deploy_x', DEPLOY_STATION_XY[0])
        self.declare_parameter('deploy_y', DEPLOY_STATION_XY[1])
        self.declare_parameter('roof_z', DEPLOY_ROOF_Z)
        self.declare_parameter('pos_tol', 1.0)
        self.declare_parameter('yaw_tol', 0.4)
        self.declare_parameter('hold_seconds', 1.0)
        self.declare_parameter('control_hz', 20.0)

        p_ns = str(self.get_parameter('parrot_namespace').value)
        h_ns = str(self.get_parameter('husky_namespace').value)
        self.map_frame = f'{h_ns}_map'
        self._parrot_odom: Optional[Odometry] = None
        self._phase = Phase.WAIT_TRUTH
        self._truth: Optional[Dict[str, Any]] = None
        self._target_block: Optional[Dict[str, Any]] = None
        self._hover_pose: Optional[Tuple[float, float, float, float]] = None
        self._patrol_index = 0
        self._hold_start: Optional[float] = None
        self._husky_done = False
        self._husky_ok = False
        self._started_at = time.monotonic()

        dx = float(self.get_parameter('deploy_x').value)
        dy = float(self.get_parameter('deploy_y').value)
        rz = float(self.get_parameter('roof_z').value)
        self._patrol: List[Waypoint] = parrot_full_cable_tour_waypoints(
            spawn_xy=(dx, dy))

        self._cmd_pub = self.create_publisher(Twist, f'/{p_ns}/cmd_vel', 10)
        self._marker_pub = self.create_publisher(MarkerArray, f'/{p_ns}/tour_markers', 10)
        self.create_timer(0.5, self._publish_markers)
        self.create_subscription(
            Odometry,
            f'/{p_ns}/odometry',
            self._on_parrot_odom,
            qos_profile_sensor_data,
        )
        self._real_detection: Optional[Dict[str, Any]] = None
        self.create_subscription(
            PoseStamped,
            '/fde/detection',
            self._on_real_detection,
            10,
        )

        self._husky_nav = ActionClient(
            self, NavigateToPose, f'/{h_ns}/navigate_to_pose')
        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)

        self.get_logger().info(
            'Ice aerial→Husky handoff ready. Patrol: %s'
            % [w[0] for w in self._patrol]
        )

    def _truth_file(self) -> Path:
        raw = str(self.get_parameter('truth_path').value).strip()
        if raw:
            return Path(raw).expanduser()
        return Path.home() / '.ros' / 'ice_ground_truth.json'

    def _load_truth(self) -> Optional[Dict[str, Any]]:
        path = self._truth_file()
        if not path.is_file():
            return None
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (json.JSONDecodeError, OSError):
            return None
        blocks = data.get('blocks') or []
        if not blocks:
            return None
        return data

    def _publish_markers(self) -> None:
        if not self._patrol:
            return
        arr = MarkerArray()
        
        frame_id = self._parrot_odom.header.frame_id if self._parrot_odom else "parrot1_odom"
        
        path_marker = Marker()
        path_marker.header.frame_id = frame_id
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "tour_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.1 # line width
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.8
        
        for lbl, x, y, z, yaw in self._patrol:
            p = Point(x=x, y=y, z=z)
            path_marker.points.append(p)
            
        arr.markers.append(path_marker)
        
        for i, (lbl, x, y, z, yaw) in enumerate(self._patrol):
            wp_marker = Marker()
            wp_marker.header.frame_id = frame_id
            wp_marker.header.stamp = path_marker.header.stamp
            wp_marker.ns = "tour_waypoints"
            wp_marker.id = i + 1
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = x
            wp_marker.pose.position.y = y
            wp_marker.pose.position.z = z
            wp_marker.pose.orientation.w = 1.0
            wp_marker.scale.x = 0.5
            wp_marker.scale.y = 0.5
            wp_marker.scale.z = 0.5
            
            if i < self._patrol_index:
                wp_marker.color.r = 0.5
                wp_marker.color.g = 0.5
                wp_marker.color.b = 0.5
                wp_marker.color.a = 0.5
            elif i == self._patrol_index:
                wp_marker.color.r = 1.0
                wp_marker.color.g = 0.0
                wp_marker.color.b = 0.0
                wp_marker.color.a = 1.0
                wp_marker.scale.x = 0.8
                wp_marker.scale.y = 0.8
                wp_marker.scale.z = 0.8
            else:
                wp_marker.color.r = 0.0
                wp_marker.color.g = 0.0
                wp_marker.color.b = 1.0
                wp_marker.color.a = 0.8
            arr.markers.append(wp_marker)
            
        self._marker_pub.publish(arr)


    def _on_parrot_odom(self, msg: Odometry) -> None:
        self._parrot_odom = msg

    def _parrot_pose(self) -> Optional[Tuple[float, float, float, float]]:
        if self._parrot_odom is None:
            return None
        p = self._parrot_odom.pose.pose.position
        q = self._parrot_odom.pose.pose.orientation
        return (p.x, p.y, p.z, _yaw_from_quat(q.z, q.w))

    def _stop_parrot(self) -> None:
        self._cmd_pub.publish(Twist())

    def _fly_toward(
        self, tx: float, ty: float, tz: float, tyaw: float,
    ) -> bool:
        """P-control toward waypoint; return True when held at goal."""
        pose = self._parrot_pose()
        if pose is None:
            return False
        x, y, z, yaw = pose
        ex, ey, ez = tx - x, ty - y, tz - z
        dist = math.sqrt(ex * ex + ey * ey + ez * ez)
        eyaw = _yaw_err(tyaw, yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        ex_b = c * ex + s * ey
        ey_b = -s * ex + c * ey

        cmd = Twist()
        cmd.linear.x = max(-1.2, min(1.2, 0.6 * ex_b))
        cmd.linear.y = max(-1.2, min(1.2, 0.6 * ey_b))
        cmd.linear.z = max(-0.8, min(0.8, 0.8 * ez))
        cmd.angular.z = max(-1.0, min(1.0, 1.2 * eyaw))

        tol = float(self.get_parameter('pos_tol').value)
        ytol = float(self.get_parameter('yaw_tol').value)
        hold_s = float(self.get_parameter('hold_seconds').value)

        if dist < tol and abs(eyaw) < ytol:
            if self._hold_start is None:
                self._hold_start = time.monotonic()
            elif time.monotonic() - self._hold_start >= hold_s:
                self._stop_parrot()
                self._hold_start = None
                return True
            self._stop_parrot()
            return False

        self._hold_start = None
        self._cmd_pub.publish(cmd)
        return False

    def _hover_hold(self) -> None:
        if self._hover_pose is None:
            self._stop_parrot()
            return
        tx, ty, tz, tyaw = self._hover_pose
        self._fly_toward(tx, ty, tz, tyaw)

    def _on_real_detection(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._real_detection = {
            'position': [float(p.x), float(p.y), float(p.z)],
            'span_id': 'span_01',
        }

    def _detect_nearest_block(
        self,
    ) -> Optional[Dict[str, Any]]:
        pose = self._parrot_pose()
        if pose is None:
            return None
            
        x, y, z, _ = pose
        
        # Override detect range to 8.0m so it actually has to fly directly over the ice to trigger
        rng = 8.0

        # Prioritize real perception detection from ice_rgbd_detector
        if self._real_detection is not None:
            rx, ry, _rz = self._real_detection['position']
            dist = math.sqrt((x - rx) ** 2 + (y - ry) ** 2 + (z - _rz) ** 2)
            if dist <= rng:
                return self._real_detection

        if not bool(self.get_parameter('use_truth_detection').value):
            return None
        if self._truth is None:
            return None

        best = None
        best_d = rng
        for block in self._truth.get('blocks', []):
            pos = block.get('position')
            if not isinstance(pos, (list, tuple)) or len(pos) < 3:
                continue
            d = math.sqrt(
                (x - pos[0]) ** 2 + (y - pos[1]) ** 2 + (z - pos[2]) ** 2
            )
            if d < best_d:
                best_d = d
                best = block
        return best

    def _send_husky_goal(self, x: float, y: float, yaw: float) -> None:
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self._husky_nav.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Waiting for Husky navigate_to_pose...')
            return
        self.get_logger().info(
            'Dispatch Husky Nav2 → (%.1f, %.1f) yaw=%.2f in %s'
            % (x, y, yaw, self.map_frame)
        )
        future = self._husky_nav.send_goal_async(goal)
        future.add_done_callback(self._husky_goal_response)

    def _husky_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Husky rejected Nav2 goal.')
            self._husky_done = True
            self._husky_ok = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._husky_result)

    def _husky_result(self, future) -> None:
        status = future.result().status
        self._husky_ok = status == GoalStatus.STATUS_SUCCEEDED
        self._husky_done = True
        self.get_logger().info(
            'Husky Nav2 finished ok=%s status=%s' % (self._husky_ok, status)
        )

    def _finish(self, ok: bool) -> None:
        self._stop_parrot()
        self._phase = Phase.DONE
        summary = {
            'mode': 'ice_aerial_husky_handoff',
            'pass': ok,
            'ice_block': self._target_block.get('id') if self._target_block else None,
            'husky_ok': self._husky_ok,
        }
        self.get_logger().info('HANDOFF SUMMARY %s' % json.dumps(summary))

    def _tick(self) -> None:
        if self._phase == Phase.DONE:
            return

        if self._phase == Phase.WAIT_TRUTH:
            self._truth = self._load_truth()
            wait = float(self.get_parameter('truth_wait_seconds').value)
            if self._truth is not None:
                self.get_logger().info(
                    'Ice truth loaded (%d blocks) from %s'
                    % (len(self._truth['blocks']), self._truth_file())
                )
                self._phase = Phase.PATROL
                return
            if time.monotonic() - self._started_at > wait:
                self.get_logger().error('Timed out waiting for ice truth file.')
                self._finish(False)
            else:
                self.get_logger().info(
                    'Waiting for ice ground truth...',
                    throttle_duration_sec=5.0,
                )
            return

        if self._phase == Phase.PATROL:
            block = self._detect_nearest_block()
            if block is not None:
                self._target_block = block
                pos = block['position']
                self.get_logger().info(
                    'Detected ice at (%.2f, %.2f, %.2f) — hovering, dispatch Husky'
                    % (pos[0], pos[1], pos[2])
                )
                pose = self._parrot_pose()
                if pose is not None:
                    self._hover_pose = pose
                self._phase = Phase.HOVER
                gx, gy, gyaw = husky_corridor_goal_for_ice_x(pos[0])
                self._send_husky_goal(gx, gy, gyaw)
                self._phase = Phase.HUSKY_NAV
                return

            if self._patrol_index >= len(self._patrol):
                self.get_logger().info('Completed full patrol without finding ice.')
                self._finish(True)
                return

            label, tx, ty, tz, tyaw = self._patrol[self._patrol_index]
            if self._fly_toward(tx, ty, tz, tyaw):
                self.get_logger().info('Patrol reached %s.' % label)
                self._patrol_index += 1
            return

        if self._phase == Phase.HUSKY_NAV:
            self._hover_hold()
            if self._husky_done:
                self._finish(self._husky_ok and self._target_block is not None)
            return


def main() -> None:
    rclpy.init()
    node = IceAerialHuskyHandoff()
    try:
        while rclpy.ok() and node._phase != Phase.DONE:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node._stop_parrot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
