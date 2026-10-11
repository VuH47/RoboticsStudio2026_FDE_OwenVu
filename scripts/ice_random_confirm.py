#!/usr/bin/env python3
"""Random-ice confirm mission. Does not replace ice_unified_handoff.

Parrot flies every cable span. Each ice block it comes across — whichever
span the random placer chose — gets a marker. Husky then drives the corridor
and steps toward the lane edge until that block is inside the RGB camera.
After the confirm (or a failed approach), the Parrot continues to the next one.

  ros2 launch 41068_ignition_bringup ice_unified_random_confirm.launch.py
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
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from visualization_msgs.msg import Marker, MarkerArray

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    CABLE_ENVELOPE_Y,
    DEPLOY_STATION_XY,
    inside_grid,
    min_cable_clearance,
    parrot_full_cable_tour_waypoints,
)

# Legacy geometrical frustum check when visual confirmation is disabled.
# Camera: base_link + (0.45, 0, 0.25), pitched up ~20 deg (-0.35 rad).
_CAM_X = 0.45
_CAM_Z = 0.25
_CAM_PITCH_UP = 0.35
_HFOV = 1.309
_VFOV = 2.0 * math.atan(math.tan(_HFOV * 0.5) * (480.0 / 640.0))
_CAM_NEAR = 0.4
_CAM_FAR = 40.0

Waypoint = Tuple[str, float, float, float, float]


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _yaw_err(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def ice_in_husky_camera(
    hx: float, hy: float, hz: float, hyaw: float,
    ix: float, iy: float, iz: float,
) -> bool:
    """True when the ice point projects inside the Husky RGB image."""
    cx = hx + _CAM_X * math.cos(hyaw)
    cy = hy + _CAM_X * math.sin(hyaw)
    cz = hz + _CAM_Z
    dx, dy, dz = ix - cx, iy - cy, iz - cz
    c, s = math.cos(hyaw), math.sin(hyaw)
    ahead = c * dx + s * dy
    left = -s * dx + c * dy
    if ahead < _CAM_NEAR or ahead > _CAM_FAR:
        return False
    if abs(math.atan2(left, ahead)) > _HFOV * 0.5:
        return False
    if abs(math.atan2(dz, ahead) - _CAM_PITCH_UP) > _VFOV * 0.5:
        return False
    return True


class Phase(Enum):
    WAIT_TRUTH = auto()
    PATROL = auto()
    CONFIRM = auto()
    ALIGN = auto()
    INSPECT = auto()
    DONE = auto()


class IceRandomConfirm(Node):
    def __init__(self) -> None:
        super().__init__('ice_random_confirm')
        self.declare_parameter('parrot_namespace', 'parrot1')
        self.declare_parameter('husky_namespace', 'husky1')
        self.declare_parameter('truth_path', '')
        self.declare_parameter('truth_wait_seconds', 180.0)
        self.declare_parameter('detect_range_m', 8.0)
        self.declare_parameter('approach_y', -10.0)
        # One Nav2 order is a short step. An 18 m leg is outside the free
        # lane the lidar has cleared, and NavFn then fails the whole run.
        self.declare_parameter('drive_step_m', 5.0)
        self.declare_parameter('deploy_x', DEPLOY_STATION_XY[0])
        self.declare_parameter('deploy_y', DEPLOY_STATION_XY[1])
        self.declare_parameter('pos_tol', 1.5)
        self.declare_parameter('yaw_tol', 0.45)
        self.declare_parameter('hold_seconds', 0.8)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('require_husky_visual_confirmation', False)
        self.declare_parameter('visual_confirm_timeout_s', 12.0)

        p_ns = str(self.get_parameter('parrot_namespace').value)
        h_ns = str(self.get_parameter('husky_namespace').value)
        self._nav_frame = '%s_map' % h_ns
        self._marker_frame = '%s_odom' % h_ns
        self._approach_y = float(self.get_parameter('approach_y').value)
        self._drive_step = max(3.0, float(self.get_parameter('drive_step_m').value))
        if self._approach_y < CABLE_ENVELOPE_Y:
            self.get_logger().warn(
                'approach_y %.1f crosses the lane edge %.1f; clamping'
                % (self._approach_y, CABLE_ENVELOPE_Y)
            )
            self._approach_y = CABLE_ENVELOPE_Y + 1.0

        self._phase = Phase.WAIT_TRUTH
        self._truth: Optional[Dict[str, Any]] = None
        self._parrot_odom: Optional[Odometry] = None
        self._husky_odom: Optional[Odometry] = None
        self._patrol_index = 0
        self._hold_start: Optional[float] = None
        self._hover: Optional[Tuple[float, float, float, float]] = None
        self._seen: Dict[str, Dict[str, Any]] = {}
        self._active: Optional[Dict[str, Any]] = None
        self._goals: List[Tuple[float, float, float]] = []
        self._goal_inflight = False
        self._goal_rejects = 0
        self._saw_in_camera = False
        self._logged_stale = False
        self._started_at = time.monotonic()
        self._start_wall = time.time()

        self._use_visual = bool(self.get_parameter('require_husky_visual_confirmation').value)
        self._inspection_sequence = 0
        self._inspection_request_id: Optional[str] = None
        self._inspection_started = 0.0
        self._align_target_yaw = 0.0
        self._align_started = 0.0

        dx = float(self.get_parameter('deploy_x').value)
        dy = float(self.get_parameter('deploy_y').value)
        self._patrol: List[Waypoint] = parrot_full_cable_tour_waypoints(
            spawn_xy=(dx, dy))

        self._cmd_pub = self.create_publisher(Twist, '/%s/cmd_vel' % p_ns, 10)
        self._husky_cmd_pub = self.create_publisher(Twist, '/%s/cmd_vel' % h_ns, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/%s/tour_markers' % p_ns, 10)
        self.create_timer(0.5, self._publish_markers)
        self.create_subscription(
            Odometry, '/%s/odometry' % p_ns, self._on_parrot_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/%s/odometry' % h_ns, self._on_husky_odom,
            qos_profile_sensor_data)
        self._map: Optional[OccupancyGrid] = None
        self._husky_nav = ActionClient(
            self, NavigateToPose, '/%s/navigate_to_pose' % h_ns)
        self.create_subscription(
            OccupancyGrid,
            '/%s/map' % h_ns,
            self._on_map,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._visual_request_pub = self.create_publisher(
            String, '/%s/ice_confirm/request' % h_ns, 10)
        self.create_subscription(
            String, '/%s/ice_confirm/result' % h_ns,
            self._on_husky_visual_result, 10)
        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            'Random confirm ready. approach_y=%.1f (lane edge %.1f). Patrol: %s'
            % (self._approach_y, CABLE_ENVELOPE_Y, [w[0] for w in self._patrol])
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
            if path.stat().st_mtime + 1.0 < self._start_wall:
                if not self._logged_stale:
                    self.get_logger().info(
                        'Ignoring ice truth from an earlier run: %s' % path)
                    self._logged_stale = True
                return None
            data = json.loads(path.read_text(encoding='utf-8'))
        except (json.JSONDecodeError, OSError):
            return None
        blocks = data.get('blocks') or []
        if not blocks:
            return None
        return data

    def _on_parrot_odom(self, msg: Odometry) -> None:
        self._parrot_odom = msg

    def _on_husky_odom(self, msg: Odometry) -> None:
        self._husky_odom = msg

    def _parrot_pose(self) -> Optional[Tuple[float, float, float, float]]:
        if self._parrot_odom is None:
            return None
        p = self._parrot_odom.pose.pose.position
        q = self._parrot_odom.pose.pose.orientation
        return (p.x, p.y, p.z, _yaw_from_quat(q.z, q.w))

    def _husky_pose(self) -> Optional[Tuple[float, float, float, float]]:
        if self._husky_odom is None:
            return None
        p = self._husky_odom.pose.pose.position
        q = self._husky_odom.pose.pose.orientation
        return (p.x, p.y, p.z, _yaw_from_quat(q.z, q.w))

    def _span_of(self, x: float, y: float, z: float) -> str:
        _dist, span_id, _cable = min_cable_clearance(x, y, z)
        return span_id

    def _nearest_new_block(self) -> Optional[Dict[str, Any]]:
        pose = self._parrot_pose()
        if pose is None or self._truth is None:
            return None
        x, y, z, _yaw = pose
        rng = float(self.get_parameter('detect_range_m').value)
        best = None
        best_d = rng
        for block in self._truth.get('blocks') or []:
            key = str(block.get('id') or block.get('position'))
            if key in self._seen:
                continue
            pos = block.get('position')
            if not isinstance(pos, (list, tuple)) or len(pos) < 3:
                continue
            d = math.sqrt(
                (x - float(pos[0])) ** 2
                + (y - float(pos[1])) ** 2
                + (z - float(pos[2])) ** 2
            )
            if d < best_d:
                best_d = d
                best = block
        return best

    def _note_detection(self, block: Dict[str, Any]) -> Dict[str, Any]:
        pos = block['position']
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        key = str(block.get('id') or pos)
        found = {
            'key': key,
            'position': (x, y, z),
            'span': self._span_of(x, y, z),
            'state': 'seen',
        }
        self._seen[key] = found
        self.get_logger().info(
            'Parrot detected ice %s on %s at (%.1f, %.1f, %.1f)'
            % (key, found['span'], x, y, z)
        )
        return found

    def _push_steps(
        self,
        goals: List[Tuple[float, float, float]],
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        yaw: float,
    ) -> None:
        """Split one straight leg into steps no longer than drive_step_m."""
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist < 1.0:
            return
        n = int(math.ceil(dist / self._drive_step))
        for i in range(1, n + 1):
            t = i / float(n)
            goals.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))

    def _goals_for(self, ice_x: float) -> List[Tuple[float, float, float]]:
        """Short Nav2 steps from where the Husky is to the ice, then to the lane.

        A single 18 m order is the goal the 10:00 planner aborted as off the
        map. Each order is at most drive_step_m (5 m). Already out on the
        lane stays on that y: a return to y=0 is the U-turn that spun the map.
        """
        pose = self._husky_pose()
        hx = pose[0] if pose is not None else 0.0
        hy = pose[1] if pose is not None else 0.0
        face = -math.pi / 2.0
        out_on_lane = hy < -2.0
        lane_y = self._approach_y if out_on_lane else 0.0
        drive_yaw = 0.0 if ice_x >= hx else math.pi
        goals: List[Tuple[float, float, float]] = []
        self._push_steps(goals, hx, hy, ice_x, lane_y, drive_yaw)
        if not self._use_visual:
            target_face = -math.pi / 2.0
            if self._active and 'position' in self._active:
                ix, iy = self._active['position'][:2]
                target_face = math.atan2(iy - self._approach_y, ix - ice_x)
            if goals:
                x, y, _yaw = goals[-1]
                goals[-1] = (x, y, target_face)
            else:
                goals.append((ice_x, self._approach_y, target_face))
        else:
            if not goals:
                goals.append((ice_x, lane_y, drive_yaw))
        self.get_logger().info(
            'Husky short steps (%.0f m): %s'
            % (self._drive_step, ', '.join('(%.1f, %.1f)' % (g[0], g[1]) for g in goals))
        )
        return goals

    def _publish_markers(self) -> None:
        if not self._patrol:
            return
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        path = Marker()
        path.header.frame_id = self._marker_frame
        path.header.stamp = now
        path.ns = 'tour_path'
        path.id = 0
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = 0.1
        path.color.g = 1.0
        path.color.a = 0.8
        for _lbl, x, y, z, _yaw in self._patrol:
            path.points.append(Point(x=x, y=y, z=z))
        arr.markers.append(path)

        for i, found in enumerate(self._seen.values()):
            x, y, z = found['position']
            sphere = Marker()
            sphere.header.frame_id = self._marker_frame
            sphere.header.stamp = now
            sphere.ns = 'tour_waypoints'
            sphere.id = 100 + i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.2
            if found['state'] == 'confirmed':
                sphere.color.g = 1.0
            elif found['state'] == 'missed':
                sphere.color.r = sphere.color.g = sphere.color.b = 0.45
            else:
                sphere.color.r = sphere.color.g = 1.0
            sphere.color.a = 0.95
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self._marker_frame
            text.header.stamp = now
            text.ns = 'tour_waypoints'
            text.id = 200 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + 1.6
            text.pose.orientation.w = 1.0
            text.scale.z = 0.8
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = '%s %s' % (found['span'], found['state'])
            arr.markers.append(text)
        self._marker_pub.publish(arr)

    def _stop_parrot(self) -> None:
        self._cmd_pub.publish(Twist())

    def _fly_toward(self, tx: float, ty: float, tz: float, tyaw: float) -> bool:
        pose = self._parrot_pose()
        if pose is None:
            return False
        x, y, z, yaw = pose
        ex, ey, ez = tx - x, ty - y, tz - z
        dist = math.sqrt(ex * ex + ey * ey + ez * ez)
        eyaw = _yaw_err(tyaw, yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        cmd = Twist()
        cmd.linear.x = max(-1.2, min(1.2, 0.6 * (c * ex + s * ey)))
        cmd.linear.y = max(-1.2, min(1.2, 0.6 * (-s * ex + c * ey)))
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

    def _camera_sees_active(self) -> bool:
        if self._active is None:
            return False
        pose = self._husky_pose()
        if pose is None:
            return False
        ix, iy, iz = self._active['position']
        return ice_in_husky_camera(pose[0], pose[1], pose[2], pose[3], ix, iy, iz)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _goal_on_map(self, x: float, y: float) -> bool:
        grid = self._map
        if grid is None:
            return False
        info = grid.info
        return inside_grid(
            info.origin.position.x,
            info.origin.position.y,
            info.resolution,
            info.width,
            info.height,
            x,
            y,
            1.0,
        )

    def _stop_husky(self) -> None:
        self._husky_cmd_pub.publish(Twist())

    def _prepare_visual_inspection(self) -> None:
        if not self._use_visual or self._active is None:
            return
        pose = self._husky_pose()
        if pose is None:
            self._start_visual_inspection()
            return
        hx, hy, _hz, hyaw = pose
        ix, iy, _iz = self._active['position']
        target_yaw = math.atan2(iy - hy, ix - hx)
        err = _yaw_err(target_yaw, hyaw)
        if abs(err) > 0.15:
            self._align_target_yaw = target_yaw
            self._align_started = time.monotonic()
            self._phase = Phase.ALIGN
            self.get_logger().info(
                'Husky aiming toward ice (%.1f, %.1f): target_yaw=%.2f, current=%.2f (err=%.2f rad)'
                % (ix, iy, target_yaw, hyaw, err))
        else:
            self._stop_husky()
            self._start_visual_inspection()

    def _send_next_goal(self) -> None:
        if self._goal_inflight:
            return
        if not self._goals:
            if self._use_visual:
                self._prepare_visual_inspection()
            else:
                self._finish_confirm()
            return
        if not self._husky_nav.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                'Waiting for Husky navigate_to_pose...',
                throttle_duration_sec=5.0)
            return
        x, y, yaw = self._goals[0]
        if not self._goal_on_map(x, y):
            self.get_logger().warn(
                'Holding Husky goal (%.1f, %.1f): the SLAM map does not cover it yet.'
                % (x, y),
                throttle_duration_sec=5.0,
            )
            return
        _qx, _qy, qz, qw = _yaw_to_quat(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._nav_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self._goal_inflight = True
        self.get_logger().info(
            'Husky approach → (%.1f, %.1f) yaw=%.2f' % (x, y, yaw))
        future = self._husky_nav.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._goal_inflight = False
            self.get_logger().error('Nav2 goal send failed: %s' % exc)
            self._finish_confirm('navigation_failed' if self._use_visual else None)
            return
        if handle is None or not handle.accepted:
            self._goal_inflight = False
            self._goal_rejects += 1
            if self._goal_rejects >= 8:
                self.get_logger().error('Husky kept rejecting the approach.')
                self._finish_confirm('navigation_failed' if self._use_visual else None)
            return
        self._goal_rejects = 0
        if self._goals:
            self._goals.pop(0)
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._goal_inflight = False
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error('Nav2 result failed: %s' % exc)
            self._finish_confirm('navigation_failed' if self._use_visual else None)
            return
        ok = status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            self.get_logger().warn(
                'Husky approach segment failed (status %s).' % status)
            self._finish_confirm('navigation_failed' if self._use_visual else None)
            return
        if self._use_visual:
            if self._goals:
                self._send_next_goal()
            else:
                self._prepare_visual_inspection()
            return
        # Legacy Mission 2 geometrical frustum check
        if self._camera_sees_active():
            self._saw_in_camera = True
            self.get_logger().info('Ice is inside the Husky camera.')
        if self._goals:
            self._send_next_goal()
        else:
            self._finish_confirm()

    def _start_visual_inspection(self) -> None:
        if not self._use_visual or self._phase not in (Phase.CONFIRM, Phase.ALIGN) or self._active is None:
            return
        self._stop_husky()
        self._inspection_sequence += 1
        self._inspection_request_id = '%s_%d' % (
            self._active['key'], self._inspection_sequence)
        self._inspection_started = time.monotonic()
        self._phase = Phase.INSPECT
        request = {
            'request_id': self._inspection_request_id,
            'frame_id': self._marker_frame,
            'position': list(self._active['position']),
        }
        self._visual_request_pub.publish(String(data=json.dumps(request)))
        self.get_logger().info('Husky RGB inspection requested: %s' % self._inspection_request_id)

    def _on_husky_visual_result(self, msg: String) -> None:
        if not self._use_visual or self._phase != Phase.INSPECT or self._active is None:
            return
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn('Ignoring malformed Husky visual result.')
            return
        if payload.get('request_id') != self._inspection_request_id:
            return  # stale result from a different hazard
        status = payload.get('status')
        if status not in ('CONFIRMED', 'NOT_CONFIRMED', 'INCONCLUSIVE'):
            self.get_logger().warn('Ignoring unknown Husky visual status %s' % status)
            return
        self.get_logger().info(
            'Husky REAL IMAGE verdict: %s, pixel-positive frames=%s/%s, evidence=%s'
            % (status, payload.get('positive_frames'), payload.get('valid_frames'),
               payload.get('image_path')))
        self._active['visual_evidence'] = payload
        self._finish_confirm({
            'CONFIRMED': 'confirmed',
            'NOT_CONFIRMED': 'not_confirmed',
            'INCONCLUSIVE': 'inconclusive',
        }[status])

    def _hold_parrot(self) -> None:
        """Hover where the Parrot was when the Husky was sent, or stop."""
        if self._hover is not None:
            self._fly_toward(*self._hover)
        else:
            self._stop_parrot()

    def _finish_confirm(self, outcome: Optional[str] = None) -> None:
        if self._phase not in (Phase.CONFIRM, Phase.ALIGN, Phase.INSPECT) or self._active is None:
            return
        self._stop_husky()
        if self._use_visual:
            self._active['state'] = outcome or 'inconclusive'
        else:
            self._active['state'] = 'confirmed' if self._saw_in_camera else 'missed'
        self.get_logger().info(
            '%s %s' % (self._active['span'], self._active['state']))
        self._active = None
        self._goals = []
        self._goal_inflight = False
        self._saw_in_camera = False
        self._inspection_request_id = None
        self._hover = None
        self._phase = Phase.PATROL

    def _tick(self) -> None:
        if self._phase == Phase.DONE:
            self._stop_parrot()
            self._stop_husky()
            return
        if self._phase == Phase.WAIT_TRUTH:
            self._truth = self._load_truth()
            if self._truth is None:
                if time.monotonic() - self._started_at > float(
                    self.get_parameter('truth_wait_seconds').value
                ):
                    self.get_logger().error('Timed out waiting for random ice truth.')
                    self._phase = Phase.DONE
                return
            if self._parrot_pose() is None:
                return
            if not hasattr(self, '_warmup_start'):
                self._warmup_start = time.monotonic()
                n = len(self._truth.get('blocks') or [])
                self.get_logger().info(
                    'Random ice loaded (%d blocks). Warming up.' % n)
            if time.monotonic() - self._warmup_start < 5.0:
                return
            self._phase = Phase.PATROL
            return

        if self._phase == Phase.PATROL:
            block = self._nearest_new_block()
            if block is not None:
                self._active = self._note_detection(block)
                pose = self._parrot_pose()
                if pose is not None:
                    self._hover = pose
                ix = self._active['position'][0]
                self._goals = self._goals_for(ix)
                self._saw_in_camera = False
                self._goal_rejects = 0
                self._phase = Phase.CONFIRM
                self._send_next_goal()
                return
            if self._patrol_index >= len(self._patrol):
                pending = [f for f in self._seen.values() if f['state'] == 'seen']
                if not pending:
                    self.get_logger().info(
                        'Patrol finished. confirmed=%d not_confirmed=%d inconclusive=%d missed=%d'
                        % (
                            sum(1 for f in self._seen.values() if f['state'] == 'confirmed'),
                            sum(1 for f in self._seen.values() if f['state'] == 'not_confirmed'),
                            sum(1 for f in self._seen.values() if f['state'] == 'inconclusive'),
                            sum(1 for f in self._seen.values() if f['state'] in ('missed', 'navigation_failed')),
                        ))
                    self._phase = Phase.DONE
                return
            label, tx, ty, tz, tyaw = self._patrol[self._patrol_index]
            if self._fly_toward(tx, ty, tz, tyaw):
                self.get_logger().info('Patrol reached %s.' % label)
                self._patrol_index += 1
            return

        if self._phase == Phase.CONFIRM:
            self._hold_parrot()
            if not self._use_visual and self._camera_sees_active():
                self._saw_in_camera = True
            self._send_next_goal()
            return

        if self._phase == Phase.ALIGN:
            self._hold_parrot()
            pose = self._husky_pose()
            if pose is None or time.monotonic() - self._align_started > 8.0:
                self._stop_husky()
                self._start_visual_inspection()
                return
            err = _yaw_err(self._align_target_yaw, pose[3])
            if abs(err) <= 0.15:
                self._stop_husky()
                self.get_logger().info('Husky aligned with ice target (err=%.2f rad). Starting inspection.' % err)
                self._start_visual_inspection()
                return
            cmd = Twist()
            cmd.angular.z = max(-1.0, min(1.0, 1.8 * err))
            if abs(cmd.angular.z) < 0.3:
                cmd.angular.z = 0.3 if err > 0 else -0.3
            self._husky_cmd_pub.publish(cmd)
            return

        if self._phase == Phase.INSPECT:
            self._hold_parrot()
            if time.monotonic() - self._inspection_started > float(
                self.get_parameter('visual_confirm_timeout_s').value
            ):
                self.get_logger().warn('Husky visual confirmation timed out.')
                self._finish_confirm('inconclusive')
            return


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = IceRandomConfirm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
