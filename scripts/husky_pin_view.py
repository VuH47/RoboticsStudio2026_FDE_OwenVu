#!/usr/bin/env python3
"""Parrot pins ice. Husky drives itself to a viewing pose.

The drone publishes a pin on /fde/detection. The Husky walks 4 m steps to a
viewing pose. A blocked step is shifted to the first free cell north of it
on the Nav2 costmap. Nav2 plans the curve between those free cells.
Ice height is known (the cable), so the pin's z is not used.

  ros2 launch 41068_ignition_bringup ice_pin_view.launch.py
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

# Horizontal standoff. The cable is about 8 m up and the Husky camera looks
# up about 20 degrees, so 22 m of ground range puts that height dead-centre
# in view while keeping Husky 2-3 m clear of the south SLAM map boundary.
# The pin's z is not part of this.
# Option B empirical corridor: open snow tested between y = -22.1 m and y = -6.6 m.
# Nominal standoff 16.0 m places the view stand at y ≈ -14.7 m, in the center of the open gap.
VIEW_STANDOFF_M = 16.0
# Closest the Husky may stand to the cable (tested down to -22.1 m), and the north edge of the lane.
LANE_NEAR_CABLE_Y = -22.0
LANE_FAR_Y = 2.0


def viewing_pose(
    pin_x: float,
    pin_y: float,
    standoff: float = VIEW_STANDOFF_M,
    y_near: float = LANE_NEAR_CABLE_Y,
    y_far: float = LANE_FAR_Y,
) -> Tuple[float, float, float, float]:
    """Return (x, y, yaw, horizontal_distance). z is not an input.

    The stand is standoff metres toward +Y from the pin (the corridor side),
    then clamped into the lane. Yaw faces the pin. The distance is the
    horizontal gap the camera is allowed to look across.
    """
    view_y = float(pin_y) + float(standoff)
    if view_y < y_near:
        view_y = y_near
    if view_y > y_far:
        view_y = y_far
    dist = math.hypot(0.0, float(pin_y) - view_y)
    yaw = math.atan2(float(pin_y) - view_y, 0.0)
    return (float(pin_x), view_y, yaw, dist)


# Issue 30/31: one long Nav2 goal is off the SLAM grid until the Husky has
# driven there. worldToMap then fails and the robot never moves. Four metre
# steps stay inside the 10x10 m local costmap and cleared SLAM corridor.
DRIVE_STEP_M = 4.0


def drive_steps(
    hx: float,
    hy: float,
    vx: float,
    vy: float,
    yaw: float,
    step: float = DRIVE_STEP_M,
) -> List[Tuple[float, float, float]]:
    """Straight steps from the Husky to the viewing pose.

    The last step keeps the viewing heading. Earlier steps face the
    direction of travel. Nav2 plans each short piece itself.
    """
    dist = math.hypot(vx - hx, vy - hy)
    if dist < 1.0:
        return [(vx, vy, yaw)]
    n = max(1, int(math.ceil(dist / max(step, 1.0))))
    travel = math.atan2(vy - hy, vx - hx)
    out = []
    for i in range(1, n + 1):
        t = i / float(n)
        out.append((
            hx + (vx - hx) * t,
            hy + (vy - hy) * t,
            yaw if i == n else travel,
        ))
    return out


def aim_yaw(hx: float, hy: float, pin_x: float, pin_y: float) -> float:
    """Heading from Husky's actual stand (hx, hy) directly toward the pin."""
    return math.atan2(float(pin_y) - float(hy), float(pin_x) - float(hx))


def world_to_map(
    wx: float,
    wy: float,
    wyaw: float,
    husky_world: Optional[Tuple[float, float, float, float]],
    husky_map: Optional[Tuple[float, float, float]],
) -> Tuple[float, float, float]:
    """Transform a world-frame step (wx, wy, wyaw) into Nav2's husky1_map frame.

    Uses T_{map<-base} * T_{world<-base}^{-1} so that a step at distance d and
    relative bearing alpha from Husky in Gazebo world is placed at the exact
    same distance d and relative bearing alpha from Husky in husky1_map,
    eliminating wheel-slip and SLAM-yaw step jumps.
    """
    if husky_world is None or husky_map is None:
        return (float(wx), float(wy), float(wyaw))
    hw_x, hw_y, _hw_z, hw_yaw = husky_world
    hm_x, hm_y, hm_yaw = husky_map
    dtheta = hm_yaw - hw_yaw
    c, s = math.cos(dtheta), math.sin(dtheta)
    dx, dy = float(wx) - hw_x, float(wy) - hw_y
    mx = hm_x + c * dx - s * dy
    my = hm_y + s * dx + c * dy
    myaw = math.atan2(math.sin(float(wyaw) + dtheta), math.cos(float(wyaw) + dtheta))
    return (mx, my, myaw)


# Deploy-roof spawn used by ice_pin_view (dual-ice parrot_x/y/z). The tour
# stays here until the sim clock is moving, the pose has sat in this ball,
# and this run's placer says the blocks are on the cables.
ROOF_SPAWN_XYZ = (-8.0, 0.0, 4.0)
ROOF_HOLD_S = 2.0
ROOF_RADIUS_M = 1.5


def pose_near_roof(
    pose: Optional[Tuple[float, float, float]],
    roof: Tuple[float, float, float] = ROOF_SPAWN_XYZ,
    radius: float = ROOF_RADIUS_M,
) -> bool:
    """True when pose is inside the roof ball. Origin and a missing pose are not."""
    if pose is None:
        return False
    return math.hypot(
        float(pose[0]) - roof[0],
        float(pose[1]) - roof[1],
        float(pose[2]) - roof[2],
    ) <= float(radius)


def ice_place_succeeded(text: str) -> bool:
    """True only for a full place this run, such as 'placed 4/4 seed=1 ...'."""
    raw = str(text).strip()
    if 'not updated' in raw:
        return False
    parts = raw.split()
    if len(parts) < 2 or parts[0] != 'placed':
        return False
    counts = parts[1].split('/')
    if len(counts) != 2 or not counts[0].isdigit() or not counts[1].isdigit():
        return False
    done = int(counts[0])
    total = int(counts[1])
    return total > 0 and done == total


class TourGate:
    """Clock and roof-hold memory for the pure start check."""

    def __init__(self) -> None:
        self.prev_sim_s: Optional[float] = None
        self.steady_since_s: Optional[float] = None


def tour_may_start(
    gate: TourGate,
    sim_s: Optional[float],
    pose: Optional[Tuple[float, float, float]],
    placed: bool,
    hold_s: float = ROOF_HOLD_S,
) -> bool:
    """True when the clock just advanced, the roof hold has elapsed, and ice is placed.

    A missing pose or a pose outside the roof ball clears the hold. The first
    clock sample only records the time, so a single stamp cannot release the tour.
    """
    clock_moving = (
        sim_s is not None
        and gate.prev_sim_s is not None
        and float(sim_s) > float(gate.prev_sim_s)
    )
    if sim_s is not None:
        gate.prev_sim_s = float(sim_s)
    if sim_s is None or not pose_near_roof(pose):
        gate.steady_since_s = None
    elif gate.steady_since_s is None:
        gate.steady_since_s = float(sim_s)
    held = (
        gate.steady_since_s is not None
        and sim_s is not None
        and (float(sim_s) - gate.steady_since_s) >= float(hold_s)
    )
    return bool(clock_moving and held and placed)


# Matches general_goal_checker xy_goal_tolerance. A crumb inside this
# radius is already "Reached the goal" and must not be sent again.
XY_GOAL_TOLERANCE_M = 1.0
# A succeeded goal that did not move the body is the same false arrival.
NO_MOTION_M = 0.15


def step_action(
    hx: float,
    hy: float,
    stand_xy: Tuple[float, float],
    moved_m: Optional[float] = None,
    goal_radius: float = XY_GOAL_TOLERANCE_M,
    no_motion_m: float = NO_MOTION_M,
) -> str:
    """Return 'align' or 'send'.

    'align' does not build a NavigateToPose goal. stand_xy is the cell
    classify_step returned, including a north free cell when the view
    line itself is blocked.

    Before a send, omit moved_m: inside the 1 m goal radius is already
    arrived. After a success, pass moved_m: only a body that stayed
    put is a false arrival. A real drive ends inside that same radius
    and must still be allowed to take the next crumb.
    """
    if moved_m is not None:
        if float(moved_m) < float(no_motion_m):
            return 'align'
        return 'send'
    sx, sy = float(stand_xy[0]), float(stand_xy[1])
    if math.hypot(float(hx) - sx, float(hy) - sy) <= float(goal_radius):
        return 'align'
    return 'send'


def arrival_xy(
    view_xy: Tuple[float, float],
    sent_xy: Optional[Tuple[float, float]],
) -> Tuple[float, float]:
    """The 1.5 m and 2.5 m gates use the cell that was sent, once one exists."""
    if sent_xy is not None:
        return (float(sent_xy[0]), float(sent_xy[1]))
    return (float(view_xy[0]), float(view_xy[1]))


def _yaw_err(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def needs_reverse_before_turn(
    hx: float,
    hy: float,
    hyaw: float,
    target_yaw: float,
    cost_at,
    min_front_clear: float = 1.3,
) -> bool:
    """True if forward obstacle clearance is insufficient for safe in-place corner swing.

    The Husky's 1.0 m x 1.0 m footprint sweeps an outer radius of R ≈ 0.71 m.
    If obstacles are within min_front_clear ahead and an in-place turn is required,
    reversing first opens the envelope to avoid corner clipping.
    """
    err = _yaw_err(target_yaw, hyaw)
    if abs(err) <= 0.15:
        return False
    angles = (float(hyaw) - 0.25, float(hyaw), float(hyaw) + 0.25)
    for r in (0.6, 0.9, 1.1, float(min_front_clear)):
        for ang in angles:
            cx = float(hx) + r * math.cos(ang)
            cy = float(hy) + r * math.sin(ang)
            c = cost_at(cx, cy)
            if c is not None and c not in (0, 255) and c >= 1:
                return True
    return False


import json
import time
from enum import Enum, auto

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from husky_detour import (
    blocked_notice, choose_stand, classify_step, detour_notice, grid_cost,
    near_sight_clear, pose_past_block, pose_retreat, sight_clear,
)
from ice_cable_geometry import (
    DEPLOY_STATION_XY, inside_grid, parrot_full_cable_tour_waypoints,
)
from ice_target_tracker import (
    ACTIVE, CONFIRMED, FINISHED, MISSED, NOT_CONFIRMED, READY, TRACKING,
    TargetTracker,
)


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _yaw_to_quat(yaw: float) -> Tuple[float, float]:
    return (math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class Phase(Enum):
    FLY = auto()
    NAV = auto()
    ALIGN = auto()
    INSPECT = auto()
    DONE = auto()


class HuskyPinView(Node):
    def __init__(self) -> None:
        super().__init__('husky_pin_view')
        self.declare_parameter('standoff_m', VIEW_STANDOFF_M)
        self.declare_parameter('y_near', LANE_NEAR_CABLE_Y)
        self.declare_parameter('y_far', LANE_FAR_Y)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('merge_radius_m', 3.0)
        self.declare_parameter('pos_tol', 1.5)
        self.declare_parameter('yaw_tol', 0.45)
        self.declare_parameter('hold_seconds', 0.8)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('inspect_timeout_s', 12.0)
        self.declare_parameter('drive_step_m', DRIVE_STEP_M)
        self.declare_parameter('detour_hold_s', 8.0)
        self.declare_parameter('costmap_topic', '/husky1/global_costmap/costmap')
        self.declare_parameter(
            'local_costmap_topic', '/husky1/local_costmap/costmap')
        self._standoff = float(self.get_parameter('standoff_m').value)
        self._y_near = float(self.get_parameter('y_near').value)
        self._y_far = float(self.get_parameter('y_far').value)
        self._tracker = TargetTracker(
            merge_radius=float(self.get_parameter('merge_radius_m').value),
            min_hits=int(self.get_parameter('min_hits').value))
        self._patrol = parrot_full_cable_tour_waypoints(spawn_xy=DEPLOY_STATION_XY)
        self._patrol_index = 0
        self._hold_start: Optional[float] = None
        self._phase = Phase.FLY
        self._parrot: Optional[Tuple[float, float, float, float]] = None
        self._husky: Optional[Tuple[float, float, float, float]] = None
        self._goal_inflight = False
        self._steps: List[Tuple[float, float, float]] = []
        self._held_log = False
        self._hold_key: Optional[str] = None
        self._block_hold_start: Optional[float] = None
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._local_costmap: Optional[OccupancyGrid] = None
        self._failed_goals: List[Tuple[float, float]] = []
        self._last_goal_world: Optional[Tuple[float, float]] = None
        self._active: Optional[dict] = None
        self._request_id: Optional[str] = None
        self._inspect_started = 0.0
        self._seq = 0
        self._primary_id: Optional[str] = None
        self._retry_after = 0.0
        self._align_yaw = -math.pi / 2.0
        self._align_started = 0.0
        self._align_backing: bool = False
        self._align_backing_start: float = 0.0
        self._sonar_min_dist: float = 99.0
        self._corridor_notice: Optional[str] = None
        self._gates_passed: set = set()
        self._pin_retries: dict = {}
        self._tour_gate = TourGate()
        self._goal_from_xy: Optional[Tuple[float, float]] = None
        self._stand_align_logged = False
        self._tour_released = False
        self._tour_wait_logged = False
        self._ice_placed = False
        self._place_fail_logged = False
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cmd = self.create_publisher(Twist, '/parrot1/cmd_vel', 10)
        self._husky_cmd = self.create_publisher(Twist, '/husky1/cmd_vel', 10)
        self._pins = self.create_publisher(MarkerArray, '/parrot1/tour_markers', 10)
        self._targets = self.create_publisher(String, '/fde/ice_targets', 10)
        self._ask = self.create_publisher(String, '/husky1/ice_confirm/request', 10)
        self._nav = ActionClient(self, NavigateToPose, '/husky1/navigate_to_pose')
        self.create_subscription(
            Odometry, '/parrot1/odometry', self._on_parrot, qos_profile_sensor_data)
        place_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/ice/placed', self._on_ice_placed, place_qos)
        self.create_subscription(
            Odometry, '/husky1/odometry', self._on_husky, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/fde/detection', self._on_pin, 10)
        self.create_subscription(
            String, '/husky1/ice_confirm/result', self._on_result, 10)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, '/husky1/map', self._on_map, map_qos)
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('costmap_topic').value),
            self._on_costmap, map_qos)
        # Local costmap is volatile (rolling window) — not transient_local.
        local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('local_costmap_topic').value),
            self._on_local_costmap, local_qos)
        self.create_subscription(
            LaserScan, '/husky1/sonar', self._on_sonar, qos_profile_sensor_data)
        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.create_timer(0.5, self._publish_pins)
        self.get_logger().info(
            'Pin view ready. Standoff %.0f m (z ignored). Lane y in [%.0f, %.0f].'
            % (self._standoff, self._y_near, self._y_far))

    def _on_parrot(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._parrot = (p.x, p.y, p.z, _yaw_from_quat(q.z, q.w))

    def _on_ice_placed(self, msg: String) -> None:
        if ice_place_succeeded(msg.data):
            self._ice_placed = True
            return
        if (str(msg.data).strip() == 'placement_failed'
                and not self._place_fail_logged):
            self._place_fail_logged = True
            self.get_logger().error(
                'Ice placement failed. Parrot staying on the roof.')

    def _sim_time_s(self) -> Optional[float]:
        stamp = self.get_clock().now().nanoseconds
        if stamp <= 0:
            return None
        return float(stamp) * 1e-9

    def _on_husky(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._husky = (p.x, p.y, p.z, _yaw_from_quat(q.z, q.w))

    def _husky_in_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                'husky1_map', 'husky1_base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (float(t.x), float(t.y), _yaw_from_quat(float(q.z), float(q.w)))

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._local_costmap = msg

    def _cost_at_world(self, x: float, y: float) -> Optional[int]:
        """Merged Nav2 cost: local (odom) + global (map).

        Local blocked wins — that is what the lidar sees right now. Goals
        Nav2 just refused are treated as lethal so classify_step detours.
        """
        for fx, fy in self._failed_goals:
            if math.hypot(float(x) - fx, float(y) - fy) < 1.0:
                return 254
        local_c: Optional[int] = None
        if self._local_costmap is not None:
            info = self._local_costmap.info
            local_c = grid_cost(
                info.origin.position.x, info.origin.position.y, info.resolution,
                info.width, info.height, self._local_costmap.data,
                float(x), float(y))
        global_c: Optional[int] = None
        if self._husky_in_map() is not None and self._costmap is not None:
            mx, my, _myaw = world_to_map(
                x, y, 0.0, self._husky, self._husky_in_map())
            info = self._costmap.info
            global_c = grid_cost(
                info.origin.position.x, info.origin.position.y, info.resolution,
                info.width, info.height, self._costmap.data, mx, my)
        # Prefer the more conservative reading.
        for cost in (local_c, global_c):
            if cost is None:
                continue
            if int(cost) not in (0, 255) and int(cost) >= 1:
                return int(cost)
        for cost in (local_c, global_c):
            if cost is not None and int(cost) == 0:
                return 0
        if local_c is None and global_c is None:
            return None
        return 255

    def _on_sonar(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self._sonar_min_dist = min(valid)
        else:
            self._sonar_min_dist = 99.0

    def _forward_obstacle_near(
        self,
        hx: float,
        hy: float,
        hyaw: float,
        min_clearance: float = 1.3,
    ) -> bool:
        """True if an obstacle or high cost is detected within min_clearance ahead."""
        if self._sonar_min_dist < min(min_clearance, 1.0):
            return True
        angles = (hyaw - 0.25, hyaw, hyaw + 0.25)
        for r in (0.6, 0.9, 1.1, float(min_clearance)):
            for ang in angles:
                cx = hx + r * math.cos(ang)
                cy = hy + r * math.sin(ang)
                c = self._cost_at_world(cx, cy)
                if c is not None and c not in (0, 255) and c >= 1:
                    return True
        return False

    def _rear_obstacle_near(
        self,
        hx: float,
        hy: float,
        hyaw: float,
        min_clearance: float = 0.8,
    ) -> bool:
        """True if an obstacle is detected within min_clearance behind the robot."""
        rear_yaw = hyaw + math.pi
        angles = (rear_yaw - 0.25, rear_yaw, rear_yaw + 0.25)
        for r in (0.6, float(min_clearance)):
            for ang in angles:
                cx = hx + r * math.cos(ang)
                cy = hy + r * math.sin(ang)
                c = self._cost_at_world(cx, cy)
                if c is not None and c not in (0, 255) and c >= 1:
                    return True
        return False

    def _note_hold(self, key: str, text: str) -> None:
        if key == self._hold_key:
            return
        self._hold_key = key
        self.get_logger().info(text)

    def _skip_pin(self, reason: str) -> None:
        self._husky_cmd.publish(Twist())
        active = self._active
        self._steps = []
        self._active = None
        self._block_hold_start = None
        self._hold_key = None
        self._failed_goals = []
        self._last_goal_world = None
        self._phase = Phase.FLY
        if active is None:
            return
        if self._husky is not None:
            self._set_corridor(blocked_notice(self._husky[0]))
        self._tracker.set_state(active['track'], MISSED, 'missed')
        self.get_logger().info('Pin %s skipped. %s' % (active['track'], reason))

    def _goal_on_map(self, x: float, y: float) -> bool:
        grid = self._map
        if grid is None:
            return False
        info = grid.info
        return inside_grid(
            info.origin.position.x, info.origin.position.y, info.resolution,
            info.width, info.height, x, y, 1.0)

    def _on_pin(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        before = len(self._tracker.tracks)
        tr = self._tracker.add_hit(time.monotonic(), float(p.x), float(p.y), float(p.z))
        if len(self._tracker.tracks) > before:
            if self._primary_id is None:
                self._primary_id = tr.id
                self.get_logger().info(
                    'First pin %s stays tracked until Husky has looked at it.'
                    % tr.id)
            self.get_logger().info(
                'Ice pin %s at (%.1f, %.1f). z %.1f is not used for the drive.'
                % (tr.id, p.x, p.y, p.z))

    def _on_result(self, msg: String) -> None:
        if self._phase != Phase.INSPECT or self._active is None:
            return
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if payload.get('request_id') != self._request_id:
            return
        status = payload.get('status')
        tid = self._active['track']
        if status == 'CONFIRMED':
            self._tracker.set_state(tid, CONFIRMED, 'confirmed')
            if payload.get('priority'):
                self._tracker.set_priority(
                    tid, str(payload['priority']), payload.get('priority_reason'))
            self.get_logger().info(
                'Husky saw the reported pin %s: %s %s'
                % (tid, payload.get('priority', ''), payload.get('priority_reason', '')))
        else:
            self._tracker.set_state(tid, NOT_CONFIRMED, 'not_confirmed')
            self.get_logger().info('Husky did not see pin %s (%s).' % (tid, status))
        self._active = None
        self._phase = Phase.FLY

    def _fly(self) -> None:
        if not self._tour_released:
            self._cmd.publish(Twist())
            pose = None if self._parrot is None else self._parrot[:3]
            if tour_may_start(
                    self._tour_gate, self._sim_time_s(), pose, self._ice_placed):
                self._tour_released = True
                self.get_logger().info(
                    'Gazebo pose is steady and ice is placed. '
                    'Starting the cable tour.')
            else:
                if not self._tour_wait_logged:
                    self.get_logger().info(
                        'Waiting for Gazebo and ice placement.')
                    self._tour_wait_logged = True
                return
        if self._patrol_index >= len(self._patrol):
            self._cmd.publish(Twist())
            return
        _label, tx, ty, tz, tyaw = self._patrol[self._patrol_index]
        if self._parrot is None:
            return
        x, y, z, yaw = self._parrot
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
        if dist < tol and abs(eyaw) < ytol:
            if self._hold_start is None:
                self._hold_start = time.monotonic()
            elif time.monotonic() - self._hold_start >= float(
                    self.get_parameter('hold_seconds').value):
                self._cmd.publish(Twist())
                self._hold_start = None
                self.get_logger().info('Patrol reached %s.' % _label)
                self._patrol_index += 1
                return
            self._cmd.publish(Twist())
            return
        self._hold_start = None
        self._cmd.publish(cmd)

    def _next_pin(self):
        """First pin is mandatory. Later pins are chosen only after it is done."""
        if self._primary_id is not None:
            first = self._tracker.get(self._primary_id)
            if first is not None and first.state not in FINISHED:
                if first.state in (READY, ACTIVE) or (
                        first.state == TRACKING and first.n_hits >= self._tracker.min_hits):
                    return first
                return None
        origin = (self._husky[0], self._husky[1]) if self._husky else (0.0, 0.0)
        return self._tracker.pick_next(origin, self._y_near, final=False)

    def _view_goal(self) -> bool:
        if self._phase != Phase.FLY or self._goal_inflight:
            return False
        if time.monotonic() < self._retry_after:
            return False
        origin = (self._husky[0], self._husky[1]) if self._husky else (0.0, 0.0)
        tr = self._next_pin()
        if tr is None:
            return False
        x, y, z = tr.position
        vx, vy, yaw, dist = viewing_pose(
            x, y, self._standoff, self._y_near, self._y_far)
        retries = self._pin_retries.get(tr.id, 0)
        if retries > 0:
            vy = min(vy + 1.0 * retries, -5.5)
            dist = abs(y - vy)
        hx, hy = origin
        step = float(self.get_parameter('drive_step_m').value)
        straight = drive_steps(hx, hy, vx, vy, yaw, step)
        self._steps = straight
        self._held_log = False
        self._hold_key = None
        self._block_hold_start = None
        self._failed_goals = []
        self._last_goal_world = None
        self._goal_from_xy = None
        self._stand_align_logged = False
        self._active = {
            'track': tr.id,
            'pin': (x, y, z),
            'view': (vx, vy, yaw),
            'start_dist': math.hypot(hx - vx, hy - vy),
        }
        self._tracker.set_state(tr.id, ACTIVE)
        self.get_logger().info(
            'Pin (%.1f, %.1f). View from (%.1f, %.1f) yaw=%.2f, '
            'horizontal distance %.1f m. %d short steps.'
            % (x, y, vx, vy, yaw, dist, len(self._steps)))
        self._phase = Phase.NAV
        self._send_next()
        return True

    def _send_next(self) -> None:
        if self._goal_inflight:
            return
        if self._husky is not None and self._active is not None:
            vx, vy, _vyaw = self._active['view']
            ax, ay = arrival_xy((vx, vy), self._active.get('stand'))
            if math.hypot(self._husky[0] - ax, self._husky[1] - ay) <= 1.5:
                self._steps = []
                self._commit_or_shift_stand(ax, ay)
                return
        if self._husky is None or self._active is None:
            return
        vx, vy, view_yaw = self._active['view']
        hx, hy = self._husky[0], self._husky[1]
        step = float(self.get_parameter('drive_step_m').value)
        steps = drive_steps(hx, hy, vx, vy, view_yaw, step)
        while (len(steps) > 1
               and math.hypot(steps[0][0] - hx, steps[0][1] - hy) <= 1.2):
            steps.pop(0)
        x, y, yaw = steps[0]
        if self._husky_in_map() is None:
            self._block_hold_start = None
            self._note_hold(
                'no_tf',
                'Holding step (%.1f, %.1f): husky1_map is not ready.' % (x, y))
            return
        mx, my, _myaw = world_to_map(x, y, yaw, self._husky, self._husky_in_map())
        if not self._goal_on_map(mx, my):
            self._block_hold_start = None
            self._note_hold(
                'off_map',
                'Holding step (%.1f, %.1f): the SLAM map does not cover it yet.'
                % (x, y))
            return
        decision, pose = classify_step(
            x, y, yaw, self._cost_at_world, from_xy=(hx, hy), goal_y=vy)
        # Near unknown is sent so the map grows. Far unknown waits. Blocked
        # with no free lane cell holds then skips after detour_hold_s.
        if decision == 'hold_unknown' or (
                pose is None and decision != 'hold_blocked'):
            self._block_hold_start = None
            self._note_hold(
                'hold_unknown',
                'hold_unknown step (%.1f, %.1f): waiting for costmap.'
                % (x, y))
            return
        if pose is None:
            step_m = float(self.get_parameter('drive_step_m').value)
            past = pose_past_block(
                (vx, vy), (x, y), self._cost_at_world, step=step_m,
                from_xy=(hx, hy))
            backing = False
            if past is None:
                past = pose_retreat((hx, hy), (vx, vy), self._cost_at_world)
                backing = past is not None
            if past is not None:
                self._block_hold_start = None
                self._hold_key = None
                psx, psy, pyaw = past
                if backing:
                    self.get_logger().info(
                        'Lane blocked at (%.1f, %.1f). Backing off to '
                        '(%.1f, %.1f).' % (x, y, psx, psy))
                else:
                    self.get_logger().info(
                        'Lane blocked at (%.1f, %.1f). Continuing past it '
                        'to (%.1f, %.1f).' % (x, y, psx, psy))
                self._dispatch_step(hx, hy, psx, psy, pyaw)
                return
            now = time.monotonic()
            if self._block_hold_start is None:
                self._block_hold_start = now
            limit = float(self.get_parameter('detour_hold_s').value)
            self._note_hold(
                'hold_blocked',
                'hold_blocked step (%.1f, %.1f): no free cell in the lane.'
                % (x, y))
            if now - self._block_hold_start >= limit:
                self._skip_pin('no free cell in the lane for %.0f s' % limit)
            return
        # Issue 56: Fix timer reset bug during stand inspection.
        # OLD CODE:
        #   self._block_hold_start = None  # Wiped unconditionally on every tick
        #   ...
        #   if step_action(hx, hy, (sx, sy)) == 'align':
        #       self._commit_or_shift_stand(sx, sy)
        # OLD BEHAVIOUR:
        #   When the robot arrived at a detour stand, pose was not None, so line 814
        #   cleared self._block_hold_start = None at 10 Hz. When _commit_or_shift_stand()
        #   attempted to hold because sight was occluded, the timer was wiped on the next
        #   tick before detour_hold_s (8.0 s) could ever elapse, causing an infinite stall.
        # NEW BEHAVIOUR:
        #   Only reset the hold timer when a new driving step is actually dispatched
        #   (_dispatch_step) or the stand is accepted (_accept_stand).
        sx, sy, syaw = pose
        if decision == 'detour':
            self.get_logger().info(
                'detour (%.1f, %.1f) -> free cell (%.1f, %.1f).'
                % (x, y, sx, sy),
                throttle_duration_sec=2.0)
        else:
            self.get_logger().info(
                '%s step (%.1f, %.1f).' % (decision, sx, sy),
                throttle_duration_sec=2.0)
        if step_action(hx, hy, (sx, sy)) == 'align':
            self._commit_or_shift_stand(sx, sy)
            return
        self._block_hold_start = None
        self._hold_key = None
        self._dispatch_step(hx, hy, sx, sy, syaw)

    def _dispatch_step(
        self, hx: float, hy: float, sx: float, sy: float, syaw: float,
    ) -> None:
        if math.hypot(sx - hx, sy - hy) > 0.2:
            nav_yaw = math.atan2(sy - hy, sx - hx)
        else:
            nav_yaw = syaw
        self._last_goal_world = (sx, sy)
        self._goal_from_xy = (hx, hy)
        gx, gy, gyaw = world_to_map(
            sx, sy, nav_yaw, self._husky, self._husky_in_map())
        self._send(gx, gy, gyaw)

    def _commit_or_shift_stand(self, sx: float, sy: float) -> None:
        """Align only when the costmap ray to the pin is clear and camera is unoccluded.

        Otherwise move the view to the nearest free lane cell that can see
        the pin, and keep driving. Nothing here names an obstacle.
        """
        if self._active is None:
            return
        px, py, _pz = self._active['pin']
        vx, vy, vyaw = self._active['view']
        stand_aim = aim_yaw(sx, sy, px, py)
        if (sight_clear(sx, sy, px, py, self._cost_at_world)
                and near_sight_clear(sx, sy, stand_aim, self._cost_at_world)):
            self._accept_stand(sx, sy)
            return
        alt = choose_stand(vx, vy, vyaw, px, py, self._cost_at_world)
        if alt is None:
            self._note_hold(
                'no_view',
                'No free lane cell can see pin %s. Holding.'
                % self._active['track'])
            now = time.monotonic()
            if self._block_hold_start is None:
                self._block_hold_start = now
            limit = float(self.get_parameter('detour_hold_s').value)
            if now - self._block_hold_start >= limit:
                self._skip_pin('no viewpoint with a clear ray to the pin')
            return
        self._block_hold_start = None
        self._active['view'] = alt
        self._active.pop('stand', None)
        self.get_logger().info(
            'Stand (%.1f, %.1f) cannot see pin or camera is occluded. Moving the stand '
            'to (%.1f, %.1f).' % (sx, sy, alt[0], alt[1]))

    def _accept_stand(self, sx: float, sy: float) -> None:
        """The free cell is already the stand. Do not send it to Nav2."""
        self._block_hold_start = None
        self._hold_key = None
        if self._active is not None:
            self._active['stand'] = (float(sx), float(sy))
        self._steps = []
        self._last_goal_world = None
        self._goal_from_xy = None
        if not self._stand_align_logged:
            self._stand_align_logged = True
            self.get_logger().info(
                'Already within 1.0 m of free cell (%.1f, %.1f). '
                'Aligning from the detour stand.' % (sx, sy))
        self._start_align()

    def _send(self, x: float, y: float, yaw: float) -> None:
        if not self._nav.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                'Waiting for Husky navigate_to_pose...', throttle_duration_sec=5.0)
            return
        qz, qw = _yaw_to_quat(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'husky1_map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self._goal_inflight = True
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self._on_accept)

    def _on_accept(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._goal_inflight = False
            self.get_logger().error('Nav2 goal send failed: %s' % exc)
            self._give_up()
            return
        if handle is None or not handle.accepted:
            self._goal_inflight = False
            self.get_logger().warn('Nav2 rejected the step.')
            self._give_up()
            return
        handle.get_result_async().add_done_callback(self._on_nav)

    def _on_nav(self, future) -> None:
        self._goal_inflight = False
        try:
            ok = future.result().status == GoalStatus.STATUS_SUCCEEDED
        except Exception as exc:
            self.get_logger().error('Nav2 result failed: %s' % exc)
            self._give_up()
            return
        if self._active is None:
            self._give_up()
            return
        if not ok:
            if self._husky is not None:
                vx, vy, _vyaw = self._active['view']
                ax, ay = arrival_xy((vx, vy), self._active.get('stand'))
                rem = math.hypot(self._husky[0] - ax, self._husky[1] - ay)
                if rem <= 2.5:
                    self.get_logger().info(
                        'Within %.2f m of viewing pose; aligning toward pin.'
                        % rem)
                    self._steps = []
                    self._start_align()
                    return
            self.get_logger().warn('Short step failed.')
            if self._last_goal_world is not None:
                fx, fy = self._last_goal_world
                self._failed_goals.append((fx, fy))
                if len(self._failed_goals) > 12:
                    self._failed_goals = self._failed_goals[-12:]
                self.get_logger().info(
                    'Marking refused crumb (%.1f, %.1f) blocked; trying again.'
                    % (fx, fy))
                self._last_goal_world = None
                self._send_next()
                return
            self._give_up()
            return
        if (self._husky is not None and self._last_goal_world is not None
                and self._goal_from_xy is not None):
            moved = math.hypot(
                self._husky[0] - self._goal_from_xy[0],
                self._husky[1] - self._goal_from_xy[1])
            if step_action(
                    self._husky[0], self._husky[1], self._last_goal_world,
                    moved_m=moved) == 'align':
                sx, sy = self._last_goal_world
                self._accept_stand(sx, sy)
                return
        self._send_next()

    def _start_align(self) -> None:
        if self._active is None:
            return
        px, py, _pz = self._active['pin']
        if self._husky is None:
            self._start_inspect()
            return
        hx, hy, _hz, hyaw = self._husky
        target_yaw = aim_yaw(hx, hy, px, py)
        err = _yaw_err(target_yaw, hyaw)
        if abs(err) <= 0.15:
            self._husky_cmd.publish(Twist())
            self._start_inspect()
            return
        self._align_yaw = target_yaw
        self._align_started = time.monotonic()
        self._phase = Phase.ALIGN

        # Phase 6: Check forward clearance before in-place rotation
        if self._forward_obstacle_near(hx, hy, hyaw, min_clearance=1.3):
            if not self._rear_obstacle_near(hx, hy, hyaw, min_clearance=0.7):
                self._align_backing = True
                self._align_backing_start = time.monotonic()
                self.get_logger().info(
                    'Forward clearance < 1.3 m at stand (%.1f, %.1f). '
                    'Reversing before turn to open corner envelope.' % (hx, hy))
            else:
                self._align_backing = False
                self.get_logger().warn(
                    'Forward obstacle detected, but rear is also constrained. '
                    'Proceeding with cautious turn.')
        else:
            self._align_backing = False

        self.get_logger().info(
            'At viewing stand (%.1f, %.1f). Aiming Husky toward pin %s '
            '(target_yaw=%.2f, current=%.2f, err=%.2f rad).'
            % (hx, hy, self._active['track'], target_yaw, hyaw, err))

    def _align(self) -> None:
        if self._active is None:
            self._husky_cmd.publish(Twist())
            self._phase = Phase.FLY
            return
        if self._husky is None or time.monotonic() - self._align_started > 10.0:
            self._husky_cmd.publish(Twist())
            self._start_inspect()
            return
        hx, hy, _hz, hyaw = self._husky

        # Phase 6: Reverse-before-turn backup execution
        if self._align_backing:
            backing_time = time.monotonic() - self._align_backing_start
            forward_clear = not self._forward_obstacle_near(hx, hy, hyaw, min_clearance=1.3)
            rear_blocked = self._rear_obstacle_near(hx, hy, hyaw, min_clearance=0.7)
            if backing_time >= 1.8 or forward_clear or rear_blocked:
                self._align_backing = False
                self._husky_cmd.publish(Twist())
                self.get_logger().info(
                    'Pre-turn reverse completed (elapsed %.1f s). Commencing aim_yaw rotation.'
                    % backing_time)
            else:
                cmd = Twist()
                cmd.linear.x = -0.15
                self._husky_cmd.publish(cmd)
                return

        px, py, _pz = self._active['pin']
        self._align_yaw = aim_yaw(hx, hy, px, py)
        err = _yaw_err(self._align_yaw, hyaw)
        if abs(err) <= 0.15:
            self._husky_cmd.publish(Twist())
            self.get_logger().info(
                'Husky aligned with pin %s (err=%.2f rad).'
                % (self._active['track'], err))
            self._start_inspect()
            return

        # Safeguard: if front sonar detects an obstacle point-blank during turn, pause turn and back up
        if self._sonar_min_dist < 0.60:
            cmd = Twist()
            cmd.linear.x = -0.10
            self._husky_cmd.publish(cmd)
            return

        cmd = Twist()
        cmd.angular.z = max(-1.0, min(1.0, 1.8 * err))
        if abs(cmd.angular.z) < 0.40:
            cmd.angular.z = 0.40 if err > 0.0 else -0.40
        self._husky_cmd.publish(cmd)

    def _start_inspect(self) -> None:
        if self._active is None:
            return
        self._husky_cmd.publish(Twist())
        self._seq += 1
        self._request_id = 'pin_%s_%d' % (self._active['track'], self._seq)
        x, y, z = self._active['pin']
        req = {
            'request_id': self._request_id,
            'frame_id': 'husky1_odom',
            'position': [x, y, z],
        }
        self._ask.publish(String(data=json.dumps(req)))
        self._inspect_started = time.monotonic()
        self._phase = Phase.INSPECT
        self.get_logger().info('At the viewing pose. Asking Husky to look at the pin.')

    def _set_corridor(self, text: str) -> None:
        if not text or text == self._corridor_notice:
            return
        self._corridor_notice = text
        self.get_logger().info(text)

    def _watch_rock_gates(self) -> None:
        if self._phase not in (Phase.NAV, Phase.ALIGN) or self._husky is None:
            return
        notice = detour_notice(self._husky[0], self._husky[1])
        if notice is None or notice in self._gates_passed:
            return
        self._gates_passed.add(notice)
        self._set_corridor(notice)

    def _give_up(self) -> None:
        self._husky_cmd.publish(Twist())
        if self._active is None:
            self._steps = []
            self._phase = Phase.FLY
            return
        tid = self._active['track']
        if self._husky is not None and 'view' in self._active:
            vx, vy, _vyaw = self._active['view']
            now_dist = math.hypot(self._husky[0] - vx, self._husky[1] - vy)
            start_dist = float(self._active.get('start_dist', now_dist))
            if start_dist - now_dist >= 1.5:
                self._pin_retries[tid] = 0
                self._tracker.set_state(tid, READY)
                self._retry_after = time.monotonic() + 0.5
                self.get_logger().info(
                    'Husky advanced %.1f m toward pin %s (%.1f m left); '
                    'replanning remaining steps.'
                    % (start_dist - now_dist, tid, now_dist))
                self._steps = []
                self._active = None
                self._phase = Phase.FLY
                return
        retries = self._pin_retries.get(tid, 0) + 1
        self._pin_retries[tid] = retries
        if tid == self._primary_id or retries <= 3:
            self._tracker.set_state(tid, READY)
            self._retry_after = time.monotonic() + 2.0
            self.get_logger().info(
                'Still tracking pin %s (attempt %d). Nav2 missed a step; retrying.'
                % (tid, retries))
        else:
            if self._husky is not None:
                self._set_corridor(blocked_notice(self._husky[0]))
            self._tracker.set_state(tid, MISSED, 'missed')
            self.get_logger().info(
                'Later pin %s skipped after Nav2 could not plan.' % tid)
        self._steps = []
        self._active = None
        self._phase = Phase.FLY

    def _publish_pins(self) -> None:
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        path = Marker()
        path.header.frame_id = 'husky1_odom'
        path.header.stamp = now
        path.ns = 'tour_path'
        path.id = 0
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = 0.1
        path.color.g = 1.0
        path.color.a = 0.8
        from geometry_msgs.msg import Point
        for _lbl, x, y, z, _yaw in self._patrol:
            path.points.append(Point(x=x, y=y, z=z))
        arr.markers.append(path)
        for i, tr in enumerate(self._tracker.tracks):
            x, y, _z = tr.position
            vx, vy, yaw, _dist = viewing_pose(
                x, y, self._standoff, self._y_near, self._y_far)
            pin = Marker()
            pin.header.frame_id = 'husky1_odom'
            pin.header.stamp = now
            pin.ns = 'tour_waypoints'
            pin.id = 100 + i
            pin.type = Marker.SPHERE
            pin.action = Marker.ADD
            pin.pose.position.x = x
            pin.pose.position.y = y
            pin.pose.position.z = 8.0
            pin.pose.orientation.w = 1.0
            pin.scale.x = pin.scale.y = pin.scale.z = 1.2
            pin.color.r = pin.color.g = 1.0
            pin.color.a = 0.95
            arr.markers.append(pin)
            text = Marker()
            text.header.frame_id = 'husky1_odom'
            text.header.stamp = now
            text.ns = 'tour_waypoints'
            text.id = 200 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 9.6
            text.pose.orientation.w = 1.0
            text.scale.z = 0.8
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = '%s pin view (%.0f, %.0f)' % (tr.id, vx, vy)
            arr.markers.append(text)
            del yaw
        self._pins.publish(arr)
        data = {
            'parrot': {
                'patrol_index': self._patrol_index,
                'patrol_total': len(self._patrol),
                'patrol_done': self._patrol_index >= len(self._patrol),
            },
            'husky': {
                'phase': self._phase.name,
                'active': None if self._active is None else self._active['track'],
                'corridor': self._corridor_notice,
            },
            'targets': [],
        }
        origin = (self._husky[0], self._husky[1]) if self._husky else (0.0, 0.0)
        for t in self._tracker.snapshot(origin, self._y_near):
            data['targets'].append(t)
        self._targets.publish(String(data=json.dumps(data)))

    def _tick(self) -> None:
        self._watch_rock_gates()
        if self._phase == Phase.DONE:
            self._cmd.publish(Twist())
            self._husky_cmd.publish(Twist())
            return
        self._fly()
        if self._phase == Phase.NAV:
            self._send_next()
            return
        if self._phase == Phase.ALIGN:
            self._align()
            return
        if self._phase == Phase.FLY:
            self._view_goal()
            if (self._patrol_index >= len(self._patrol)
                    and self._tracker.unfinished_count() == 0
                    and self._active is None):
                self.get_logger().info(
                    'Pins done. %s' % [
                        (t.id, t.state) for t in self._tracker.tracks])
                self._phase = Phase.DONE
            return
        if self._phase == Phase.INSPECT:
            limit = float(self.get_parameter('inspect_timeout_s').value)
            if time.monotonic() - self._inspect_started > limit:
                self.get_logger().warn('Husky look timed out.')
                self._give_up()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HuskyPinView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
