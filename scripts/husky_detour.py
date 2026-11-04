#!/usr/bin/env python3
"""Path around a blocked corridor step. No ROS.

Nav2 is asked to drive a pose. When the straight step crosses a rock, this
builds another path to that same pose and the same final heading. Intermediate
corners face the direction of travel so a differential-drive Husky is not
told to strafe. The last pose is the pose that was requested.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

Pose = Tuple[float, float, float]  # x, y, yaw
Gate = Dict[str, float]

# Rock half-size plus inflation (1.0 m) and half the footprint (0.5 m).
# A straight step that enters this box is blocked. The open-side pass sits
# further out, on gate['open_y'].
PASS_MARGIN = 1.5

# Offline fixture only (tests / confirm notices). Live pin-view reads the
# global costmap and does not use these centres. V2 snow_forest.sdf:
# gate A bush (22, 1), gate B (56, -0.8), lane (36, -9), (46, -8.5), (52, -11).
ROCK_GATES: Tuple[Gate, ...] = (
    {'x': 22.0, 'y': 1.0, 'half': 1.2, 'open_y': -4.0, 'south': 1.0},
    {'x': 36.0, 'y': -9.0, 'half': 1.2, 'open_y': -3.0, 'south': 0.0},
    {'x': 46.0, 'y': -8.5, 'half': 1.2, 'open_y': -3.0, 'south': 0.0},
    {'x': 52.0, 'y': -11.0, 'half': 1.2, 'open_y': -3.0, 'south': 0.0},
    {'x': 56.0, 'y': -0.8, 'half': 1.2, 'open_y': 4.0, 'south': 0.0},
)


def detour_notice(x: float, y: float) -> Optional[str]:
    """Announce a gate once the robot has left y=0 on that gate's open side."""
    for gate in ROCK_GATES:
        if abs(x - gate['x']) > 8.0:
            continue
        if gate['south'] and -8.0 < y < -1.5:
            return 'rock gate at x=%.0f, passing south' % gate['x']
        if not gate['south'] and 1.5 < y < 8.0:
            return 'rock gate at x=%.0f, passing north' % gate['x']
    return None


def blocked_notice(x: Optional[float]) -> str:
    if x is None:
        return 'corridor blocked'
    for gate in ROCK_GATES:
        if abs(x - gate['x']) <= 8.0:
            return 'corridor blocked at rock gate x=%.0f' % gate['x']
    return 'corridor blocked'


def _travel_yaw(x0: float, y0: float, x1: float, y1: float, fallback: float) -> float:
    if math.hypot(x1 - x0, y1 - y0) < 0.2:
        return fallback
    return math.atan2(y1 - y0, x1 - x0)


def _inside(x: float, y: float, gate: Gate, margin: float) -> bool:
    half = float(gate['half']) + margin
    return (abs(x - gate['x']) <= half) and (abs(y - gate['y']) <= half)


def segment_blocked(
    x0: float, y0: float, x1: float, y1: float, gate: Gate, margin: float,
) -> bool:
    dist = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(math.ceil(dist)))
    for i in range(steps + 1):
        t = i / float(steps)
        if _inside(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, gate, margin):
            return True
    return False


def _append_leg(
    out: List[Pose],
    x0: float, y0: float, x1: float, y1: float,
    yaw: float, step: float, final: bool,
) -> None:
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist < 0.05:
        return
    travel = _travel_yaw(x0, y0, x1, y1, yaw)
    n = 1 if dist < 1.0 else int(math.ceil(dist / step))
    for i in range(1, n + 1):
        t = i / float(n)
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        out.append((x, y, yaw if (final and i == n) else travel))


def _detour_to(
    x0: float, y0: float, goal: Pose,
    gates: Sequence[Gate], step: float, margin: float,
) -> List[Pose]:
    gx, gy, gyaw = goal
    crossed = [g for g in gates if segment_blocked(x0, y0, gx, gy, g, margin)]
    if not crossed:
        out: List[Pose] = []
        _append_leg(out, x0, y0, gx, gy, gyaw, step, True)
        return out

    dx, dy = gx - x0, gy - y0
    length2 = dx * dx + dy * dy or 1.0

    def progress(gate: Gate) -> float:
        return ((gate['x'] - x0) * dx + (gate['y'] - y0) * dy) / length2

    crossed.sort(key=progress)
    corners: List[Tuple[float, float]] = []
    cx, cy = x0, y0
    for gate in crossed:
        direction = 1.0 if gx >= cx else -1.0
        clear = float(gate['half']) + margin
        before = gate['x'] - direction * clear
        after = gate['x'] + direction * clear
        side = float(gate['open_y'])
        south_edge = float(gate['y']) - float(gate['half'])
        if cy < south_edge and gy < south_edge and side > south_edge:
            side = south_edge - margin - 0.5
        if (before - cx) * direction > 0.5:
            corners.append((before, cy))
            corners.append((before, side))
        else:
            corners.append((cx, side))
        corners.append((after, side))
        cx, cy = after, side
    # Come back to the lane only at cx, which is already past the rock.
    # A crumb still beside the rock, such as (33.5, 0) after the south pass,
    # is not appended: the leg back to it is the turn into the mesh.
    # The pose we do keep has the requested heading.
    corners.append((cx, gy))
    if not any(_inside(gx, gy, gate, margin) for gate in crossed):
        corners.append((gx, gy))

    out = []
    sx, sy = x0, y0
    last = len(corners) - 1
    for i, (px, py) in enumerate(corners):
        _append_leg(out, sx, sy, px, py, gyaw, step, i == last)
        sx, sy = px, py
    return out


def plan_around(
    start: Tuple[float, float],
    goals: Sequence[Pose],
    gates: Sequence[Gate],
    step: float = 5.0,
    margin: float = PASS_MARGIN,
) -> List[Pose]:
    """Rebuild goals so a blocked straight step goes around.

    Each original goal is still the end of its piece, with the same yaw.
    """
    cx, cy = float(start[0]), float(start[1])
    out: List[Pose] = []
    for goal in goals:
        piece = _detour_to(cx, cy, goal, gates, step, margin)
        out.extend(piece)
        if piece:
            cx, cy = piece[-1][0], piece[-1][1]
        else:
            cx, cy = goal[0], goal[1]
    return out


def crosses_any(
    x0: float, y0: float, x1: float, y1: float,
    gates: Sequence[Gate], margin: float = PASS_MARGIN,
) -> Optional[Gate]:
    for gate in gates:
        if segment_blocked(x0, y0, x1, y1, gate, margin):
            return gate
    return None


# Live pin-view detour (Costmap2D bands):
#   0            free
#   1 .. 254     blocked (occupied, inscribed, inflated)
#   255 / None   unknown (straight step may still be sent; NavFn allow_unknown)
# Detour pass lines require a free (0) cell. Pass order is north from the
# drive line. Nothing south of y = -11.
FREE_COST = 0
UNKNOWN_COST = 255
SEGMENT_SAMPLE_M = 0.5
# Option B empirical corridor: open snow tested from y = -22.1 m up to y = -6.6 m.
# Detour pass lines include open southern snow corridor and northern bypasses.
LANE_PASS_YS: Tuple[float, ...] = (
    -22.0, -20.0, -18.0, -16.0, -14.0, -12.0, -10.0, -8.0,
    -5.5, -5.0, -4.5, -4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0.0,
)
LANE_Y_MIN = -24.0
LANE_Y_MAX = 2.0
# Only the near frontier may be sent while unknown. Farther unknown cells
# wait so the Husky does not drive into rocks the laser has not painted yet
# on the global costmap (Issue 52 live hit).
UNKNOWN_SEND_M = 4.5
# Stand search along the lane, nearest first. Expanded for Option B wide baseline.
SIGHT_X_OFFSETS_M: Tuple[float, ...] = (
    0.0, 2.0, -2.0, 4.0, -4.0, 6.0, -6.0, 8.0, -8.0, 10.0, -10.0, 12.0, -12.0,
)


def grid_cost(
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    data: Sequence[int],
    x: float,
    y: float,
) -> Optional[int]:
    """Cost at a world or map point. None when the point is off the grid."""
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return None
    mx = int(math.floor((float(x) - float(origin_x)) / float(resolution)))
    my = int(math.floor((float(y) - float(origin_y)) / float(resolution)))
    if mx < 0 or my < 0 or mx >= int(width) or my >= int(height):
        return None
    return _as_cost(data[my * int(width) + mx])


def _as_cost(value: int) -> int:
    """Nav2 writes the grid as int8. -1 is unknown. A wrapped raw cost is restored."""
    value = int(value)
    if value == -1:
        return UNKNOWN_COST
    if value < 0:
        return value + 256
    return value


def _unknown_cost(cost: Optional[int]) -> bool:
    if cost is None:
        return True
    value = int(cost)
    return value < 0 or value == UNKNOWN_COST


def _cell_state(cost: Optional[int]) -> str:
    """'free', 'unknown', or 'blocked'."""
    if _unknown_cost(cost):
        return 'unknown'
    if int(cost) == FREE_COST:
        return 'free'
    return 'blocked'


# Issue 56 & Option B: Differentiate driving traversability vs camera optical visibility.
# In Nav2 Costmap2D (raw internal):
#   0             free space
#   1 .. 252      inflated buffer around obstacles (repulsion for wheels)
#   253 .. 254    inscribed and lethal obstacles (rock/bush meshes, bumper contacts)
#   255           unknown space
# In ROS 2 OccupancyGrid (published topic data from Costmap2DPublisher):
#   0             free space
#   1 .. 98       inflated buffer around obstacles
#   99            inscribed inflated obstacle
#   100           lethal obstacle
#   -1 / 255      unknown space
#
# Camera optical line-of-sight must only be blocked by physical lethal/inscribed obstacles
# (cost 99..100 in ROS 2 OccupancyGrid, or 253..254 in raw Costmap2D).
# Repulsive inflation halos (1..98 and 1..252) represent open air and do not block vision.
OCCUPIED_COST = 253


def _camera_cell_state(cost: Optional[int]) -> str:
    """'free', 'unknown', or 'blocked' for optical line of sight.

    Only physical obstacle meshes (cost in 99..100 for ROS OccupancyGrid,
    or >= 253 for raw Costmap2D) block light. Repulsive inflation halos
    (1..98 and 1..252) represent open air and do not block vision.
    """
    if _unknown_cost(cost):
        return 'unknown'
    c = int(cost)
    if (99 <= c <= 100) or (253 <= c <= 254):
        return 'blocked'
    return 'free'


def _sample_segment(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    spacing: float = SEGMENT_SAMPLE_M,
) -> List[Tuple[float, float]]:
    """Points from just after the start through the end, about every spacing m."""
    dist = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
    if dist < 1e-6:
        return [(float(x1), float(y1))]
    n = max(1, int(math.ceil(dist / max(float(spacing), 0.1))))
    out: List[Tuple[float, float]] = []
    for i in range(1, n + 1):
        t = i / float(n)
        out.append((
            float(x0) + (float(x1) - float(x0)) * t,
            float(y0) + (float(y1) - float(y0)) * t,
        ))
    return out


def _segment_blocked(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    cost_at,
    skip_m: float = 0.0,
    state_fn=_cell_state,
) -> bool:
    """True when any sample on the segment is a known blocked cost.

    skip_m ignores samples next to the start so a robot already sitting in
    inflation can still leave, while a rock one metre ahead still blocks.
    state_fn defaults to _cell_state for driving (cost >= 1). For camera
    sight lines, state_fn=_camera_cell_state is passed (cost >= OCCUPIED_COST).
    """
    for sx, sy in _sample_segment(x0, y0, x1, y1):
        if float(skip_m) > 0.0 and math.hypot(sx - float(x0), sy - float(y0)) < float(skip_m):
            continue
        if state_fn(cost_at(sx, sy)) == 'blocked':
            return True
    return False


def _approach_blocked(x0, y0, x1, y1, cost_at) -> bool:
    """Occupied cell on the drive, ignoring the footprint under the robot.

    Samples every 0.25 m. Blocks when any sample enters lethal or inscribed
    obstacle space (cost in 99..100 for ROS OccupancyGrid, or >= 253 for raw
    Costmap2D). Outer repulsive inflation halos (1..98 / 1..252) do not block
    the approach as Nav2 smoothly plans the trajectory through them.
    """
    for sx, sy in _sample_segment(x0, y0, x1, y1, spacing=0.25):
        if math.hypot(sx - float(x0), sy - float(y0)) < 0.8:
            continue
        c = cost_at(sx, sy)
        if c is not None and not _unknown_cost(c):
            val = int(c)
            if (99 <= val <= 100) or (253 <= val <= 254):
                return True
    return False


def _pass_candidates(
    y: float,
    pass_ys: Sequence[float],
    y_min: float,
    y_max: float,
    goal_y: Optional[float] = None,
) -> List[float]:
    """Straight y first, then candidate pass lines sorted by proximity to goal_y (or y if no goal).

    Both south (closer to cable) and north lines are evaluated, prioritizing lines
    toward the destination stand so Husky moves into the open snow corridor rather than
    being forced into narrow obstacle channels.
    """
    chosen: List[float] = []

    def add(value: float) -> None:
        if value < y_min or value > y_max:
            return
        for have in chosen:
            if abs(have - value) < 0.05:
                return
        chosen.append(value)

    add(float(y))
    target = float(y) if goal_y is None else float(goal_y)
    sorted_pass = sorted(
        pass_ys,
        key=lambda py: (abs(float(py) - target), abs(float(py) - float(y))),
    )
    for py in sorted_pass:
        add(float(py))
    return chosen


def _stand_candidates(
    y: float,
    pass_ys: Sequence[float],
    y_min: float,
    y_max: float,
) -> List[float]:
    """Straight y first, then candidate stand lines sorted by distance from y (both south and north)."""
    chosen: List[float] = []

    def add(value: float) -> None:
        if value < y_min or value > y_max:
            return
        for have in chosen:
            if abs(have - value) < 0.05:
                return
        chosen.append(value)

    add(float(y))
    sorted_pass = sorted(pass_ys, key=lambda py: abs(float(py) - float(y)))
    for py in sorted_pass:
        add(float(py))
    return chosen



def classify_step(
    x: float,
    y: float,
    yaw: float,
    cost_at,
    pass_ys: Sequence[float] = LANE_PASS_YS,
    y_min: float = LANE_Y_MIN,
    y_max: float = LANE_Y_MAX,
    from_xy: Optional[Tuple[float, float]] = None,
    unknown_send_m: float = UNKNOWN_SEND_M,
    goal_y: Optional[float] = None,
) -> Tuple[str, Optional[Pose]]:
    """Decide the pose Nav2 may be given at this x.

    Returns one of:
      ('send_free', pose)       end cell 0, segment has no blocked sample
      ('send_unknown', pose)    near frontier unknown; sent to grow the map
      ('hold_unknown', None)    far unknown; wait for laser/costmap to paint
      ('detour', pose)          straight blocked; first free pass line toward goal_y
      ('hold_blocked', None)    no free pass line in the lane
    When from_xy is set, samples the segment from the Husky to the step so a
    rock between crumbs is treated as blocked even if the end cell is free.
    """
    end_state = _cell_state(cost_at(float(x), float(y)))
    straight_hit = end_state == 'blocked'
    if from_xy is not None and not straight_hit:
        straight_hit = _segment_blocked(
            float(from_xy[0]), float(from_xy[1]), float(x), float(y), cost_at)

    def _approach_clear(px: float, py: float) -> bool:
        if from_xy is None:
            return True
        return not _approach_blocked(
            float(from_xy[0]), float(from_xy[1]), px, py, cost_at)

    def _detour() -> Tuple[str, Optional[Pose]]:
        for py in _pass_candidates(y, pass_ys, y_min, y_max, goal_y=goal_y):
            if abs(py - float(y)) < 0.05:
                continue
            if _cell_state(cost_at(float(x), py)) != 'free':
                continue
            # A free cell whose path still crosses the blob is how the Husky
            # was sent into the rock at x≈36. Nav2 does not get that cell.
            if not _approach_clear(float(x), py):
                continue
            travel = _travel_yaw(float(x), float(y), float(x), py, float(yaw))
            return ('detour', (float(x), py, travel))
        return ('hold_blocked', None)

    if not straight_hit and end_state == 'free':
        return ('send_free', (float(x), float(y), float(yaw)))
    if not straight_hit and end_state == 'unknown':
        near = True
        if from_xy is not None:
            near = math.hypot(
                float(x) - float(from_xy[0]),
                float(y) - float(from_xy[1]),
            ) <= float(unknown_send_m)
        if near:
            return ('send_unknown', (float(x), float(y), float(yaw)))
        decision, pose = _detour()
        if pose is not None:
            return (decision, pose)
        return ('hold_unknown', None)
    # Blocked straight step (or blocked segment): north free cell, else hold.
    return _detour()


def sight_clear(
    stand_x: float,
    stand_y: float,
    pin_x: float,
    pin_y: float,
    cost_at,
) -> bool:
    """True when no occupied costmap cell lies on the ray from stand to pin.

    Unknown cells do not block the ray. The stand cell itself is the start
    sample and is not treated as the obstacle.
    Uses _camera_cell_state: only lethal/inscribed obstacles (cost >= OCCUPIED_COST)
    block light rays; inflation halos around obstacles are transparent.
    """
    return not _segment_blocked(
        float(stand_x), float(stand_y), float(pin_x), float(pin_y), cost_at,
        state_fn=_camera_cell_state)


def near_sight_clear(
    stand_x: float,
    stand_y: float,
    yaw: float,
    cost_at,
    dist_m: float = 2.5,
    half_fov: float = 0.35,
) -> bool:
    """True when the near-field camera cone is clear of obstacles.

    Verifies that no lethal obstacle (cost >= OCCUPIED_COST) lies within dist_m
    along the camera heading (yaw) within [-half_fov, +half_fov]. Also ensures
    the stand itself is not on an obstacle.
    """
    if _camera_cell_state(cost_at(float(stand_x), float(stand_y))) == 'blocked':
        return False
    angles = (
        float(yaw) - float(half_fov),
        float(yaw) - 0.5 * float(half_fov),
        float(yaw),
        float(yaw) + 0.5 * float(half_fov),
        float(yaw) + float(half_fov),
    )
    start_r = min(0.6, float(dist_m))
    step_r = 0.3
    num_steps = max(1, int(math.ceil((float(dist_m) - start_r) / step_r)))
    for i in range(num_steps + 1):
        r = start_r + i * step_r
        if r > float(dist_m) + 1e-6:
            r = float(dist_m)
        for ang in angles:
            cx = float(stand_x) + r * math.cos(ang)
            cy = float(stand_y) + r * math.sin(ang)
            if _camera_cell_state(cost_at(cx, cy)) == 'blocked':
                return False
    return True


def choose_stand(
    view_x: float,
    view_y: float,
    yaw: float,
    pin_x: float,
    pin_y: float,
    cost_at,
    x_offsets: Sequence[float] = SIGHT_X_OFFSETS_M,
    pass_ys: Sequence[float] = LANE_PASS_YS,
    y_min: float = LANE_Y_MIN,
    y_max: float = LANE_Y_MAX,
    dist_m: float = 2.5,
) -> Optional[Pose]:
    """First lane cell that is free and can see the pin.

    Search walks along the lane (small x offsets, nearest first) and across
    candidate stand lines (closest to nominal view y first). Candidate must be
    free, have a clear sight ray to the pin, and an unoccluded near-field camera cone
    along the true aim angle toward the pin.
    """
    ys = _stand_candidates(float(view_y), pass_ys, y_min, y_max)
    for dx in x_offsets:
        x = float(view_x) + float(dx)
        for y in ys:
            if _cell_state(cost_at(x, y)) != 'free':
                continue
            cand_yaw = math.atan2(float(pin_y) - float(y), float(pin_x) - float(x))
            if not sight_clear(x, y, pin_x, pin_y, cost_at):
                continue
            if not near_sight_clear(x, y, cand_yaw, cost_at, dist_m=dist_m):
                continue
            return (x, y, cand_yaw)
    return None


def pose_past_block(
    goal_xy: Tuple[float, float],
    blocked_xy: Tuple[float, float],
    cost_at,
    step: float = 4.0,
    pass_ys: Sequence[float] = LANE_PASS_YS,
    y_min: float = LANE_Y_MIN,
    y_max: float = LANE_Y_MAX,
    from_xy: Optional[Tuple[float, float]] = None,
) -> Optional[Pose]:
    """Next free lane cell toward the goal, past a fully blocked slice.

    The blocked x is not reused. When from_xy is set, the cell is returned
    only if the drive from the Husky to it does not cross an occupied cell.
    None when nothing reachable remains before the goal.
    """
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    bx, by = float(blocked_xy[0]), float(blocked_xy[1])
    if math.hypot(gx - bx, gy - by) < max(float(step) * 0.5, 1.0):
        return None
    travel = math.hypot(gx - bx, gy - by)
    n = max(1, int(math.ceil(travel / max(float(step), 0.5))))
    ys = _pass_candidates(gy, pass_ys, y_min, y_max, goal_y=gy)
    for i in range(1, n + 1):
        t = i / float(n)
        x = bx + (gx - bx) * t
        y_here = by + (gy - by) * t
        for y in (y_here,) + tuple(ys):
            if _cell_state(cost_at(x, y)) != 'free':
                continue
            if from_xy is not None and _approach_blocked(
                    float(from_xy[0]), float(from_xy[1]), x, y, cost_at):
                continue
            yaw = math.atan2(gy - y, gx - x)
            return (x, y, yaw)
    return None


def pose_retreat(
    from_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    cost_at,
    pass_ys: Sequence[float] = LANE_PASS_YS,
    y_min: float = LANE_Y_MIN,
    y_max: float = LANE_Y_MAX,
) -> Optional[Pose]:
    """A free cell behind the Husky, off the blob it is pressed against.

    Used when every forward cell still crosses occupied cost. The step is
    opposite the goal along x, on the current lane line and the north passes.
    """
    hx, hy = float(from_xy[0]), float(from_xy[1])
    gx = float(goal_xy[0])
    sign = -1.0 if gx >= hx else 1.0
    ys = _pass_candidates(hy, pass_ys, y_min, y_max)
    for dist in (2.0, 4.0):
        x = hx + sign * dist
        for y in ys:
            if _cell_state(cost_at(x, y)) != 'free':
                continue
            if _approach_blocked(hx, hy, x, y, cost_at):
                continue
            yaw = math.atan2(y - hy, x - hx)
            return (x, y, yaw)
    return None


def plan_free(
    start: Tuple[float, float],
    goal: Pose,
    cost_at,
    step: float = 4.0,
    pass_ys: Sequence[float] = LANE_PASS_YS,
    y_min: float = LANE_Y_MIN,
    y_max: float = LANE_Y_MAX,
) -> List[Pose]:
    """Walk to goal in short steps, shifting a blocked step north.

    Stops at the first fully blocked cell with no free pass line. Unseen
    cells stay on the straight line. The last pose is the goal when that
    cell is free or unseen. A known grid of only 0 and occupied costs is
    the case the unit test builds.
    """
    gx, gy, gyaw = float(goal[0]), float(goal[1]), float(goal[2])
    cx, cy = float(start[0]), float(start[1])
    out: List[Pose] = []
    for _ in range(80):
        dist = math.hypot(gx - cx, gy - cy)
        if dist < 1.0:
            decision, pose = classify_step(
                gx, gy, gyaw, cost_at, pass_ys, y_min, y_max,
                from_xy=(cx, cy))
            if pose is not None and decision != 'detour':
                out.append((gx, gy, gyaw))
            elif pose is not None:
                out.append(pose)
            break
        travel = math.atan2(gy - cy, gx - cx)
        reach = min(float(step), dist)
        nx = cx + (gx - cx) * (reach / dist)
        ny = cy + (gy - cy) * (reach / dist)
        decision, pose = classify_step(
            nx, ny, travel, cost_at, pass_ys, y_min, y_max,
            from_xy=(cx, cy))
        if pose is None:
            break
        if math.hypot(pose[0] - cx, pose[1] - cy) < 0.3:
            break
        face = _travel_yaw(cx, cy, pose[0], pose[1], travel)
        out.append((pose[0], pose[1], face))
        cx, cy = pose[0], pose[1]
    return out
