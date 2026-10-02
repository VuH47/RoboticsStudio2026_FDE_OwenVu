#!/usr/bin/env python3
"""Cable centreline for alpine_trail ice-block placement .

Every valid ice position is (span_id, cable_id, t) with t in [0, 1] along the
span (linear in model x). This module is pure Python 3 stdlib; no rclpy.

Frames
------
The alpine_trail include pose is WORLD_OFFSET = (20, -30, 0) with no extra RPY.
The mesh is pre-inclined: ground z ≈ tan(RAMP_PITCH)*x in the model frame.

    world = model + WORLD_OFFSET

Centreline
----------
Twin tubes, diameter 0.34 m. cables.obj vertex AABB y[-0.92, 0.92] with radius
0.17 m ⇒ centreline y = ±0.75 (locked names cable_left / cable_right).

Tower-head z is the outer-vertex envelope midpoint of cables.obj (not the
z = tan(0.07)*x + 6.50 rule, which sits ~0.12 m above T0). Between towers a
vertical parabola drops sag metres below the chord at t = 0.5 (sag ≤ 0.35 m,
fitted to mesh ring centroids at each span midpoint).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple


WORLD_OFFSET: Tuple[float, float, float] = (20.0, -30.0, 0.0)
RAMP_PITCH: float = 0.07
CABLE_RADIUS: float = 0.17

# Mesh-measured centreline y. World: left −30.75 (away from Nav2), right −29.25.
CABLE_Y_MODEL: Dict[str, float] = {
    "cable_left": -0.75,
    "cable_right": 0.75,
}

# Tower model x and mesh envelope-mid z (outer vertices, centre ≈ mid of range).
TOWER_X: Tuple[float, float, float, float] = (8.0, 24.0, 40.0, 54.0)
TOWER_Z: Tuple[float, float, float, float] = (6.938, 8.153, 9.305, 10.266)

VALID_T_RANGE: Tuple[float, float] = (0.15, 0.85)
CABIN_KEEP_OUT_M: float = 1.5

PHASE3_SPAN = "span_01"
PHASE3_T = 0.5
PHASE3_CABLE = "cable_right"


@dataclass(frozen=True)
class Pose:
    """Position and RPY (radians) in one frame (model or world)."""

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    frame: str = "world"

    def xyz(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def rpy(self) -> Tuple[float, float, float]:
        return (self.roll, self.pitch, self.yaw)


@dataclass(frozen=True)
class Span:
    """One tower-to-tower cable run. t = 0 at tower_a, t = 1 at tower_b."""

    span_id: str
    tower_a: int
    tower_b: int
    x0: float
    x1: float
    z0: float
    z1: float
    sag: float  # metres below the chord at t = 0.5

    @property
    def length_x(self) -> float:
        return self.x1 - self.x0

    @property
    def chord_length(self) -> float:
        return math.hypot(self.x1 - self.x0, self.z1 - self.z0)

    def x_at(self, t: float) -> float:
        return self.x0 + t * (self.x1 - self.x0)

    def z_at(self, t: float) -> float:
        # Vertical sag parabola: 4 s t (1-t) metres below the tower chord.
        return (1.0 - t) * self.z0 + t * self.z1 - 4.0 * self.sag * t * (1.0 - t)

    def dz_dx(self, t: float) -> float:
        dz_dt = (self.z1 - self.z0) - 4.0 * self.sag * (1.0 - 2.0 * t)
        return dz_dt / (self.x1 - self.x0)

    def pitch_at(self, t: float) -> float:
        return math.atan(self.dz_dx(t))

    def t_of_x(self, x: float) -> float:
        return (x - self.x0) / (self.x1 - self.x0)


# sag: cables.obj ring-centroid z at span mid-x, minus the tower chord (≤ 0.35 m).
SPANS: Dict[str, Span] = {
    "span_01": Span("span_01", 0, 1, 8.0, 24.0, 6.938, 8.153, 0.267),
    "span_02": Span("span_02", 1, 2, 24.0, 40.0, 8.153, 9.305, 0.328),
    "span_03": Span("span_03", 2, 3, 40.0, 54.0, 9.305, 10.266, 0.339),
}

# Cabins sit under the cables; keep-out is CABIN_KEEP_OUT_M along the chord.
CABINS: Tuple[Tuple[str, float, str], ...] = (
    ("cabin_0", 18.0, "span_01"),
    ("cabin_1", 46.0, "span_03"),
)

SPAN_IDS: Tuple[str, ...] = tuple(SPANS.keys())
CABLE_IDS: Tuple[str, ...] = ("cable_left", "cable_right")

# Cylinder length, radius (m). Width/height for truth = 2 * radius.
SEVERITY_SIZE: Dict[str, Tuple[float, float]] = {
    "small": (0.50, 0.22),
    "medium": (1.00, 0.32),
    "large": (1.80, 0.45),
}

# Preloaded Gazebo names → severity. Hidden at z = HIDDEN_Z until the manager
# calls set_pose. Keep this list in sync with worlds/snow_forest.sdf.
ICE_POOL: Tuple[Tuple[str, str], ...] = (
    ("ice_block_01", "small"),
    ("ice_block_02", "small"),
    ("ice_block_03", "small"),
    ("ice_block_04", "medium"),
    ("ice_block_05", "medium"),
    ("ice_block_06", "medium"),
    ("ice_block_07", "large"),
    ("ice_block_08", "large"),
)
HIDDEN_Z: float = -20.0
MIN_SPACING_M: float = 3.0

# Husky stays on the Nav2 ski corridor and yaws −Y toward the lift.
HUSKY_INSPECT_Y: float = 0.0
HUSKY_INSPECT_YAW: float = -math.pi / 2.0
CABLE_ENVELOPE_Y: float = -12.0  # |y| < 12 is the drive lane; closer is a violation.


def _require_span(span_id: str) -> Span:
    try:
        return SPANS[span_id]
    except KeyError as exc:
        raise ValueError(
            "unknown span_id %r; expected one of %s" % (span_id, SPAN_IDS)
        ) from exc


def _require_cable_y(cable_id: str) -> float:
    try:
        return CABLE_Y_MODEL[cable_id]
    except KeyError as exc:
        raise ValueError(
            "unknown cable_id %r; expected one of %s" % (cable_id, CABLE_IDS)
        ) from exc


def model_to_world(pose: Pose) -> Pose:
    """Translate by WORLD_OFFSET. Include RPY is zero, so orientation is copied."""
    ox, oy, oz = WORLD_OFFSET
    return Pose(
        x=pose.x + ox,
        y=pose.y + oy,
        z=pose.z + oz,
        roll=pose.roll,
        pitch=pose.pitch,
        yaw=pose.yaw,
        frame="world",
    )


def world_to_model(pose: Pose) -> Pose:
    ox, oy, oz = WORLD_OFFSET
    return Pose(
        x=pose.x - ox,
        y=pose.y - oy,
        z=pose.z - oz,
        roll=pose.roll,
        pitch=pose.pitch,
        yaw=pose.yaw,
        frame="model",
    )


def cable_pose(
    span_id: str,
    cable_id: str,
    t: float,
    frame: str = "world",
) -> Pose:
    """Centreline pose at t. Local +X is the uphill tangent (increasing model x).

    Roll = 0, yaw = 0 (the lift runs along +X). Pitch follows dz/dx of the
    sagging centreline (chord slope plus sag derivative).
    """
    if frame not in ("world", "model"):
        raise ValueError("frame must be 'world' or 'model', got %r" % (frame,))
    span = _require_span(span_id)
    y = _require_cable_y(cable_id)
    t = float(t)
    pose = Pose(
        x=span.x_at(t),
        y=y,
        z=span.z_at(t),
        roll=0.0,
        pitch=span.pitch_at(t),
        yaw=0.0,
        frame="model",
    )
    if frame == "world":
        return model_to_world(pose)
    return pose


def cabin_keepout_t(span_id: str) -> Optional[Tuple[float, float, str]]:
    """Return (t_centre, dt_half, cabin_name) in this span, or None.

    dt_half converts CABIN_KEEP_OUT_M along the cable chord into t.
    """
    span = _require_span(span_id)
    for name, x_cabin, cabin_span in CABINS:
        if cabin_span != span_id:
            continue
        t_c = span.t_of_x(x_cabin)
        dt = CABIN_KEEP_OUT_M / span.chord_length
        return (t_c, dt, name)
    return None


def valid_t_intervals(span_id: str) -> List[Tuple[float, float]]:
    """Closed t intervals that pass tower and cabin keep-outs, inside [0, 1]."""
    _require_span(span_id)
    t0, t1 = VALID_T_RANGE
    intervals: List[Tuple[float, float]] = [(t0, t1)]
    keep = cabin_keepout_t(span_id)
    if keep is None:
        return intervals
    t_c, dt, _name = keep
    lo, hi = t_c - dt, t_c + dt
    out: List[Tuple[float, float]] = []
    for a, b in intervals:
        if hi <= a or lo >= b:
            out.append((a, b))
            continue
        if a < lo:
            out.append((a, min(b, lo)))
        if b > hi:
            out.append((max(a, hi), b))
    return [(a, b) for a, b in out if b > a]


def is_valid(span_id: str, cable_id: str, t: float) -> bool:
    """False in tower heads, cabin keep-outs, or outside the span."""
    if span_id not in SPANS or cable_id not in CABLE_Y_MODEL:
        return False
    try:
        t = float(t)
    except (TypeError, ValueError):
        return False
    if math.isnan(t) or math.isinf(t):
        return False
    if t < VALID_T_RANGE[0] or t > VALID_T_RANGE[1]:
        return False
    keep = cabin_keepout_t(span_id)
    if keep is not None:
        t_c, dt, _name = keep
        if abs(t - t_c) < dt:
            return False
    return True


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) for Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def size_xyz(severity: str) -> Tuple[float, float, float]:
    """Truth size [length along cable, width, height] from cylinder L and 2r."""
    if severity not in SEVERITY_SIZE:
        raise ValueError("unknown severity %r" % (severity,))
    length, radius = SEVERITY_SIZE[severity]
    d = 2.0 * radius
    return (length, d, d)


def _rand_t(rng: random.Random, span_id: str) -> Optional[float]:
    intervals = valid_t_intervals(span_id)
    if not intervals:
        return None
    widths = [b - a for a, b in intervals]
    total = sum(widths)
    if total <= 0.0:
        return None
    pick = rng.random() * total
    acc = 0.0
    for (a, b), w in zip(intervals, widths):
        acc += w
        if pick <= acc:
            return a + (pick - (acc - w))
    return intervals[-1][1]


def _too_close(pose: Pose, placed: Sequence[Pose], min_spacing: float) -> bool:
    for other in placed:
        dx = pose.x - other.x
        dy = pose.y - other.y
        dz = pose.z - other.z
        if (dx * dx + dy * dy + dz * dz) < (min_spacing * min_spacing):
            return True
    return False


def sample_layout(
    seed: int,
    block_count: int,
    min_spacing: float = MIN_SPACING_M,
    severities: Optional[Sequence[str]] = None,
    span_ids: Optional[Sequence[str]] = None,
    cable_ids: Optional[Sequence[str]] = None,
    one_per_span: bool = True,
    max_tries: int = 500,
) -> List[Dict[str, Any]]:
    """Deterministic ice poses. Same seed → same list. No ROS."""
    if block_count < 0:
        raise ValueError("block_count must be >= 0")
    spans = tuple(span_ids) if span_ids is not None else SPAN_IDS
    cables = tuple(cable_ids) if cable_ids is not None else CABLE_IDS
    allowed_sev = tuple(severities) if severities is not None else tuple(SEVERITY_SIZE.keys())
    for s in spans:
        _require_span(s)
    for c in cables:
        _require_cable_y(c)
    for sev in allowed_sev:
        if sev not in SEVERITY_SIZE:
            raise ValueError("unknown severity %r" % (sev,))

    pool = [(name, sev) for name, sev in ICE_POOL if sev in allowed_sev]
    rng = random.Random(int(seed))
    rng.shuffle(pool)
    if block_count > len(pool):
        raise ValueError(
            "block_count %d exceeds pool of %d matching models" % (block_count, len(pool))
        )
    chosen = pool[:block_count]

    placed_poses: List[Pose] = []
    records: List[Dict[str, Any]] = []
    span_cycle = list(spans)
    for i, (name, sev) in enumerate(chosen):
        record = None
        for _try in range(max_tries):
            if one_per_span and span_cycle:
                span_id = span_cycle[i % len(span_cycle)]
            else:
                span_id = rng.choice(span_cycle)
            cable_id = rng.choice(list(cables))
            t = _rand_t(rng, span_id)
            if t is None or not is_valid(span_id, cable_id, t):
                continue
            pose = cable_pose(span_id, cable_id, t, frame="world")
            if _too_close(pose, placed_poses, min_spacing):
                continue
            length, width, height = size_xyz(sev)
            record = {
                "id": name,
                "span_id": span_id,
                "cable_id": cable_id,
                "t": float(t),
                "severity": sev,
                "position": [pose.x, pose.y, pose.z],
                "orientation": [pose.roll, pose.pitch, pose.yaw],
                "size": [length, width, height],
                "seed": int(seed),
            }
            placed_poses.append(pose)
            break
        if record is None:
            raise RuntimeError(
                "could not place block %s after %d tries (seed=%s)" % (name, max_tries, seed)
            )
        records.append(record)
    return records


def husky_inspect_waypoints(
    span_ids: Optional[Sequence[str]] = None,
) -> List[Tuple[str, float, float, float]]:
    """Corridor goals (label, x, y, yaw) facing the lift. Nav2 is 2D."""
    spans = tuple(span_ids) if span_ids is not None else SPAN_IDS
    out: List[Tuple[str, float, float, float]] = []
    for span_id in spans:
        pose = cable_pose(span_id, "cable_right", 0.5, frame="world")
        out.append(("inspect_%s" % span_id, pose.x, HUSKY_INSPECT_Y, HUSKY_INSPECT_YAW))
    return out


# Deploy station include pose in snow_forest; roof hover above pad (~2.86 m).
DEPLOY_STATION_XY: Tuple[float, float] = (-8.0, 0.0)
DEPLOY_ROOF_Z: float = 3.9
PARROT_VIEW_Y_OFFSET: float = 3.0  # toward corridor (+Y from cable_right)
PARROT_VIEW_Z_OFFSET: float = 1.2  # above cable centreline
PARROT_CLIMB_Z: float = 4.5


def parrot_cable_tour_waypoints(
    spawn_xy: Optional[Tuple[float, float]] = None,
) -> List[Tuple[str, float, float, float, float]]:
    """Short 3D tour: climb → roof → approach → span01 view → roof.

    Returns (label, x, y, z, yaw). yaw at view faces the cable (−Y).
    """
    sx, sy = spawn_xy if spawn_xy is not None else (2.0, 0.0)
    roof_x, roof_y = DEPLOY_STATION_XY
    roof_z = DEPLOY_ROOF_Z
    cable = cable_pose("span_01", "cable_right", 0.5, frame="world")
    view_x = cable.x
    view_y = cable.y + PARROT_VIEW_Y_OFFSET
    view_z = cable.z + PARROT_VIEW_Z_OFFSET
    view_yaw = -math.pi / 2.0
    mid_x = 0.5 * (roof_x + view_x)
    mid_y = 0.5 * (roof_y + view_y)
    mid_z = max(roof_z, view_z)
    return [
        ("climb", sx, sy, PARROT_CLIMB_Z, 0.0),
        ("deploy_roof", roof_x, roof_y, roof_z, 0.0),
        ("span01_approach", mid_x, mid_y, mid_z, view_yaw),
        ("span01_view", view_x, view_y, view_z, view_yaw),
        ("return_roof", roof_x, roof_y, roof_z, 0.0),
    ]


def parrot_nav2_tour_xy() -> List[Tuple[str, float, float, float]]:
    """Phase 2 planar goals from Phase 1 XYZ waypoints (drop climb; rename roof).

    Returns (label, x, y, yaw). z is ignored — altitude-follow owns height.
    """
    out: List[Tuple[str, float, float, float]] = []
    for label, x, y, _z, yaw in parrot_cable_tour_waypoints():
        if label == "climb":
            continue
        if label == "deploy_roof":
            out.append(("deploy_near", x, y, yaw))
        elif label == "return_roof":
            out.append(("return_deploy", x, y, yaw))
        else:
            out.append((label, x, y, yaw))
    return out


def parrot_ice_patrol_waypoints(
    deploy_xy: Optional[Tuple[float, float]] = None,
    roof_z: float = DEPLOY_ROOF_Z,
) -> List[Tuple[str, float, float, float, float]]:
    """Aerial patrol: roof hover → cable-side view (detect ice), then hold."""
    dx, dy = deploy_xy if deploy_xy is not None else DEPLOY_STATION_XY
    cable = cable_pose("span_01", "cable_right", 0.5, frame="world")
    view_x = cable.x
    view_y = cable.y + PARROT_VIEW_Y_OFFSET
    view_z = cable.z + PARROT_VIEW_Z_OFFSET
    view_yaw = -math.pi / 2.0
    return [
        ("deploy_roof", dx, dy, roof_z, 0.0),
        ("span01_view", view_x, view_y, view_z, view_yaw),
    ]


def husky_corridor_goal_for_ice_x(ice_x: float) -> Tuple[float, float, float]:
    """Ground Nav2 goal on corridor y=0 aligned with ice world x."""
    return (float(ice_x), HUSKY_INSPECT_Y, HUSKY_INSPECT_YAW)


def husky_nav2_tour_xy() -> List[Tuple[str, float, float, float]]:
    """Husky Phase 2 short Nav2 loop — cable corridor only (no deploy station).

    Returns (label, x, y, yaw). Starts toward ramp/span_01 inspect at y=0.
    """
    cable = cable_pose("span_01", "cable_right", 0.5, frame="world")
    return [
        ("span01_approach", 22.0, HUSKY_INSPECT_Y, 0.0),
        ("span01_view", cable.x, HUSKY_INSPECT_Y, HUSKY_INSPECT_YAW),
    ]


def min_cable_clearance_xy(x: float, y: float) -> Tuple[float, str, str]:
    """Nearest horizontal (xy) distance from a point to a cable centreline sample."""
    best = float("inf")
    best_span = SPAN_IDS[0]
    best_cable = CABLE_IDS[0]
    for span_id in SPAN_IDS:
        for cable_id in CABLE_IDS:
            for i in range(11):
                t = i / 10.0
                p = cable_pose(span_id, cable_id, t, frame="world")
                d = math.hypot(x - p.x, y - p.y)
                if d < best:
                    best = d
                    best_span = span_id
                    best_cable = cable_id
    return best, best_span, best_cable


def min_cable_clearance(x: float, y: float, z: float) -> Tuple[float, str, str]:
    """Nearest 3D distance from a point to a cable centreline sample."""
    best = float("inf")
    best_span = SPAN_IDS[0]
    best_cable = CABLE_IDS[0]
    for span_id in SPAN_IDS:
        for cable_id in CABLE_IDS:
            for i in range(11):
                t = i / 10.0
                p = cable_pose(span_id, cable_id, t, frame="world")
                d = math.sqrt((x - p.x) ** 2 + (y - p.y) ** 2 + (z - p.z) ** 2)
                if d < best:
                    best = d
                    best_span = span_id
                    best_cable = cable_id
    return best, best_span, best_cable


def corridor_violation(y: float) -> bool:
    """True if Husky has left the ski lane toward the alpine (y too negative)."""
    return y < CABLE_ENVELOPE_Y


def iter_valid_samples(
    step: float = 0.05,
    span_ids: Optional[Sequence[str]] = None,
    cable_ids: Optional[Sequence[str]] = None,
) -> Iterator[Tuple[str, str, float]]:
    """Yield (span_id, cable_id, t) on a uniform t grid that pass is_valid."""
    if step <= 0.0:
        raise ValueError("step must be positive")
    spans = tuple(span_ids) if span_ids is not None else SPAN_IDS
    cables = tuple(cable_ids) if cable_ids is not None else CABLE_IDS
    n = int(round(1.0 / step))
    for span_id in spans:
        _require_span(span_id)
        for cable_id in cables:
            _require_cable_y(cable_id)
            for i in range(n + 1):
                t = i * step
                if is_valid(span_id, cable_id, t):
                    yield (span_id, cable_id, t)


def _print_table(rows: Iterable[Sequence[object]]) -> None:
    text_rows = [[str(c) for c in row] for row in rows]
    widths = [max(len(r[i]) for r in text_rows) for i in range(len(text_rows[0]))]
    for i, row in enumerate(text_rows):
        line = "  ".join(cell.ljust(widths[j]) for j, cell in enumerate(row))
        print(line)
        if i == 0:
            print("  ".join("-" * widths[j] for j in range(len(widths))))


def main() -> None:
    ox, oy, oz = WORLD_OFFSET
    print("alpine_trail ice cable geometry — Phase 1")
    print(
        "WORLD_OFFSET = (%.1f, %.1f, %.1f)  include RPY = (0, 0, 0)"
        % (ox, oy, oz)
    )
    print(
        "RAMP_PITCH = %.2f rad   CABLE_RADIUS = %.2f m   sag <= 0.35 m"
        % (RAMP_PITCH, CABLE_RADIUS)
    )
    print(
        "cables: cable_left y_model=%.2f (world y=%.2f)  "
        "cable_right y_model=%.2f (world y=%.2f)"
        % (
            CABLE_Y_MODEL["cable_left"],
            CABLE_Y_MODEL["cable_left"] + oy,
            CABLE_Y_MODEL["cable_right"],
            CABLE_Y_MODEL["cable_right"] + oy,
        )
    )
    print()

    header = (
        "span",
        "cable",
        "t",
        "frame",
        "x",
        "y",
        "z",
        "pitch_rad",
        "valid",
    )
    rows: List[Sequence[object]] = [header]
    for span_id in SPANS:
        for cable_id in CABLE_IDS:
            for t in (0.0, 1.0):
                for frame in ("model", "world"):
                    pose = cable_pose(span_id, cable_id, t, frame=frame)
                    rows.append(
                        (
                            span_id,
                            cable_id,
                            "%.1f" % t,
                            frame,
                            "%8.3f" % pose.x,
                            "%8.3f" % pose.y,
                            "%8.3f" % pose.z,
                            "%8.5f" % pose.pitch,
                            str(is_valid(span_id, cable_id, t)),
                        )
                    )
    print("Span endpoints (t=0 tower downhill, t=1 tower uphill)")
    _print_table(rows)
    print()

    print("Valid t (default [%.2f, %.2f], cabin keep-out %.1f m along chord)"
          % (VALID_T_RANGE[0], VALID_T_RANGE[1], CABIN_KEEP_OUT_M))
    for span_id, span in SPANS.items():
        keep = cabin_keepout_t(span_id)
        intervals = valid_t_intervals(span_id)
        iv = ", ".join("[%.6f, %.6f]" % pair for pair in intervals)
        extra = "none"
        if keep is not None:
            t_c, dt, name = keep
            extra = "%s at model x  t=%.6f ± %.6f" % (name, t_c, dt)
        print(
            "  %s  chord=%.3f m  sag=%.3f m  keep-out: %s"
            % (span_id, span.chord_length, span.sag, extra)
        )
        print("           valid t: %s" % iv)
    print()

    print("Phase-3 placement  span=%s  t=%.1f" % (PHASE3_SPAN, PHASE3_T))
    for cable_id in (PHASE3_CABLE, "cable_left"):
        world = cable_pose(PHASE3_SPAN, cable_id, PHASE3_T, frame="world")
        model = cable_pose(PHASE3_SPAN, cable_id, PHASE3_T, frame="model")
        print("  %s" % cable_id)
        print(
            "    model  xyz=(%.4f, %.4f, %.4f)  rpy=(%.6f, %.6f, %.6f) rad"
            % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw)
        )
        print(
            "    world  xyz=(%.4f, %.4f, %.4f)  rpy=(%.6f, %.6f, %.6f) rad"
            % (world.x, world.y, world.z, world.roll, world.pitch, world.yaw)
        )
        print("    is_valid = %s" % is_valid(PHASE3_SPAN, cable_id, PHASE3_T))
    print()
    print("Husky inspect waypoints (corridor y=0, yaw=-pi/2)")
    for label, x, y, yaw in husky_inspect_waypoints():
        print("  %s  x=%.3f y=%.3f yaw=%.4f" % (label, x, y, yaw))
    print()
    print("Parrot short cable tour waypoints")
    for label, x, y, z, yaw in parrot_cable_tour_waypoints():
        print(
            "  %s  xyz=(%.2f, %.2f, %.2f) yaw=%.2f"
            % (label, x, y, z, yaw)
        )
    print("Parrot Nav2 Phase-2 tour XY")
    for label, x, y, yaw in parrot_nav2_tour_xy():
        print("  %s  xy=(%.2f, %.2f) yaw=%.2f" % (label, x, y, yaw))
    print("Husky Nav2 Phase-2 tour XY")
    for label, x, y, yaw in husky_nav2_tour_xy():
        print("  %s  xy=(%.2f, %.2f) yaw=%.2f" % (label, x, y, yaw))
    a = sample_layout(42, 3, one_per_span=True)
    b = sample_layout(42, 3, one_per_span=True)
    c = sample_layout(43, 3, one_per_span=True)
    assert a == b, "seed 42 must replay"
    assert a != c, "seed 43 must differ"
    print("seed replay OK  (42==42, 42!=43)  n=%d" % len(a))
    for rec in a:
        print(
            "  %s  %s %s t=%.3f  world=(%.3f, %.3f, %.3f)"
            % (
                rec["id"],
                rec["span_id"],
                rec["cable_id"],
                rec["t"],
                rec["position"][0],
                rec["position"][1],
                rec["position"][2],
            )
        )


if __name__ == "__main__":
    main()
