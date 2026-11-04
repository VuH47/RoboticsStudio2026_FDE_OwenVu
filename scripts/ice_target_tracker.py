"""Averaged ice targets and an ordered Husky queue. No ROS imports.

The Parrot detector publishes one pose per accepted frame. This module merges
those hits into one target per ice block (per-axis median, so one bad frame
does not move it), and picks which target the Husky should visit next.
"""

from __future__ import annotations

import math
import statistics
from typing import Any, Dict, List, Optional, Tuple

TRACKING = 'tracking'
READY = 'ready'
ACTIVE = 'active'
CONFIRMED = 'confirmed'
NOT_CONFIRMED = 'not_confirmed'
INCONCLUSIVE = 'inconclusive'
MISSED = 'missed'
FINISHED = (CONFIRMED, NOT_CONFIRMED, INCONCLUSIVE, MISSED)


class Track:
    def __init__(self, track_id: str, max_hits: int) -> None:
        self.id = track_id
        self.max_hits = max_hits
        self.hits: List[Tuple[float, float, float, float]] = []
        self.state = TRACKING
        self.result: Optional[str] = None
        self.priority: Optional[str] = None
        self.priority_reason: Optional[str] = None

    def add(self, t: float, x: float, y: float, z: float) -> None:
        self.hits.append((t, x, y, z))
        if len(self.hits) > self.max_hits:
            self.hits = self.hits[-self.max_hits:]

    @property
    def n_hits(self) -> int:
        return len(self.hits)

    @property
    def position(self) -> Tuple[float, float, float]:
        return (
            statistics.median(h[1] for h in self.hits),
            statistics.median(h[2] for h in self.hits),
            statistics.median(h[3] for h in self.hits),
        )

    @property
    def spread(self) -> float:
        x, y, _z = self.position
        return max(math.hypot(h[1] - x, h[2] - y) for h in self.hits)


class TargetTracker:
    def __init__(self, merge_radius: float = 3.0, min_hits: int = 3,
                 max_hits: int = 40) -> None:
        self.merge_radius = merge_radius
        self.min_hits = min_hits
        self.max_hits = max_hits
        self.tracks: List[Track] = []

    def add_hit(self, t: float, x: float, y: float, z: float) -> Track:
        best: Optional[Track] = None
        best_d = self.merge_radius
        for tr in self.tracks:
            px, py, pz = tr.position
            d = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
            if d < best_d:
                best, best_d = tr, d
        if best is None:
            best = Track('T%02d' % (len(self.tracks) + 1), self.max_hits)
            self.tracks.append(best)
        best.add(t, x, y, z)
        if best.state == TRACKING and best.n_hits >= self.min_hits:
            best.state = READY
        return best

    def get(self, track_id: str) -> Optional[Track]:
        for tr in self.tracks:
            if tr.id == track_id:
                return tr
        return None

    def set_priority(self, track_id: str, priority: str,
                     reason: Optional[str] = None) -> None:
        """Record Husky's hazard judgement. Does not change visit order."""
        tr = self.get(track_id)
        if tr is not None:
            tr.priority = priority
            tr.priority_reason = reason

    def set_state(self, track_id: str, state: str,
                  result: Optional[str] = None) -> None:
        tr = self.get(track_id)
        if tr is not None:
            tr.state = state
            if result is not None:
                tr.result = result

    def unfinished_count(self) -> int:
        return sum(1 for t in self.tracks if t.state not in FINISHED)

    def _eligible(self, final_pass: bool) -> List[Track]:
        out = []
        for tr in self.tracks:
            if tr.state == READY or (
                    final_pass and tr.state == TRACKING and tr.n_hits >= 1):
                out.append(tr)
        return out

    def queue_order(self, from_xy: Tuple[float, float], lane_y: float,
                    final_pass: bool = False) -> List[Track]:
        """Greedy nearest-neighbour order along the lane."""
        left = self._eligible(final_pass)
        cx, cy = from_xy
        order: List[Track] = []
        while left:
            nxt = min(left, key=lambda t: math.hypot(
                t.position[0] - cx, lane_y - cy))
            left.remove(nxt)
            order.append(nxt)
            cx, cy = nxt.position[0], lane_y
        return order

    def pick_next(self, from_xy: Tuple[float, float], lane_y: float,
                  final: bool = False) -> Optional[Track]:
        order = self.queue_order(from_xy, lane_y, final)
        return order[0] if order else None

    def snapshot(self, husky_xy: Tuple[float, float],
                 lane_y: float) -> List[Dict[str, Any]]:
        rank = {t.id: i + 1 for i, t in enumerate(
            self.queue_order(husky_xy, lane_y, True))
            if t.state in (READY, TRACKING)}
        out = []
        for tr in self.tracks:
            x, y, z = tr.position
            out.append({
                'id': tr.id,
                'state': tr.state,
                'x': round(x, 2), 'y': round(y, 2), 'z': round(z, 2),
                'n_hits': tr.n_hits,
                'spread_m': round(tr.spread, 2),
                'rank': rank.get(tr.id, 0),
                'dist_m': round(math.hypot(x - husky_xy[0],
                                           lane_y - husky_xy[1]), 1),
                'result': tr.result,
                'priority': tr.priority,
                'priority_reason': tr.priority_reason,
            })
        return out

