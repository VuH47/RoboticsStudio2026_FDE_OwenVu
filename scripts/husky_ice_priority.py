#!/usr/bin/env python3
"""Husky's hazard judgement. No ROS, and no ice ground-truth file.

The Parrot only supplies a pose. After Husky's camera confirms that pose,
this module scores where the ice sits on the known lift: towers, the two
cables, and the two gondolas. Visit order is not decided here.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from ice_cable_geometry import (
    CABINS,
    CABLE_Y_MODEL,
    SPANS,
    WORLD_OFFSET,
    Span,
)

# Launch file tunes the Husky camera at 480x360 with this minimum blob.
MIN_SLEEVE_AREA_PX = 14.0
MIDSPAN_DT = 0.15
OFFSET_DT = 0.30
CABIN_HAZARD_M = 4.0

CASE_TEXT = {
    'midspan_cog': (
        'mid-span ice moves the cable centre of gravity off the towers'),
    'offset_mass': 'ice sits between mid-span and a tower',
    'near_tower': 'ice is near a tower, which carries the load',
    'single_cable': 'only one cable is loaded',
    'over_cabin': 'ice is over a gondola',
    'large_sleeve': 'the sleeve looks large in the Husky camera',
    'medium_sleeve': 'the sleeve is a medium size in the Husky camera',
    'small_sleeve': 'the sleeve looks small in the Husky camera',
}


def _span_at(model_x: float) -> Optional[Span]:
    hits: List[Span] = []
    for span in SPANS.values():
        if span.x0 - 1e-6 <= model_x <= span.x1 + 1e-6:
            hits.append(span)
    if not hits:
        return None
    # A tower x belongs to two spans. Keep the one where t is interior.
    def outside(span: Span) -> int:
        t = span.t_of_x(model_x)
        return 0 if 0.0 < t < 1.0 else 1
    hits.sort(key=outside)
    return hits[0]


def _nearest_cable(model_y: float) -> str:
    return min(CABLE_Y_MODEL, key=lambda name: abs(CABLE_Y_MODEL[name] - model_y))


def _cabin_name(span: Span, t: float) -> Optional[str]:
    for name, x_cabin, cabin_span in CABINS:
        if cabin_span != span.span_id:
            continue
        dist = abs(t - span.t_of_x(x_cabin)) * span.chord_length
        if dist <= CABIN_HAZARD_M:
            return name
    return None


def _sleeve_case(area_px: float, min_area_px: float) -> Tuple[str, int]:
    floor = max(float(min_area_px), 1.0)
    if area_px >= 4.0 * floor:
        return 'large_sleeve', 2
    if area_px >= 2.0 * floor:
        return 'medium_sleeve', 1
    return 'small_sleeve', 0


def judge_ice(
    position: Sequence[float],
    area_px: float,
    min_area_px: float = MIN_SLEEVE_AREA_PX,
) -> Optional[Dict[str, object]]:
    """Score one confirmed ice pose. None when the pose is not on a span."""
    if len(position) != 3:
        return None
    x, y, _z = (float(position[0]), float(position[1]), float(position[2]))
    model_x = x - WORLD_OFFSET[0]
    model_y = y - WORLD_OFFSET[1]
    span = _span_at(model_x)
    if span is None:
        return None
    t = span.t_of_x(model_x)
    cable_id = _nearest_cable(model_y)
    station_dt = abs(t - 0.5)
    if station_dt <= MIDSPAN_DT:
        station, station_pts = 'midspan_cog', 3
    elif station_dt <= OFFSET_DT:
        station, station_pts = 'offset_mass', 1
    else:
        station, station_pts = 'near_tower', 0

    cases: List[str] = [station, 'single_cable']
    score = station_pts + 1
    cabin = _cabin_name(span, t)
    if cabin is not None:
        cases.append('over_cabin')
        score += 3
    sleeve, sleeve_pts = _sleeve_case(float(area_px), min_area_px)
    cases.append(sleeve)
    score += sleeve_pts

    if (score >= 5 or 'over_cabin' in cases
            or ('midspan_cog' in cases and 'large_sleeve' in cases)):
        priority = 'HIGH'
    elif score >= 3:
        priority = 'MEDIUM'
    else:
        priority = 'LOW'

    return {
        'priority': priority,
        'score': score,
        'cases': cases,
        'reason': '; '.join(CASE_TEXT[name] for name in cases),
        'span_id': span.span_id,
        'cable_id': cable_id,
        't': round(t, 3),
        'cabin': cabin,
    }
