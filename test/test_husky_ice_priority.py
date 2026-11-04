#!/usr/bin/env python3
"""Offline tests for Husky's hazard rubric. No ROS.

Run:  python scripts/test_husky_ice_priority.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from ice_cable_geometry import SPANS, cable_pose  # noqa: E402
from husky_ice_priority import judge_ice  # noqa: E402

MIN_AREA = 14.0


def _xyz(span_id, t, cable='cable_right'):
    pose = cable_pose(span_id, cable, t)
    return (pose.x, pose.y, pose.z)


def test_midspan_medium_sleeve_is_high():
    verdict = judge_ice(_xyz('span_02', 0.5), 40.0, MIN_AREA)
    assert verdict is not None
    assert verdict['priority'] == 'HIGH'
    assert verdict['score'] == 5
    assert verdict['cases'][:2] == ['midspan_cog', 'single_cable']
    assert 'medium_sleeve' in verdict['cases']
    assert 'centre of gravity' in verdict['reason']
    assert 'only one cable is loaded' in verdict['reason']


def test_near_tower_small_sleeve_is_low():
    verdict = judge_ice(_xyz('span_01', 0.95), MIN_AREA, MIN_AREA)
    assert verdict is not None
    assert verdict['priority'] == 'LOW'
    assert verdict['score'] == 1
    assert 'near_tower' in verdict['cases']
    assert 'over_cabin' not in verdict['cases']


def test_cabin_block_is_high():
    span = SPANS['span_03']
    t = span.t_of_x(46.0)
    verdict = judge_ice(_xyz('span_03', t, 'cable_left'), MIN_AREA, MIN_AREA)
    assert verdict is not None
    assert verdict['cabin'] == 'cabin_1'
    assert 'over_cabin' in verdict['cases']
    assert verdict['priority'] == 'HIGH'


def test_pose_off_the_lift_has_no_priority():
    assert judge_ice((0.0, 0.0, 0.0), 80.0, MIN_AREA) is None


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        fn()
        print('ok  ', fn.__name__)
    print('%d tests passed' % len(tests))
