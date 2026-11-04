#!/usr/bin/env python3
"""Offline tests for ice_target_tracker."""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from ice_target_tracker import (  # noqa: E402
    ACTIVE, CONFIRMED, READY, TRACKING, TargetTracker)


def test_merge_and_median():
    t = TargetTracker(merge_radius=3.0, min_hits=3)
    for i, x in enumerate([40.0, 40.2, 39.9, 42.0]):
        t.add_hit(i, x, -30.0, 9.0)
    assert len(t.tracks) == 1
    assert abs(t.tracks[0].position[0] - 40.1) < 0.2
    assert t.tracks[0].n_hits == 4


def test_separate_blocks():
    t = TargetTracker()
    t.add_hit(0, 40, -30, 9)
    t.add_hit(0, 60, -30, 9)
    assert [x.id for x in t.tracks] == ['T01', 'T02']


def test_ready_gating_and_final_pass():
    t = TargetTracker(min_hits=3)
    t.add_hit(0, 40, -30, 9)
    t.add_hit(1, 40, -30, 9)
    assert t.tracks[0].state == TRACKING
    assert t.pick_next((0, 0), -10) is None
    assert t.pick_next((0, 0), -10, final=True) is not None
    t.add_hit(2, 40, -30, 9)
    assert t.tracks[0].state == READY
    assert t.pick_next((0, 0), -10) is t.tracks[0]


def test_priority_does_not_reorder_queue():
    t = TargetTracker(min_hits=1)
    far = t.add_hit(0, 70, -30, 9)
    t.add_hit(0, 30, -30, 9)
    t.set_priority(far.id, 'HIGH', 'mid-span ice')
    xs = [round(tr.position[0]) for tr in t.queue_order((0, 0), -10)]
    assert xs == [30, 70]


def test_queue_order_nearest_first():
    t = TargetTracker(min_hits=1)
    for x in (70, 30, 50):
        t.add_hit(0, x, -30, 9)
    xs = [round(tr.position[0]) for tr in t.queue_order((0, 0), -10)]
    assert xs == [30, 50, 70]
    xs = [round(tr.position[0]) for tr in t.queue_order((72, -10), -10)]
    assert xs == [70, 50, 30]


def test_state_transitions_leave_queue():
    t = TargetTracker(min_hits=1)
    tr = t.add_hit(0, 40, -30, 9)
    t.set_state(tr.id, ACTIVE)
    assert t.pick_next((0, 0), -10, True) is None
    assert t.unfinished_count() == 1
    t.set_state(tr.id, CONFIRMED, 'x')
    assert t.unfinished_count() == 0


def test_snapshot_json():
    t = TargetTracker(min_hits=1)
    t.add_hit(0, 40, -30, 9)
    snap = t.snapshot((10, 0), -10)
    json.dumps(snap)
    assert snap[0]['rank'] == 1 and snap[0]['dist_m'] > 30


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
