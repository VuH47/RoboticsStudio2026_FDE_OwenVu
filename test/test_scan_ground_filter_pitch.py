#!/usr/bin/env python3
"""Does scan_ground_filter still erase ground hits once /husky1/odom carries real pitch?

Before the EKF went 3D the filter always saw pitch = 0. Now it sees the true
pitch. This casts the 360 lidar beams (same mount as husky.urdf.xacro) onto a
ground plane in the robot frame and counts the ground hits the filter would
keep as obstacles ("phantom walls"), for pitch = real versus pitch = 0.

Convention: ROS pitch > 0 is nose DOWN. A robot climbing a slope of angle
alpha with pitch p sees the ground at slope (alpha + p) in its own frame.

Run: python3 test_scan_ground_filter_pitch.py   (needs rclpy; run in WSL)
"""

import math
import sys

import scan_ground_filter as sgf

RANGE_MAX = 40.0


def make_filter(pitch):
    f = object.__new__(sgf.ScanGroundFilter)
    f.road_gradient = 0.10  # 41068_ignition_husky.launch.py default
    f.height_tolerance = 0.25
    f.max_ground_height = 0.35
    f.lidar_xyz = [0.0, 0.0, 0.85]
    f.lidar_rpy = [0.0, -0.12, 0.0]
    f.ground_z = -0.132
    f._pitch = pitch
    return f


def ground_hits(slope_body, n=360):
    """(range, angle) of every beam that actually hits a ground plane that
    rises with tan(slope_body) along the robot's +x, as seen in the robot frame."""
    ref = make_filter(0.0)
    tan_s = math.tan(slope_body)
    hits = []
    for i in range(n):
        a = i * 2.0 * math.pi / n
        mx, my, mz = sgf._rpy_rotate(*ref.lidar_rpy, math.cos(a), math.sin(a), 0.0)
        denom = mx * tan_s - mz
        if abs(denom) < 1e-9:
            continue
        t = (ref.lidar_xyz[2] - ref.ground_z) / denom
        if 0.3 < t < RANGE_MAX:
            hits.append((t, a))
    return hits


def phantom_walls(alpha, p, filter_pitch):
    """Ground beams kept as obstacles. alpha = world slope, p = true robot pitch."""
    f = make_filter(filter_pitch)
    hits = ground_hits(alpha + p)
    kept = [(r, a) for r, a in hits if not f._is_ground(r, a)]
    return len(hits), kept


SCENARIOS = [
    ('flat ground, level robot', 0.0, 0.0),
    ('base of ramp, ramp ahead, level robot', math.radians(4.0), 0.0),
    ('steady climb on 4 deg ramp', math.radians(4.0), -math.radians(4.0)),
    ('front wheels on ramp (half pitch)', math.radians(4.0), -math.radians(2.0)),
    ('crest, flat ground ahead, still nose up', 0.0, -math.radians(4.0)),
    ('descending 4 deg ramp', -math.radians(4.0), math.radians(4.0)),
]


def main():
    bad = 0
    print('%-44s %5s %12s %12s' % ('scenario', 'hits', 'walls p=real', 'walls p=0'))
    for name, alpha, p in SCENARIOS:
        n, real = phantom_walls(alpha, p, p)
        _, zero = phantom_walls(alpha, p, 0.0)
        print('%-44s %5d %12d %12d' % (name, n, len(real), len(zero)))
        if len(real) > len(zero):
            bad += 1
            close = sorted(real)[:3]
            print('   real-pitch walls at (range m, angle deg):',
                  [(round(r, 1), round(math.degrees(a))) for r, a in close])
    crest = SCENARIOS[4]
    assert len(phantom_walls(crest[1], crest[2], crest[2])[1]) == 0, 'crest still paints walls'
    assert len(phantom_walls(crest[1], crest[2], 0.0)[1]) > 0, 'crest case no longer exercises pitch=0'
    descent = SCENARIOS[5]
    assert len(phantom_walls(descent[1], descent[2], descent[2])[1]) == 0, 'descent paints walls'
    # a vertical trunk 8 m ahead / beside must stay an obstacle for any pitch
    for p in (-math.radians(4.0), 0.0, math.radians(4.0)):
        f = make_filter(p)
        for a in (0.0, math.pi / 2, -math.pi / 2):
            assert not f._is_ground(8.0, a), 'trunk erased at pitch %.2f angle %.2f' % (p, a)
    print('trunk beams stay obstacles at pitch -4/0/+4 deg: ok')
    if bad:
        print('REGRESSION: real pitch creates more phantom walls in %d scenario(s)' % bad)
        return 1
    print('real pitch never creates more phantom walls than pitch=0')
    return 0


if __name__ == '__main__':
    sys.exit(main())
