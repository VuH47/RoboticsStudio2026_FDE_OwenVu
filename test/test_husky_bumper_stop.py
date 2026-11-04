#!/usr/bin/env python3
"""Bumper contact filter. No ROS."""

import os
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'husky_bumper_stop.py')
if not os.path.exists(_path):
    _path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'husky_bumper_stop.py')
_lines = []
with open(_path, encoding='utf-8') as handle:
    for line in handle:
        if line.startswith('import rclpy'):
            break
        _lines.append(line)
_ns = {}
exec(''.join(_lines), _ns)
contact_blocks = _ns['contact_blocks']
bumper_range = _ns['bumper_range']


def test_ground_contact_does_not_stop():
    assert contact_blocks(['husky1::base_link_collision', 'snow_ground::link::collision']) is False
    assert contact_blocks([
        'husky1_front_left_wheel_link::wheel_collision',
        'snow_ground::link::collision',
    ]) is False


def test_rock_contact_stops_and_marks_ahead():
    assert contact_blocks(['husky1::base_link_collision', 'prop_rock_R_lane_1::link::collision']) is True
    assert contact_blocks([
        'husky1_front_left_wheel_link::wheel_collision',
        'prop_rock_R_lane_1::link::collision',
    ]) is True
    assert bumper_range(True) == 0.70
    assert bumper_range(False) == 5.0


def test_empty_contact_does_not_stop():
    assert contact_blocks(['', '']) is False


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        fn()
        print('ok  ', fn.__name__)
    print('%d tests passed' % len(tests))
