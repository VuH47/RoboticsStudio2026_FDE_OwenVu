#!/usr/bin/env python3
"""Look-image names. No ROS."""

import os
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts', 'husky_path_log.py')
if not os.path.exists(_path):
    _path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'husky_path_log.py')
_lines = []
with open(_path, encoding='utf-8') as handle:
    for line in handle:
        if line.startswith('import json'):
            break
        _lines.append(line)
_ns = {'__file__': _path}
exec(''.join(_lines), _ns)
look_filename = _ns['look_filename']
result_filename = _ns['result_filename']
_default_output_root = _ns['_default_output_root']


def test_look_name_is_the_hazard():
    assert look_filename('pin_T01_2') == 'T01_under_determination.png'
    assert look_filename('pin_T04_1') == 'T04_under_determination.png'


def test_result_name_is_verified_or_not():
    assert result_filename('pin_T01_2', 'CONFIRMED') == 'T01_verified.png'
    assert result_filename('pin_T02_1', 'NOT_CONFIRMED') == 'T02_cannot_verify.png'
    assert result_filename('pin_T03_1', 'INCONCLUSIVE') == 'T03_cannot_verify.png'


def test_default_output_root():
    root = _default_output_root()
    assert root.endswith(os.path.join('image', 'path_log'))
    assert os.path.isabs(root)


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        fn()
        print('ok  ', fn.__name__)
    print('%d tests passed' % len(tests))
