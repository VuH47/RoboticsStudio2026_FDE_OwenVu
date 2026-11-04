#!/usr/bin/env python3
"""Viewing-pose rule. No ROS.

Run:  py -3 scripts/test_husky_pin_view.py
"""

import math
from pathlib import Path

# The mission module imports ROS. Load only the pose rule above that import.
_pin_view_path = Path(__file__).resolve().parent.parent / 'scripts' / 'husky_pin_view.py'
if not _pin_view_path.exists():
    _pin_view_path = Path(__file__).with_name('husky_pin_view.py')
_src = _pin_view_path.read_text(encoding='utf-8')
_head = _src.split('import rclpy', 1)[0]
_ns = {}
exec(compile(_head, 'husky_pin_view.py', 'exec'), _ns)
viewing_pose = _ns['viewing_pose']
drive_steps = _ns['drive_steps']
aim_yaw = _ns['aim_yaw']
world_to_map = _ns['world_to_map']
TourGate = _ns['TourGate']
tour_may_start = _ns['tour_may_start']
ice_place_succeeded = _ns['ice_place_succeeded']
step_action = _ns['step_action']
arrival_xy = _ns['arrival_xy']
needs_reverse_before_turn = _ns['needs_reverse_before_turn']
ROOF = (-8.0, 0.0, 4.0)


def test_standoff_ignores_height_and_faces_the_pin():
    x, y, yaw, dist = viewing_pose(63.8, -30.0)
    assert abs(x - 63.8) < 1e-9
    # Option B empirical standoff 16.0 m: -30.0 + 16.0 = -14.0 m
    assert abs(y - (-14.0)) < 1e-9
    assert abs(dist - 16.0) < 1e-9
    assert abs(yaw + math.pi / 2.0) < 1e-9


def test_exact_standoff_is_not_clamped():
    x, y, yaw, dist = viewing_pose(40.0, -29.25)
    assert abs(x - 40.0) < 1e-9
    # Option B empirical standoff 16.0 m: -29.25 + 16.0 = -13.25 m
    assert abs(y - (-13.25)) < 1e-6
    assert abs(dist - 16.0) < 1e-6
    assert yaw < 0.0


def test_long_view_is_split_into_five_metre_steps():
    steps = drive_steps(0.0, 0.0, 32.9, -8.0, -math.pi / 2.0, 5.0)
    assert len(steps) >= 6
    assert abs(steps[-1][0] - 32.9) < 1e-6
    assert abs(steps[-1][1] - (-8.0)) < 1e-6
    assert abs(steps[-1][2] + math.pi / 2.0) < 1e-6
    prev = (0.0, 0.0)
    for x, y, _yaw in steps:
        assert math.hypot(x - prev[0], y - prev[1]) <= 5.0 + 1e-6
        prev = (x, y)


def test_far_pin_stays_on_the_lane_edge():
    _x, y, _yaw, dist = viewing_pose(50.0, -40.0)
    # Option B: Clamped to LANE_NEAR_CABLE_Y = -22.0 m
    assert abs(y - (-22.0)) < 1e-9
    assert dist > 16.0


def test_aim_yaw_points_directly_at_pin_from_slightly_offset_stand():
    yaw = aim_yaw(39.1, -8.9, 40.1, -29.3)
    assert -math.pi / 2.0 < yaw < -1.45
    assert abs(aim_yaw(40.0, -10.0, 40.0, -30.0) + math.pi / 2.0) < 1e-9


def test_tour_stays_down_without_odometry():
    gate = TourGate()
    assert tour_may_start(gate, 1.0, None, True) is False
    assert tour_may_start(gate, 4.0, None, True) is False


def test_tour_stays_down_at_the_origin():
    gate = TourGate()
    origin = (0.0, 0.0, 0.0)
    assert tour_may_start(gate, 1.0, origin, True) is False
    assert tour_may_start(gate, 5.0, origin, True) is False


def test_tour_stays_down_until_this_run_places_ice():
    gate = TourGate()
    assert tour_may_start(gate, 1.0, ROOF, False) is False
    assert tour_may_start(gate, 4.0, ROOF, False) is False
    assert ice_place_succeeded('placed 3/4 seed=1 (truth file not updated)') is False
    assert ice_place_succeeded('placement_failed') is False
    assert ice_place_succeeded('placed 4/4 seed=1 truth=/tmp/ice.json') is True


def test_tour_starts_after_steady_roof_pose_and_placement():
    gate = TourGate()
    assert tour_may_start(gate, 10.0, ROOF, True) is False
    assert tour_may_start(gate, 11.0, ROOF, True) is False
    assert tour_may_start(gate, 12.0, ROOF, True) is True


def test_detour_cell_inside_goal_tolerance_aligns_without_nav2():
    # Video stall: 0.96 m from the free cell, still ~2.2 m from the blocked
    # view line at y=-8. That must align and must not build NavigateToPose.
    free = (35.2, -6.0)
    blocked = (35.2, -8.0)
    hx, hy = 34.24, -6.0
    assert abs(math.hypot(hx - free[0], hy - free[1]) - 0.96) < 1e-9
    assert math.hypot(hx - blocked[0], hy - blocked[1]) > 2.1
    assert step_action(hx, hy, free) == 'align'
    assert step_action(34.0, -6.0, free) == 'send'
    assert step_action(30.4, -10.7, free, moved_m=0.05) == 'align'
    assert step_action(hx, hy, free, moved_m=0.96) == 'send'
    assert arrival_xy(blocked, free) == free
    assert arrival_xy(blocked, None) == blocked


def test_world_to_map_preserves_step_distance_under_slam_drift():
    # Husky at (31.9, -6.8, yaw=0.0) in Gazebo world, but SLAM map has drifted
    # to (29.7, -7.45, yaw=0.12) in husky1_map.
    husky_world = (31.9, -6.8, 0.0, 0.0)
    husky_map = (29.7, -7.45, 0.12)
    wx, wy, wyaw = (35.7, -7.0, -0.05)
    mx, my, myaw = world_to_map(wx, wy, wyaw, husky_world, husky_map)
    world_dist = math.hypot(wx - husky_world[0], wy - husky_world[1])
    map_dist = math.hypot(mx - husky_map[0], my - husky_map[1])
    assert abs(world_dist - map_dist) < 1e-9
    assert abs((myaw - husky_map[2]) - (wyaw - husky_world[3])) < 1e-9


def test_needs_reverse_before_turn_when_obstacle_ahead():
    # Obstacle at (37.0, -7.0). Robot at (36.0, -7.0) facing east (yaw=0.0).
    # Needs to turn south (-pi/2).
    def cost(x, y):
        if math.hypot(x - 37.0, y - (-7.0)) <= 0.6:
            return 254
        return 0

    assert needs_reverse_before_turn(36.0, -7.0, 0.0, -math.pi / 2.0, cost, min_front_clear=1.3) is True
    # If space ahead is clear:
    assert needs_reverse_before_turn(36.0, -7.0, 0.0, -math.pi / 2.0, lambda _x, _y: 0, min_front_clear=1.3) is False
    # If robot is already aligned (err <= 0.15), no turn is needed:
    assert needs_reverse_before_turn(36.0, -7.0, -math.pi / 2.0, -math.pi / 2.0, cost, min_front_clear=1.3) is False


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        fn()
        print('ok  ', fn.__name__)
    print('%d tests passed' % len(tests))
