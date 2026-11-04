#!/usr/bin/env python3
"""Offline tests for the blocked-lane detour. No ROS.

Run:  py -3 scripts/test_husky_detour.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from husky_detour import (  # noqa: E402
    ROCK_GATES, choose_stand, classify_step, near_sight_clear, plan_around, plan_free,
    pose_past_block, pose_retreat, segment_blocked, sight_clear,
)

GATE_A = {'x': 32.0, 'y': 0.8, 'half': 1.2, 'open_y': -4.0, 'south': 1.0}
GATE_B = {'x': 56.0, 'y': -0.8, 'half': 1.2, 'open_y': 4.0, 'south': 0.0}
GATES = (GATE_A, GATE_B)


def _hard(x, y, gate):
    return abs(x - gate['x']) <= gate['half'] and abs(y - gate['y']) <= gate['half']


def test_clear_lane_keeps_the_pose_and_heading():
    goals = [(10.0, 0.0, 0.0), (15.0, 0.0, 0.0)]
    out = plan_around((5.0, 0.0), goals, GATES)
    assert out[-1] == (15.0, 0.0, 0.0)
    assert all(abs(y) < 0.1 for _x, y, _yaw in out)


def test_blocked_step_ends_on_the_same_pose_and_heading():
    goal = (40.0, 0.0, 0.0)
    assert segment_blocked(20.0, 0.0, goal[0], goal[1], GATE_A, 1.5)
    out = plan_around((20.0, 0.0), [goal], GATES)
    assert out[-1][0] == 40.0 and out[-1][1] == 0.0
    assert abs(out[-1][2] - 0.0) < 1e-9
    assert any(y < -3.0 for _x, y, _yaw in out[:-1])
    for x, y, _yaw in out[:-1]:
        assert not _hard(x, y, GATE_A)
        assert not _hard(x, y, GATE_B)


def test_arrival_heading_is_kept_when_it_is_not_the_travel_direction():
    goal = (40.0, 0.0, -math.pi / 2.0)
    out = plan_around((20.0, 0.0), [goal], GATES)
    assert abs(out[-1][2] + math.pi / 2.0) < 1e-9
    assert abs(out[-1][0] - 40.0) < 1e-9 and abs(out[-1][1]) < 1e-9


def test_second_gate_passes_on_the_north_side():
    goal = (70.0, 0.0, 0.0)
    out = plan_around((50.0, 0.0), [goal], (GATE_B,))
    assert out[-1] == (70.0, 0.0, 0.0)
    assert any(y > 3.0 for _x, y, _yaw in out[:-1])
    for x, y, _yaw in out[:-1]:
        assert not _hard(x, y, GATE_B)


def test_does_not_turn_back_into_the_rock():
    # The logged failure: after (34.7, -4) the next crumb was (33.5, 0),
    # and the Husky left the detour and hit the rock.
    goals = [(29.0, 0.0, 0.0), (33.5, -0.1, 0.0), (38.0, 0.0, 0.0)]
    out = plan_around((24.0, 0.0), goals, (GATE_A,))
    assert abs(out[-1][0] - 38.0) < 1e-6 and abs(out[-1][1]) < 1e-6
    assert abs(out[-1][2]) < 1e-9
    assert not any(abs(x - 33.5) < 0.4 and y > -2.0 for x, y, _yaw in out)
    # Rejoin is past the rock, on the lane, still facing the travel heading.
    assert any(x >= 34.6 and abs(y) < 0.2 and abs(yaw) < 1e-9 for x, y, yaw in out)
    prev = (24.0, 0.0)
    for x, y, _yaw in out:
        assert not segment_blocked(prev[0], prev[1], x, y, GATE_A, 0.0)
        prev = (x, y)


def test_steps_stay_within_five_metres():
    out = plan_around((0.0, 0.0), [(80.0, 0.0, math.pi)], GATES, step=5.0)
    prev = (0.0, 0.0)
    for x, y, _yaw in out:
        assert math.hypot(x - prev[0], y - prev[1]) <= 5.0 + 1e-6
        prev = (x, y)
    assert out[-1][0] == 80.0 and abs(out[-1][2] - math.pi) < 1e-9


def test_pin_view_lane_rocks_detour_north():
    lane = tuple(g for g in ROCK_GATES if g['x'] in (36.0, 46.0, 52.0))
    assert len(lane) == 3
    goal = (60.0, -8.0, -math.pi / 2.0)
    out = plan_around((30.0, -8.0), [goal], lane, step=4.0)
    assert abs(out[-1][0] - 60.0) < 1e-6
    assert abs(out[-1][1] - (-8.0)) < 1e-6
    assert abs(out[-1][2] - (-math.pi / 2.0)) < 1e-9
    assert any(abs(y - (-3.0)) < 0.2 for _x, y, _yaw in out[:-1])
    for rock in lane:
        for x, y, _yaw in out:
            assert not _hard(x, y, rock)


# V2 pin-view lane centres in snow_forest.sdf.
_LANE_ROCKS = ((36.0, -9.0), (46.0, -8.5), (52.0, -11.0))


def _lane_cost(x, y):
    """0 free, 254 inside a 2.2 m disk, matching rock half plus inflation."""
    for rx, ry in _LANE_ROCKS:
        if math.hypot(x - rx, y - ry) <= 2.2:
            return 254
    return 0


def test_plan_free_shifts_north_of_unlisted_rocks():
    goal = (60.0, -8.0, -math.pi / 2.0)
    out = plan_free((30.0, -8.0), goal, _lane_cost, step=4.0)
    assert out
    assert abs(out[-1][0] - 60.0) < 1e-6
    assert abs(out[-1][1] - (-8.0)) < 1e-6
    assert abs(out[-1][2] - (-math.pi / 2.0)) < 1e-9
    assert any(y >= -6.0 for _x, y, _yaw in out[:-1])
    assert all(y >= -11.0 for _x, y, _yaw in out)
    for rx, ry in _LANE_ROCKS:
        for x, y, _yaw in out:
            assert math.hypot(x - rx, y - ry) > 1.2


def test_unknown_straight_cell_sends():
    # Near frontier unknown is sent so the map can grow.
    decision, pose = classify_step(
        40.0, -8.0, 0.0, lambda x, y: -1, from_xy=(37.0, -8.0))
    assert decision == 'send_unknown' and pose == (40.0, -8.0, 0.0)
    decision, pose = classify_step(
        40.0, -8.0, 0.2, lambda x, y: None, from_xy=(38.0, -8.0))
    assert decision == 'send_unknown' and pose == (40.0, -8.0, 0.2)
    decision, pose = classify_step(40.0, -8.0, 0.0, lambda x, y: 0)
    assert decision == 'send_free' and pose == (40.0, -8.0, 0.0)


def test_far_unknown_waits():
    # Far unknown without a free pass line must not drive into the void.
    decision, pose = classify_step(
        50.0, -8.0, 0.0, lambda x, y: 255, from_xy=(30.0, -8.0))
    assert decision == 'hold_unknown' and pose is None
    # A free pass line north is still preferred when available.
    def cost(_x, y):
        return 0 if abs(y - (-3.0)) < 0.1 else 255

    decision, pose = classify_step(
        50.0, -8.0, 0.0, cost, from_xy=(30.0, -8.0))
    assert decision == 'detour' and pose is not None
    assert abs(pose[1] - (-3.0)) < 1e-9


def test_blocked_step_takes_the_first_free_cell_north():
    def cost(_x, y):
        return 100 if y <= -6.5 else 0

    decision, pose = classify_step(40.0, -8.0, 0.2, cost)
    assert decision == 'detour' and pose is not None
    assert abs(pose[0] - 40.0) < 1e-9
    # LANE_PASS_YS now starts north of lane rocks at y=-5.5 for >= 2.5 m safety margin
    assert abs(pose[1] - (-5.5)) < 1e-9


def test_pass_search_stays_inside_the_lane():
    def cost(_x, y):
        return 0 if abs(y - (-3.0)) < 0.1 else 100

    decision, pose = classify_step(
        40.0, -8.0, 0.0, cost,
        pass_ys=(-14.0, -12.0, -3.0), y_min=-11.0, y_max=2.0)
    assert decision == 'detour' and pose is not None
    assert pose[1] >= -11.0
    assert abs(pose[1] - (-3.0)) < 1e-9


def test_fully_blocked_lane_holds():
    decision, pose = classify_step(40.0, -8.0, 0.0, lambda _x, _y: 100)
    assert decision == 'hold_blocked' and pose is None


def test_mid_segment_rock_forces_detour():
    # End cell is free, but a rock sits halfway — must not send the straight crumb.
    def cost(x, y):
        if abs(x - 42.0) < 0.6 and abs(y - (-8.0)) < 0.6:
            return 254
        return 0

    decision, pose = classify_step(
        44.0, -8.0, 0.0, cost, from_xy=(40.0, -8.0))
    assert decision == 'detour' and pose is not None
    assert abs(pose[1] - (-8.0)) > 0.5
    assert pose[1] >= -24.0
    # Without from_xy the free end cell would wrongly look sendable.
    decision2, pose2 = classify_step(44.0, -8.0, 0.0, cost)
    assert decision2 == 'send_free' and pose2 == (44.0, -8.0, 0.0)


def test_south_lane_stays_south_of_gate_b():
    # If Husky and goal are both south of Gate B (y < -2.0), even if the
    # segment skims the margin box, it must stay south instead of crossing
    # north to open_y=+4.0.
    out = plan_around((45.0, -3.0), [(60.0, -5.0, -math.pi / 2.0)], (GATE_B,))
    assert all(y < -2.5 for _x, y, _yaw in out)
    assert abs(out[-1][0] - 60.0) < 1e-6 and abs(out[-1][1] - (-5.0)) < 1e-6


def _blob(cx, cy, radius):
    def cost(x, y):
        if math.hypot(x - cx, y - cy) <= radius:
            return 254
        return 0
    return cost


def test_free_stand_on_the_pin_ray_is_not_a_view():
    # Occupied disc south of the stand. The footprint at the stand is free.
    cost = _blob(36.0, -9.0, 2.0)
    assert cost(35.8, -7.0) == 0
    assert sight_clear(35.8, -7.0, 35.8, -29.3, cost) is False
    stand = choose_stand(35.8, -7.0, -math.pi / 2.0, 35.8, -29.3, cost)
    assert stand is not None
    # Option B: Either shifted in X or bypassed south of the obstacle in Y
    assert abs(stand[0] - 35.8) > 1.0 or stand[1] <= -11.0
    assert sight_clear(stand[0], stand[1], 35.8, -29.3, cost) is True
    assert cost(stand[0], stand[1]) == 0


def test_clear_costmap_keeps_the_nominal_stand():
    stand = choose_stand(40.0, -8.0, -math.pi / 2.0, 40.0, -30.0, lambda _x, _y: 0)
    assert stand == (40.0, -8.0, -math.pi / 2.0)


def test_blocked_slice_steps_toward_the_goal_instead_of_stopping():
    def cost(x, _y):
        if 37.0 <= x <= 41.0:
            return 254
        return 0
    pose = pose_past_block((64.0, -8.0), (39.1, -7.3), cost, step=4.0)
    assert pose is not None
    assert pose[0] > 41.0
    assert cost(pose[0], pose[1]) == 0


def test_detour_does_not_cut_the_corner_of_a_blocked_rock():
    # Inflated rock the T02 drive clipped. Every north cell at this x still
    # crosses it, so the step must not be sent.
    def cost(x, y):
        if 33.8 <= x <= 38.2 and -11.2 <= y <= -6.8:
            return 254
        return 0
    decision, pose = classify_step(
        36.6, -8.0, 0.0, cost, from_xy=(33.4, -8.1))
    # The cell just north of the rock, (36.6, -6), is the one that cut the
    # corner. It must not be the goal. A wider north pass is allowed.
    assert pose is None or pose[1] > -5.5


def test_retreat_when_forward_cells_cross_the_rock():
    def cost(x, y):
        if x >= 34.0 and y <= -4.0:
            return 254
        return 0
    pose = pose_retreat((33.4, -8.1), (49.6, -8.0), cost)
    assert pose is not None
    assert pose[0] < 33.4
    assert cost(pose[0], pose[1]) == 0


def test_wall_all_the_way_to_the_goal_has_no_past_cell():
    def cost(x, _y):
        if x >= 37.0:
            return 254
        return 0
    assert pose_past_block((64.0, -8.0), (39.1, -7.3), cost, step=4.0) is None


def test_near_sight_clear_detects_obstacle_in_camera_frustum():
    # Rock at (36.0, -8.5), radius 1.2. Camera at (36.0, -7.0) facing south (-pi/2)
    def cost(x, y):
        if math.hypot(x - 36.0, y - 8.5 * (-1.0)) <= 1.2:
            return 254
        return 0

    assert near_sight_clear(36.0, -7.0, -math.pi / 2.0, cost, dist_m=2.5) is False
    # Camera facing away (north) is clear
    assert near_sight_clear(36.0, -7.0, math.pi / 2.0, cost, dist_m=2.5) is True
    # Camera shifted east (39.0, -7.0) looking south is completely clear of the rock
    assert near_sight_clear(39.0, -7.0, -math.pi / 2.0, cost, dist_m=2.5) is True


def test_near_sight_clear_fails_if_stand_itself_is_blocked():
    cost = lambda x, y: 254 if abs(x - 36.0) < 0.1 and abs(y - (-7.0)) < 0.1 else 0
    assert near_sight_clear(36.0, -7.0, 0.0, cost) is False


def test_camera_sight_ignores_inflation_halo_below_occupied_threshold():
    # Issue 56 & Option B: In raw Nav2 Costmap2D, 1..252 is inflation and >= 253 is lethal mesh.
    # In published ROS 2 OccupancyGrid, 1..98 is inflation, 99 is inscribed, 100 is lethal.
    # Repulsive inflation halos (both raw and ROS) must not block vision; physical obstacles must.
    raw_inflation = lambda _x, y: 120 if -9.0 <= y <= -7.0 else 0
    raw_lethal = lambda _x, y: 254 if -9.0 <= y <= -7.0 else 0
    ros_inflation = lambda _x, y: 50 if -9.0 <= y <= -7.0 else 0
    ros_inscribed = lambda _x, y: 99 if -9.0 <= y <= -7.0 else 0
    ros_lethal = lambda _x, y: 100 if -9.0 <= y <= -7.0 else 0

    # Ray across inflation halo is clear
    assert sight_clear(40.1, -3.5, 40.1, -29.25, raw_inflation) is True
    assert sight_clear(40.1, -3.5, 40.1, -29.25, ros_inflation) is True
    # Ray across lethal/inscribed obstacle mesh is blocked
    assert sight_clear(40.1, -3.5, 40.1, -29.25, raw_lethal) is False
    assert sight_clear(40.1, -3.5, 40.1, -29.25, ros_inscribed) is False
    assert sight_clear(40.1, -3.5, 40.1, -29.25, ros_lethal) is False

    # Near sight cone across inflation halo is clear
    assert near_sight_clear(40.1, -5.5, -math.pi / 2.0, raw_inflation, dist_m=2.5) is True
    assert near_sight_clear(40.1, -5.5, -math.pi / 2.0, ros_inflation, dist_m=2.5) is True
    # Near sight cone across lethal/inscribed obstacle mesh is blocked
    assert near_sight_clear(40.1, -5.5, -math.pi / 2.0, raw_lethal, dist_m=2.5) is False
    assert near_sight_clear(40.1, -5.5, -math.pi / 2.0, ros_inscribed) is False
    assert near_sight_clear(40.1, -5.5, -math.pi / 2.0, ros_lethal, dist_m=2.5) is False


def test_detour_prefers_goal_direction_south():
    # When heading to a stand at y = -14.7, a blocked step at y = -8.7 must detour south
    # toward the goal into open snow, rather than being forced north into obstacles.
    def cost(x, y):
        # Obstacle at y = -8.7, clear south (y <= -11.0) and north (y >= -5.5)
        if -10.0 <= y <= -7.0:
            return 100
        return 0

    decision, pose = classify_step(
        36.0, -8.7, 0.0, cost, goal_y=-14.7)
    assert decision == 'detour' and pose is not None
    # Must choose a southern pass line closer to -14.7 (e.g. -14.0 or -12.0), not north (-5.5)
    assert pose[1] <= -10.0
    assert abs(pose[1] - (-14.0)) < 1e-6 or abs(pose[1] - (-12.0)) < 1e-6


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        fn()
        print('ok  ', fn.__name__)
    print('%d tests passed' % len(tests))
