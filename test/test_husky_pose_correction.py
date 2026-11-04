#!/usr/bin/env python3
"""Offline tests for the Husky pose-corrected ROI projection (no ROS, no pytest).

Run:  python scripts/test_husky_pose_correction.py   (also pytest-compatible)

The camera mounting below is copied from urdf_husky/husky.urdf.xacro:
  base_link -> camera_link   xyz (0.45, 0, 0.25)  rpy (0, -0.35, 0)   (20 deg up)
  camera_link -> rgb_frame   xyz (0.005, 0.018, 0.013)
  rgb_frame -> optical       rpy (-1.57, 0, -1.57)
The span-3 numbers come from the 2026-10-11 18:49 run (seed 2024194827):
  Parrot detected (68.61, -30.69, 9.86); ground truth block (69.12, -30.75, 9.62);
  that run answered NOT_CONFIRMED 0/5 because the ROI sat ~50 px above the ice.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))
from husky_ice_vision_core import (  # noqa: E402
    VisionConfig, analyse_rgb_frame, matrix_from_rpy, odometry_has_3d,
    project_point_with_pose, quaternion_to_matrix, quaternion_to_rpy,
)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


# base_link -> optical, built from the URDF numbers above.
R_CAM = matrix_from_rpy(0.0, -0.35, 0.0)
T_CAM = np.array([0.45, 0.0, 0.25])
T_RGB = np.array([0.005, 0.018, 0.013])
R_OPT_JOINT = matrix_from_rpy(-1.57, 0.0, -1.57)
R_BO = R_CAM @ R_OPT_JOINT
T_BO = R_CAM @ T_RGB + T_CAM
OPT_T = tuple(T_BO)


def quat_from_matrix(m):
    """(x, y, z, w) of a 3x3 rotation matrix (Shepperd's method)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
            (m[1, 0] - m[0, 1]) / s)


OPT_Q = quat_from_matrix(R_BO)

HFOV = 1.309  # 75 deg, 480x360 camera
W, H = 480, 360
FX = (W / 2.0) / math.tan(HFOV / 2.0)
INTR = (FX, FX, W / 2.0, H / 2.0)


def truth_project(target, base_xyz, roll, pitch, yaw):
    """Independent reference: full 3D camera pose built with 4x4 matrices."""
    t_world_base = np.eye(4)
    t_world_base[:3, :3] = matrix_from_rpy(roll, pitch, yaw)
    t_world_base[:3, 3] = base_xyz
    t_base_opt = np.eye(4)
    t_base_opt[:3, :3] = R_BO
    t_base_opt[:3, 3] = T_BO
    t_world_opt = t_world_base @ t_base_opt
    p = np.linalg.inv(t_world_opt) @ np.array([*target, 1.0])
    x, y, z = p[:3]
    return INTR[0] * x / z + INTR[2], INTR[1] * y / z + INTR[3]


def corrected(target, tf_xy, tf_yaw, z, roll, pitch):
    return project_point_with_pose(
        target, (tf_xy[0], tf_xy[1], 0.0), quat_from_rpy(0.0, 0.0, tf_yaw),
        z, roll, pitch, OPT_T, OPT_Q, INTR)


# ---- span-3 scenario ------------------------------------------------------
RAMP_Z = 3.54  # (68.6 - 20) * tan(0.07) + ~0.14 base height
HUSKY_XY = (68.6, -10.0)
HUSKY_YAW = -math.pi / 2  # facing the cable (-y)
DETECTED = (68.61, -30.69, 9.86)  # Parrot pose
ICE_TRUTH = (69.12, -30.75, 9.62)  # evaluation only


def test_rpy_roundtrip_and_matrix():
    for rpy in ((0.0, 0.0, 0.0), (0.1, -0.2, 1.3), (-0.3, 0.25, -2.0)):
        q = quat_from_rpy(*rpy)
        assert np.allclose(quaternion_to_rpy(q), rpy, atol=1e-9)
        assert np.allclose(quaternion_to_matrix(q), matrix_from_rpy(*rpy), atol=1e-9)


def test_flat_ground_matches_independent_projection():
    for yaw in (-math.pi / 2, -math.pi / 2 + 0.25, -math.pi / 2 - 0.3):
        a = corrected(DETECTED, HUSKY_XY, yaw, 0.0, 0.0, 0.0)
        b = truth_project(DETECTED, (HUSKY_XY[0], HUSKY_XY[1], 0.0), 0.0, 0.0, yaw)
        assert a is not None and np.allclose(a, b, atol=1e-6)


def test_corrected_matches_full_3d_pose():
    for roll, pitch in ((0.0, 0.0), (0.03, -0.05), (-0.04, 0.02)):
        a = corrected(DETECTED, HUSKY_XY, HUSKY_YAW, RAMP_Z, roll, pitch)
        b = truth_project(DETECTED, (HUSKY_XY[0], HUSKY_XY[1], RAMP_Z), roll, pitch, HUSKY_YAW)
        assert a is not None and np.allclose(a, b, atol=1e-6)


def test_height_error_is_about_50px_on_span3():
    flat = corrected(DETECTED, HUSKY_XY, HUSKY_YAW, 0.0, 0.0, 0.0)
    fixed = corrected(DETECTED, HUSKY_XY, HUSKY_YAW, RAMP_Z, 0.0, 0.0)
    shift = fixed[1] - flat[1]
    assert 40.0 < shift < 70.0, shift  # flat projection is too HIGH in the image


def test_pitch_sign_nose_down_moves_target_up_in_image():
    level = corrected(DETECTED, HUSKY_XY, HUSKY_YAW, RAMP_Z, 0.0, 0.0)
    down = corrected(DETECTED, HUSKY_XY, HUSKY_YAW, RAMP_Z, 0.0, 0.1)
    assert down[1] < level[1]


def _image_with_ice_at(u, v):
    img = np.full((H, W, 3), 235, dtype=np.uint8)  # snow-white background
    x0, y0 = int(u) - 7, int(v) - 4
    img[y0:y0 + 8, x0:x0 + 14] = (247, 230, 194)  # ice colour (BGR)
    return img


def test_span3_flat_misses_ice_but_corrected_confirms():
    """Reproduces the real 0/5 failure and checks the fix, using the real classifier."""
    cfg = VisionConfig(half_width_px=75, half_height_px=56, min_area_px=14.0)
    true_u, true_v = truth_project(ICE_TRUTH, (HUSKY_XY[0], HUSKY_XY[1], RAMP_Z),
                                   0.0, 0.0, HUSKY_YAW)
    img = _image_with_ice_at(true_u, true_v)
    flat = corrected(DETECTED, HUSKY_XY, HUSKY_YAW, 0.0, 0.0, 0.0)
    fixed = corrected(DETECTED, HUSKY_XY, HUSKY_YAW, RAMP_Z, 0.0, 0.0)
    assert analyse_rgb_frame(img, flat, cfg)['positive'] is False
    assert analyse_rgb_frame(img, fixed, cfg)['positive'] is True


def test_behind_camera_returns_none():
    assert corrected((68.6, 30.0, 5.0), HUSKY_XY, HUSKY_YAW, RAMP_Z, 0.0, 0.0) is None


def test_odometry_has_3d_detects_planar_default():
    assert odometry_has_3d(0.0, 0.0, 0.0) is False
    assert odometry_has_3d(0.14, 0.0, 0.0) is True
    assert odometry_has_3d(0.0, 0.01, 0.0) is True


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print('ok  ', t.__name__)
    print('%d tests passed' % len(tests))
