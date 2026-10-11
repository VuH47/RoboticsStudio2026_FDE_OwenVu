#!/usr/bin/env python3
"""Pure OpenCV/geometry helpers for Husky's independent RGB ice inspection.

No ROS imports: this module can be tested with synthetic or saved camera frames.
The expected world position determines the ROI ONLY; pixels determine positives.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class VisionConfig:
    hsv_low: Tuple[int, int, int] = (80, 28, 90)
    hsv_high: Tuple[int, int, int] = (125, 240, 255)
    half_width_px: int = 100
    half_height_px: int = 75
    min_area_px: float = 25.0
    max_roi_area_fraction: float = 0.30
    max_aspect_ratio: float = 10.0
    max_centre_offset_fraction: float = 0.80


def rotate_quaternion(q: Tuple[float, float, float, float], point: Tuple[float, float, float]):
    """Rotate a vector by a normalised (x,y,z,w) quaternion."""
    x, y, z, w = [float(v) for v in q]
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm <= 1e-12:
        raise ValueError('Invalid zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    px, py, pz = point
    # q * point * conjugate(q), expressed with cross products.
    uv = np.cross(np.array([x, y, z]), np.array([px, py, pz]))
    uuv = np.cross(np.array([x, y, z]), uv)
    return tuple(float(v) for v in np.array([px, py, pz]) + 2.0 * (w * uv + uuv))


def project_point(
    xyz_source: Tuple[float, float, float],
    translation: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
    intrinsics: Tuple[float, float, float, float],
) -> Optional[Tuple[float, float]]:
    """Project source-frame point via TF source->optical transform and K.

    Optical coordinates: +X image-right, +Y image-down, +Z forward.
    """
    x, y, z = rotate_quaternion(quaternion, xyz_source)
    tx, ty, tz = translation
    x += tx
    y += ty
    z += tz
    if not all(math.isfinite(v) for v in (x, y, z)) or z <= 0.1:
        return None
    fx, fy, cx, cy = intrinsics
    if fx <= 0.0 or fy <= 0.0:
        return None
    return (fx * x / z + cx, fy * y / z + cy)


def quaternion_to_matrix(q: Tuple[float, float, float, float]) -> np.ndarray:
    """3x3 rotation matrix of a (x, y, z, w) quaternion (normalised first)."""
    x, y, z, w = [float(v) for v in q]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError('Invalid zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_to_rpy(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """(roll, pitch, yaw) of a quaternion, ZYX convention (REP-103)."""
    x, y, z, w = [float(v) for v in q]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError('Invalid zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Positive pitch = nose down (REP-103)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def odometry_has_3d(z: float, roll: float, pitch: float) -> bool:
    """False when odometry is the planar kind (z, roll and pitch all exactly 0).

    A real base_link never sits at exactly z=0, so an all-zero reading means the
    OdometryPublisher is still in its default 2D mode.
    """
    return not (abs(z) < 1e-12 and abs(roll) < 1e-12 and abs(pitch) < 1e-12)


def project_point_with_pose(
    xyz_odom: Tuple[float, float, float],
    base_translation: Tuple[float, float, float],
    base_quaternion: Tuple[float, float, float, float],
    base_z: float,
    base_roll: float,
    base_pitch: float,
    optical_in_base_translation: Tuple[float, float, float],
    optical_in_base_quaternion: Tuple[float, float, float, float],
    intrinsics: Tuple[float, float, float, float],
) -> Optional[Tuple[float, float]]:
    """Project an odom-frame point with the Husky's real height and tilt.

    The TF tree is planar (EKF two_d_mode): it places base_link at z=0 with no
    roll or pitch, so on a slope the camera is drawn too low and the target
    appears too high. Here the horizontal part of the pose (x, y, yaw) is kept
    from TF, because the targets are expressed in that same frame. Only
    height, roll and pitch are replaced by the measured values.

    base_translation/base_quaternion: base_link pose in the odom frame (TF).
    optical_in_base_*: static camera optical frame pose in base_link (TF).
    Returns pixel (u, v) or None if the point is behind the camera.
    """
    yaw = quaternion_to_rpy(base_quaternion)[2]
    r_base = matrix_from_rpy(base_roll, base_pitch, yaw)
    t_base = np.array([base_translation[0], base_translation[1], base_z], dtype=float)
    r_opt = r_base @ quaternion_to_matrix(optical_in_base_quaternion)
    t_opt = r_base @ np.array(optical_in_base_translation, dtype=float) + t_base
    x, y, z = r_opt.T @ (np.array(xyz_odom, dtype=float) - t_opt)
    if not all(math.isfinite(float(v)) for v in (x, y, z)) or z <= 0.1:
        return None
    fx, fy, cx, cy = intrinsics
    if fx <= 0.0 or fy <= 0.0:
        return None
    return (float(fx * x / z + cx), float(fy * y / z + cy))


def analyse_rgb_frame(
    image_bgr: np.ndarray,
    predicted_px: Tuple[float, float],
    config: VisionConfig = VisionConfig(),
) -> Dict[str, object]:
    """Analyse REAL BGR pixels in a target-constrained ROI.

    Returns positive=False for an empty target ROI. The predicted location
    alone can never produce a positive classification.
    """
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('Expected H x W x 3 BGR image')
    height, width = image_bgr.shape[:2]
    u, v = predicted_px
    if not math.isfinite(u) or not math.isfinite(v):
        return {'valid': False, 'positive': False, 'reason': 'invalid_projection'}
    x0, x1 = max(0, int(u - config.half_width_px)), min(width, int(u + config.half_width_px))
    y0, y1 = max(0, int(v - config.half_height_px)), min(height, int(v + config.half_height_px))
    if x1 <= x0 or y1 <= y0:
        return {'valid': False, 'positive': False, 'reason': 'target_outside_image'}
    roi = image_bgr[y0:y1, x0:x1]
    if roi.size == 0:
        return {'valid': False, 'positive': False, 'reason': 'empty_roi'}
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(config.hsv_low, np.uint8), np.array(config.hsv_high, np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_area = float(roi.shape[0] * roi.shape[1])
    best = None
    best_score = -1.0
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not config.min_area_px <= area <= config.max_roi_area_fraction * roi_area:
            continue
        bx, by, bw, bh = cv2.boundingRect(contour)
        if bw < 5 or bh < 4 or max(bw / bh, bh / bw) > config.max_aspect_ratio:
            continue
        centre = (x0 + bx + bw / 2.0, y0 + by + bh / 2.0)
        du = (centre[0] - u) / max(1.0, config.half_width_px)
        dv = (centre[1] - v) / max(1.0, config.half_height_px)
        displacement = math.hypot(du, dv)
        if displacement > config.max_centre_offset_fraction:
            continue
        # Prefer larger blobs near the independent UAV-projected point.
        score = area / (1.0 + displacement)
        if score > best_score:
            best_score = score
            best = {
                'bbox': (x0 + bx, y0 + by, bw, bh),
                'area_px': area,
                'centre_px': centre,
                'offset_roi_norm': round(displacement, 3),
            }
    return {
        'valid': True,
        'positive': best is not None,
        'reason': 'ice_coloured_pixels' if best is not None else 'no_valid_ice_pixels',
        'roi': (x0, y0, x1, y1),
        'candidate': best,
    }


def multiframe_verdict(positive_frames: int, valid_frames: int, frames_required: int = 5, positives_required: int = 4) -> Optional[str]:
    """Return a visual verdict only after enough actual valid images are processed."""
    if frames_required <= 0 or positives_required <= 0 or positives_required > frames_required:
        raise ValueError('Invalid frame voting thresholds')
    if valid_frames < frames_required:
        return None
    if not 0 <= positive_frames <= valid_frames:
        raise ValueError('Inconsistent image counts')
    return 'CONFIRMED' if positive_frames >= positives_required else 'NOT_CONFIRMED'

