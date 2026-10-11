#!/usr/bin/env python3
"""Offline regression tests for real-pixel Husky recognition (no ROS needed)."""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from husky_ice_vision_core import (  # noqa: E402
    VisionConfig, analyse_rgb_frame, multiframe_verdict, project_point,
    rotate_quaternion,
)


ICE_BGR = (247, 230, 194)  # source SDF ice diffuse colour scaled to BGR


def canvas():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:] = (55, 55, 55)
    cv2.line(image, (0, 240), (639, 240), (40, 40, 40), 4)
    return image


def test_medium_ice_in_target_roi_detected_from_image_pixels():
    im = canvas()
    cv2.rectangle(im, (303, 230), (337, 252), ICE_BGR, -1)
    out = analyse_rgb_frame(im, (320, 240))
    assert out['valid'] is True
    assert out['positive'] is True
    assert out['candidate']['area_px'] >= 25


def test_target_geometry_without_cyan_pixels_never_confirms():
    assert analyse_rgb_frame(canvas(), (320, 240))['positive'] is False


def test_snow_white_and_red_object_are_not_ice():
    im = canvas()
    cv2.rectangle(im, (300, 220), (340, 260), (255, 255, 255), -1)
    assert not analyse_rgb_frame(im, (320, 240))['positive']
    cv2.rectangle(im, (300, 220), (340, 260), (0, 0, 255), -1)
    assert not analyse_rgb_frame(im, (320, 240))['positive']


def test_blue_pixels_far_from_predicted_ice_are_rejected():
    im = canvas()
    cv2.rectangle(im, (12, 12), (80, 80), ICE_BGR, -1)
    assert not analyse_rgb_frame(im, (320, 240))['positive']


def test_large_cyan_background_patch_does_not_count_as_sleeve():
    im = canvas()
    cv2.rectangle(im, (228, 168), (410, 313), ICE_BGR, -1)
    assert not analyse_rgb_frame(im, (320, 240))['positive']


def test_target_out_of_image_is_invalid_not_negative_evidence():
    out = analyse_rgb_frame(canvas(), (900, 500))
    assert out['valid'] is False
    assert out['positive'] is False


def test_projection_uses_optical_camera_not_assumed_forward_axis():
    pixel = project_point((0.0, 0.0, 20.0), (0.0, 0.0, 0.0),
                          (0.0, 0.0, 0.0, 1.0), (772.5, 772.5, 320.0, 240.0))
    assert pixel == pytest.approx((320.0, 240.0))
    assert project_point((0.0, 0.0, -2.0), (0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0), (772.5, 772.5, 320, 240)) is None
    turned = rotate_quaternion((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), (1, 0, 0))
    assert turned == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)


def test_four_of_five_frames_required():
    assert multiframe_verdict(4, 4) is None  # insufficient fresh frames
    assert multiframe_verdict(4, 5) == 'CONFIRMED'
    assert multiframe_verdict(3, 5) == 'NOT_CONFIRMED'
    assert multiframe_verdict(0, 5) == 'NOT_CONFIRMED'
    with pytest.raises(ValueError):
        multiframe_verdict(4, 5, 5, 6)


def test_roi_size_is_configurable():
    im = canvas()
    cv2.rectangle(im, (360, 230), (382, 251), ICE_BGR, -1)
    assert analyse_rgb_frame(im, (320, 240), VisionConfig(half_width_px=70))['positive']
    assert not analyse_rgb_frame(im, (320, 240), VisionConfig(half_width_px=20))['positive']


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-q']))

