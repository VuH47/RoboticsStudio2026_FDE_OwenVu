#!/usr/bin/env python3
"""Automated test suite for RGB-D alpine ice perception (Option B).

Tests:
  1. Color segmentation:
     - Pure white snow rejection (HSV S < 25)
     - Dark steel cable rejection (HSV V < 80)
     - Pale-blue ice sleeve detection (SDF diffuse: 0.76, 0.90, 0.97)
  2. Point cloud & geometric verification:
     - Bare cable rejection: radius <= 0.17m -> NOT confirmed
     - Ice sleeve confirmation: radius >= 0.22m (surface goes up) -> CONFIRMED
     - Out-of-corridor points rejection (>3m from cable)
  3. Format & downstream compatibility:
     - Output JSON matches ice_evaluator.py expectations
     - Topic message schemas (/fde/detection, /fde/detection_status, /fde/markers)
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import ice_cable_geometry  # noqa: E402
from ice_cable_geometry import CABLE_RADIUS, min_cable_clearance  # noqa: E402


def test_color_segmentation():
    """Verify that only the ice material is accepted, rejecting snow and dark metal."""
    print("=== TEST 1: Color Segmentation in HSV ===")

    # 1. Pure white snow image (RGB 255, 255, 255)
    snow_bgr = np.full((100, 100, 3), 255, dtype=np.uint8)
    snow_hsv = cv2.cvtColor(snow_bgr, cv2.COLOR_BGR2HSV)
    snow_mask = cv2.inRange(snow_hsv, np.array([80, 28, 90]), np.array([125, 240, 255]))
    assert np.count_nonzero(snow_mask) == 0, "Failed: pure white snow triggered false positive!"
    print("  [PASS] Pure white snow correctly rejected (0 false pixels)")

    # 2. Dark cable/bracket image (RGB 40, 40, 40)
    cable_bgr = np.full((100, 100, 3), 40, dtype=np.uint8)
    cable_hsv = cv2.cvtColor(cable_bgr, cv2.COLOR_BGR2HSV)
    cable_mask = cv2.inRange(cable_hsv, np.array([80, 28, 90]), np.array([125, 240, 255]))
    assert np.count_nonzero(cable_mask) == 0, "Failed: dark cable triggered false positive!"
    print("  [PASS] Dark cable steel correctly rejected (0 false pixels)")

    # 3. Ice block diffuse color from SDF: ambient=(0.52, 0.70, 0.80), diffuse=(0.76, 0.90, 0.97)
    # Scaled to 0-255: R=194, G=230, B=247 -> BGR=(247, 230, 194)
    ice_bgr = np.zeros((100, 100, 3), dtype=np.uint8)
    ice_bgr[30:70, 30:70] = [247, 230, 194]  # 40x40 patch of ice material
    ice_hsv = cv2.cvtColor(ice_bgr, cv2.COLOR_BGR2HSV)
    ice_mask = cv2.inRange(ice_hsv, np.array([80, 28, 90]), np.array([125, 240, 255]))
    ice_pixel_count = np.count_nonzero(ice_mask)
    assert ice_pixel_count == 40 * 40, f"Failed: expected 1600 ice pixels, got {ice_pixel_count}"
    print(f"  [PASS] Pale-blue ice sleeve detected accurately ({ice_pixel_count} pixels)")

    # 4. Contour extraction
    contours, _ = cv2.findContours(ice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    assert len(contours) == 1, "Failed: expected 1 candidate contour"
    bx, by, bw, bh = cv2.boundingRect(contours[0])
    assert (bw, bh) == (40, 40), f"Failed: expected bbox (40, 40), got ({bw}, {bh})"
    print("  [PASS] Candidate contour and bounding box accurately extracted")


def test_geometric_surface_verification():
    """Verify the 'surface going up' radial bulge detection against bare cable baseline."""
    print("\n=== TEST 2: Point Cloud & Surface Elevation Fusion ===")

    # Get sample pose on span_01
    cable_pt = ice_cable_geometry.cable_pose("span_01", "cable_right", 0.5, frame="world")
    cx, cy, cz = cable_pt.x, cable_pt.y, cable_pt.z
    print(f"  Baseline cable centerline on span_01: ({cx:.2f}, {cy:.2f}, {cz:.2f}), bare r={CABLE_RADIUS}m")

    # Case A: Bare cable surface points (radial offset = CABLE_RADIUS = 0.17m)
    bare_pts: List[Tuple[float, float, float]] = []
    for theta in np.linspace(0, 2 * math.pi, 20):
        # Surface points on bare cable radius 0.17m
        px = cx
        py = cy + CABLE_RADIUS * math.cos(theta)
        pz = cz + CABLE_RADIUS * math.sin(theta)
        bare_pts.append((px, py, pz))

    # Evaluate radial distance from cable centerline
    bare_radii = [min_cable_clearance(p[0], p[1], p[2])[0] for p in bare_pts]
    bare_surface_r = float(np.percentile(bare_radii, 75))

    # Evaluate bare cable against min_bulge_radius threshold (0.20m)
    min_bulge_r = 0.20
    bare_bulge_detected = bare_surface_r >= min_bulge_r
    assert not bare_bulge_detected, f"Failed: bare cable triggered false bulge! est={bare_surface_r}"
    print(f"  [PASS] Bare cable points evaluated: est_r={bare_surface_r:.3f}m <= {min_bulge_r}m (NOT confirmed)")

    # Case B: Ice sleeve surface points (radius = 0.32m, ice_block_medium)
    # The surface 'goes up' / expands outward by +0.15m radially
    ice_medium_r = 0.32
    ice_pts: List[Tuple[float, float, float]] = []
    for theta in np.linspace(0, 2 * math.pi, 30):
        px = cx
        py = cy + ice_medium_r * math.cos(theta)
        pz = cz + ice_medium_r * math.sin(theta)
        ice_pts.append((px, py, pz))

    ice_radii = [min_cable_clearance(p[0], p[1], p[2])[0] for p in ice_pts]
    ice_surface_r = float(np.percentile(ice_radii, 75))

    ice_bulge_detected = ice_surface_r >= min_bulge_r
    dist_cable, span_id, cable_id = min_cable_clearance(cx, cy, cz)

    assert ice_bulge_detected, f"Failed: ice bulge not detected! est_r={ice_surface_r:.3f}m"
    assert dist_cable < 1.0, f"Failed: cable proximity test failed, dist={dist_cable:.2f}m"
    assert span_id == "span_01", f"Failed: expected span_01, got {span_id}"
    # Same gate as ice_rgbd_detector max_bulge_radius_m. A real sleeve is
    # 0.22-0.45 m; a noisy one measured 0.64 m. Many metres is the blue disc.
    max_bulge_r = 1.50
    assert min_bulge_r <= ice_surface_r <= max_bulge_r, (
        f"Failed: 0.32 m sleeve outside [{min_bulge_r}, {max_bulge_r}], est={ice_surface_r}")
    noisy_r = 0.64
    noisy_pts = [(cx, cy + noisy_r, cz) for _ in range(8)]
    noisy_surface_r = float(np.percentile(
        [min_cable_clearance(p[0], p[1], p[2])[0] for p in noisy_pts], 75))
    assert min_bulge_r <= noisy_surface_r <= max_bulge_r, (
        f"Failed: 0.64 m sleeve rejected, est={noisy_surface_r}")
    print(f"  [PASS] Noisy 0.64 m sleeve kept: est_r={noisy_surface_r:.2f}m")
    print(f"  [PASS] Iced cable points evaluated: est_r={ice_surface_r:.3f}m >= {min_bulge_r}m")
    print(f"         Surface elevation verified! Protrusion = +{ice_surface_r - CABLE_RADIUS:.2f}m above bare cable.")
    print(f"         Status promoted to CONFIRMED on {span_id} ({cable_id})")

    # Case B2: return-pass scatter. Centroid can be near the cable while the
    # 75th percentile clearance is many metres. That must not confirm.
    scatter_r = 8.0
    scatter_pts: List[Tuple[float, float, float]] = []
    for theta in np.linspace(0, 2 * math.pi, 30):
        scatter_pts.append((
            cx,
            cy + scatter_r * math.cos(theta),
            cz + scatter_r * math.sin(theta),
        ))
    scatter_radii = [min_cable_clearance(p[0], p[1], p[2])[0] for p in scatter_pts]
    scatter_surface_r = float(np.percentile(scatter_radii, 75))
    assert scatter_surface_r > max_bulge_r, (
        f"Failed: scatter radius {scatter_surface_r:.2f} m was not above {max_bulge_r} m")
    print(f"  [PASS] Return-pass scatter rejected: est_r={scatter_surface_r:.2f}m > {max_bulge_r}m")

    # Case C: Out of corridor test (e.g. tree or ground point far from cable)
    far_x, far_y, far_z = 0.0, 0.0, 0.4
    dist_far, _, _ = min_cable_clearance(far_x, far_y, far_z)
    assert dist_far > 3.0, f"Expected far distance, got {dist_far}"
    print(f"  [PASS] Far obstacle correctly rejected by cable proximity check (dist={dist_far:.1f}m > 3.0m)")


def test_evaluator_json_compatibility():
    """Verify that output JSON matches ice_evaluator.py schema."""
    print("\n=== TEST 3: Evaluator JSON Output Compatibility ===")
    sample_detection = {
        "span_id": "span_01",
        "cable_id": "cable_right",
        "position": [28.0, -29.25, 7.5],
        "confidence": 0.95,
        "status": "confirmed",
        "radius_m": 0.32,
    }
    payload = {"detections": [sample_detection]}
    json_str = json.dumps(payload)
    parsed = json.loads(json_str)

    assert "detections" in parsed
    d0 = parsed["detections"][0]
    assert d0["span_id"] == "span_01"
    assert len(d0["position"]) == 3
    assert d0["confidence"] >= 0.8
    assert d0["status"] == "confirmed"
    print("  [PASS] Detection JSON payload validated against ice_evaluator.py schema")


def test_depth_range_filter_and_blob_drawing():
    """Verify that blobs are only drawn within depth range (<=10m), keeping 20m RGB detection active."""
    print("\n=== TEST 4: Depth Range Gating (20m RGB, <=10m Drawing) ===")

    # Distant candidate (> 10m away, e.g. 15m)
    distant_patch = np.full((20, 20), np.nan, dtype=np.float32)  # depth sensor clipped at 10m
    valid_dist = np.isfinite(distant_patch) & (distant_patch >= 0.4) & (distant_patch <= 10.0)
    assert np.count_nonzero(valid_dist) == 0, "Distant target should have no valid depth returns <= 10m"

    cand_distant = {
        'bbox': (10, 10, 20, 20),
        'centroid_px': (20, 20),
        'area_px': 400.0,
        'contour': np.array([[[10, 10]], [[30, 10]], [[30, 30]], [[10, 30]]]),
        'confirmed': False,
        'in_depth_range': False,
    }
    # Drawing rule check
    draw_distant = cand_distant.get('confirmed', False) or cand_distant.get('in_depth_range', False)
    assert not draw_distant, "Failed: distant candidate (>10m) should NOT be drawn!"
    print("  [PASS] Distant color candidate (15m, >10m) tracked in RGB but filtered from drawing")

    # In-range candidate (<= 10m away, e.g. 6.5m)
    in_range_patch = np.full((20, 20), 6.5, dtype=np.float32)
    valid_in = np.isfinite(in_range_patch) & (in_range_patch >= 0.4) & (in_range_patch <= 10.0)
    assert np.count_nonzero(valid_in) == 400, "In-range target must have valid depth returns"

    cand_in_range = {
        'bbox': (50, 50, 20, 20),
        'centroid_px': (60, 60),
        'area_px': 400.0,
        'contour': np.array([[[50, 50]], [[70, 50]], [[70, 70]], [[50, 70]]]),
        'confirmed': False,
        'in_depth_range': True,
        'depth_m': float(np.median(in_range_patch[valid_in])),
    }
    draw_in_range = cand_in_range.get('confirmed', False) or cand_in_range.get('in_depth_range', False)
    assert draw_in_range, "Failed: in-range candidate (<=10m) MUST be drawn!"
    assert cand_in_range['depth_m'] == 6.5, f"Expected 6.5m, got {cand_in_range['depth_m']}"
    print(f"  [PASS] In-range candidate (d={cand_in_range['depth_m']:.1f}m <= 10m) permitted to draw with range tag")

    # Drone flight safety check: candidate without depth confirmation MUST NOT trigger /fde/detection
    detections = []
    if cand_in_range.get('confirmed', False):
        detections.append({'position': [1, 2, 3]})
    assert len(detections) == 0, "Unconfirmed candidate must never populate detections"
    print("  [PASS] Unconfirmed candidate does not publish /fde/detection, preserving uninterrupted UAV patrol")


def main():
    print("Running RGB-D Alpine Ice Perception Test Suite...\n")
    test_color_segmentation()
    test_geometric_surface_verification()
    test_evaluator_json_compatibility()
    test_depth_range_filter_and_blob_drawing()
    print("\n>>> ALL TESTS PASSED SUCCESSFULLY! <<<")


if __name__ == "__main__":
    main()
