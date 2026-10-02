#!/usr/bin/env python3
"""Seeded ice-block placement on alpine_trail cables.

Preloaded models ice_block_01..08 sit under the world in snow_forest /
snow_ice_cable_test. This node only calls Gazebo ``set_pose`` (same pattern as
dynamic_world_demo.py). Ground truth is written to a file — it is never
published on camera or detector topics.

Prereq: sim already running, e.g.
  ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \\
    world:=snow_forest slam:=true nav2:=true rviz:=true

Then:
  ros2 launch 41068_ignition_bringup ice_scenario.launch.py seed:=42
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    HIDDEN_Z,
    ICE_POOL,
    MIN_SPACING_M,
    rpy_to_quat,
    sample_layout,
)


def _csv_list(raw: str) -> List[str]:
    return [p.strip() for p in str(raw).split(",") if p.strip()]


def _write_yaml(path: Path, payload: Dict[str, Any]) -> None:
    """Minimal YAML (no PyYAML) matching the groknote record shape."""
    lines = [
        "seed: %s" % payload["seed"],
        "block_count: %s" % payload["block_count"],
        "world: %s" % payload["world"],
        "blocks:",
    ]
    for rec in payload["blocks"]:
        pos = rec["position"]
        ori = rec["orientation"]
        size = rec["size"]
        lines.extend(
            [
                "- id: %s" % rec["id"],
                "  span_id: %s" % rec["span_id"],
                "  cable_id: %s" % rec["cable_id"],
                "  t: %.6f" % rec["t"],
                "  severity: %s" % rec["severity"],
                "  position: [%.6f, %.6f, %.6f]" % (pos[0], pos[1], pos[2]),
                "  orientation: [%.6f, %.6f, %.6f]" % (ori[0], ori[1], ori[2]),
                "  size: [%.4f, %.4f, %.4f]" % (size[0], size[1], size[2]),
                "  seed: %s" % rec["seed"],
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class IceScenarioManager(Node):
    """Hide the ice pool, then place a seeded subset on the cables."""

    def __init__(self) -> None:
        super().__init__("ice_scenario_manager")
        self.declare_parameter("world_name", "snow_forest")
        self.declare_parameter("seed", 42)
        self.declare_parameter("block_count", 3)
        self.declare_parameter("min_spacing", MIN_SPACING_M)
        self.declare_parameter("one_per_span", True)
        self.declare_parameter("span_ids", "span_01,span_02,span_03")
        self.declare_parameter("cable_ids", "cable_left,cable_right")
        self.declare_parameter("severities", "small,medium,large")
        self.declare_parameter("truth_path", "")
        self.declare_parameter("place_on_start", True)

        self.world_name = str(self.get_parameter("world_name").value)
        self.gz_command = "ign" if shutil.which("ign") else "gz"
        self.msg_prefix = (
            "ignition.msgs" if self.gz_command == "ign" else "gz.msgs"
        )
        self._records: List[Dict[str, Any]] = []

        self.create_service(Trigger, "/ice/reset", self._on_reset)
        self.create_service(Trigger, "/ice/hide", self._on_hide)

        self.get_logger().info(
            "Ice scenario manager world=%s gz=%s seed=%s count=%s"
            % (
                self.world_name,
                self.gz_command,
                self.get_parameter("seed").value,
                self.get_parameter("block_count").value,
            )
        )
        if bool(self.get_parameter("place_on_start").value):
            self.create_timer(0.5, self._place_once)
            self._started = False
        else:
            self._started = True

    def _place_once(self) -> None:
        if self._started:
            return
        self._started = True
        ok, msg = self.apply_scenario()
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)

    def _on_reset(self, _req: Trigger.Request, resp: Trigger.Response):
        ok, msg = self.apply_scenario()
        resp.success = ok
        resp.message = msg
        return resp

    def _on_hide(self, _req: Trigger.Request, resp: Trigger.Response):
        ok = self.hide_all()
        resp.success = ok
        resp.message = "hidden" if ok else "hide failed"
        self._records = []
        return resp

    def apply_scenario(self) -> tuple:
        seed = int(self.get_parameter("seed").value)
        count = int(self.get_parameter("block_count").value)
        spacing = float(self.get_parameter("min_spacing").value)
        one_per = bool(self.get_parameter("one_per_span").value)
        spans = _csv_list(self.get_parameter("span_ids").value)
        cables = _csv_list(self.get_parameter("cable_ids").value)
        sevs = _csv_list(self.get_parameter("severities").value)
        try:
            records = sample_layout(
                seed=seed,
                block_count=count,
                min_spacing=spacing,
                severities=sevs,
                span_ids=spans,
                cable_ids=cables,
                one_per_span=one_per,
            )
        except (ValueError, RuntimeError) as exc:
            return False, str(exc)

        if not self.hide_all():
            return False, "failed to hide ice pool (is Gazebo running?)"

        placed = 0
        for rec in records:
            pos = rec["position"]
            ori = rec["orientation"]
            if self.set_model_pose(rec["id"], pos[0], pos[1], pos[2], ori):
                placed += 1
            else:
                self.get_logger().warn("set_pose failed for %s" % rec["id"])
        self._records = records
        path = self._write_truth(records, seed)
        return (
            placed == len(records) and placed == count,
            "placed %d/%d seed=%s truth=%s"
            % (placed, count, seed, path),
        )

    def hide_all(self) -> bool:
        ok = True
        for i, (name, _sev) in enumerate(ICE_POOL):
            if not self.set_model_pose(name, float(i) * 3.0, 0.0, HIDDEN_Z, (0.0, 0.0, 0.0)):
                ok = False
        return ok

    def set_model_pose(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        rpy: Iterable[float],
        *,
        warn_on_failure: bool = True,
    ) -> bool:
        roll, pitch, yaw = tuple(rpy)
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
        request = (
            'name: "%s" '
            "position { x: %.6f y: %.6f z: %.6f } "
            "orientation { x: %.6f y: %.6f z: %.6f w: %.6f }"
            % (name, x, y, z, qx, qy, qz, qw)
        )
        return self._run(
            [
                self.gz_command,
                "service",
                "-s",
                "/world/%s/set_pose" % self.world_name,
                "--reqtype",
                "%s.Pose" % self.msg_prefix,
                "--reptype",
                "%s.Boolean" % self.msg_prefix,
                "--timeout",
                "1000",
                "--req",
                request,
            ],
            warn_on_failure=warn_on_failure,
        )

    def _run(
        self,
        command: List[str],
        *,
        timeout: float = 2.0,
        warn_on_failure: bool = True,
    ) -> bool:
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=timeout,
                check=False,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            if warn_on_failure:
                self.get_logger().warn("set_pose helper failed: %s" % exc)
            return False
        if result.returncode != 0:
            if warn_on_failure:
                self.get_logger().warn(
                    "Command failed: %s\nstdout: %s\nstderr: %s"
                    % (
                        " ".join(command),
                        result.stdout.strip(),
                        result.stderr.strip(),
                    )
                )
            return False
        return True

    def _truth_path(self) -> Path:
        raw = str(self.get_parameter("truth_path").value).strip()
        if raw:
            return Path(raw).expanduser()
        return Path.home() / ".ros" / "ice_ground_truth.json"

    def _write_truth(self, records: List[Dict[str, Any]], seed: int) -> Path:
        path = self._truth_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "seed": seed,
            "block_count": len(records),
            "world": self.world_name,
            "one_per_span": bool(self.get_parameter("one_per_span").value),
            "min_spacing": float(self.get_parameter("min_spacing").value),
            "span_ids": _csv_list(self.get_parameter("span_ids").value),
            "cable_ids": _csv_list(self.get_parameter("cable_ids").value),
            "severities": _csv_list(self.get_parameter("severities").value),
            "blocks": records,
        }
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        _write_yaml(path.with_suffix(".yaml"), payload)
        return path


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = IceScenarioManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
