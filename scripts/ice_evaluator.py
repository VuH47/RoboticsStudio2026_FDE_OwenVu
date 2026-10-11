#!/usr/bin/env python3
"""Score ice detections against the scenario ground-truth file (Phase 6).

The RGB detector must NOT subscribe to this node and this node does not
publish ice positions. It only reads files:

  truth_path       default ~/.ros/ice_ground_truth.json  (from ice_scenario_manager)
  detections_path  optional JSON written by the perception teammate

Detection JSON shape (list or {\"detections\": [...]}):
  {\"span_id\": \"span_01\", \"position\": [x, y, z], \"confidence\": 0.8}

Without detections_path the node validates truth structure and seed replay only.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import sample_layout  # noqa: E402


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _as_detections(raw: Any) -> List[Dict[str, Any]]:
    if isinstance(raw, dict):
        raw = raw.get("detections", raw.get("blocks", []))
    if not isinstance(raw, list):
        raise ValueError("detections JSON must be a list or {detections: [...]}")
    return [d for d in raw if isinstance(d, dict)]


def _xyz(rec: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    pos = rec.get("position")
    if isinstance(pos, (list, tuple)) and len(pos) >= 3:
        return float(pos[0]), float(pos[1]), float(pos[2])
    return None


def match_detections(
    truth_blocks: List[Dict[str, Any]],
    detections: List[Dict[str, Any]],
    max_dist: float,
) -> Dict[str, Any]:
    used_det = set()
    tp = 0
    span_ok = 0
    for block in truth_blocks:
        tpos = _xyz(block)
        if tpos is None:
            continue
        best_i = None
        best_d = max_dist
        for i, det in enumerate(detections):
            if i in used_det:
                continue
            dpos = _xyz(det)
            if dpos is None:
                continue
            dist = math.sqrt(
                (tpos[0] - dpos[0]) ** 2
                + (tpos[1] - dpos[1]) ** 2
                + (tpos[2] - dpos[2]) ** 2
            )
            if dist < best_d:
                best_d = dist
                best_i = i
        if best_i is None:
            continue
        used_det.add(best_i)
        tp += 1
        det = detections[best_i]
        if det.get("span_id") == block.get("span_id"):
            span_ok += 1
    fp = len(detections) - len(used_det)
    fn = len(truth_blocks) - tp
    return {
        "true_positives": tp,
        "false_positives": fp,
        "false_negatives": fn,
        "span_correct": span_ok,
        "truth_count": len(truth_blocks),
        "detected_count": len(detections),
    }


class IceEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("ice_evaluator")
        self.declare_parameter("truth_path", "")
        self.declare_parameter("detections_path", "")
        self.declare_parameter("match_distance", 2.5)
        self.declare_parameter("run_once", True)

        if bool(self.get_parameter("run_once").value):
            self.create_timer(0.2, self._run_once)
            self._done = False
        else:
            self._done = True
            self.evaluate()

    def _truth_path(self) -> Path:
        raw = str(self.get_parameter("truth_path").value).strip()
        if raw:
            return Path(raw).expanduser()
        return Path.home() / ".ros" / "ice_ground_truth.json"

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self.evaluate()
        rclpy.shutdown()

    def evaluate(self) -> None:
        path = self._truth_path()
        if not path.is_file():
            self.get_logger().error("No truth file at %s — run ice_scenario_manager first." % path)
            return
        payload = _load_json(path)
        blocks = payload.get("blocks", [])
        seed = int(payload.get("seed", 0))
        replay = sample_layout(
            seed,
            len(blocks),
            min_spacing=float(payload.get("min_spacing", 3.0)),
            severities=payload.get("severities"),
            span_ids=payload.get("span_ids"),
            cable_ids=payload.get("cable_ids"),
            one_per_span=bool(payload.get("one_per_span", True)),
        )
        replay_ids = [r["id"] for r in replay]
        truth_ids = [r["id"] for r in blocks]
        replay_ok = replay_ids == truth_ids
        for a, b in zip(replay, blocks):
            if a["span_id"] != b["span_id"] or a["cable_id"] != b["cable_id"]:
                replay_ok = False

        summary = {
            "truth_path": str(path),
            "seed": seed,
            "truth_count": len(blocks),
            "seed_replay_ok": replay_ok,
        }

        det_path = str(self.get_parameter("detections_path").value).strip()
        if det_path:
            dpath = Path(det_path).expanduser()
            if not dpath.is_file():
                self.get_logger().error("detections_path missing: %s" % dpath)
                return
            detections = _as_detections(_load_json(dpath))
            max_d = float(self.get_parameter("match_distance").value)
            summary.update(match_detections(blocks, detections, max_d))
            summary["pass"] = (
                summary["false_negatives"] == 0
                and summary["false_positives"] == 0
                and replay_ok
            )
        else:
            summary["note"] = (
                "truth-only: perception teammate has not supplied detections_path"
            )
            summary["pass"] = replay_ok and len(blocks) > 0

        self.get_logger().info("ICE EVAL %s" % json.dumps(summary))
        print(json.dumps(summary, indent=2))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = IceEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
