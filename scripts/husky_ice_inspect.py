#!/usr/bin/env python3
"""Husky ice-cable inspection 

Drives Nav2 along the ski corridor (y=0), yaws toward the alpine lift (−Y),
saves one RGB frame per span, and logs 3D clearance to the cables.

Husky cannot reach the cables (they sit ~6.5–10 m up at y≈−30). Clearance
logging is the safety evidence; a corridor violation is y < −12.

Prereq: sim + Nav2, and optionally ice_scenario.launch.py.

  ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \\
    world:=snow_forest slam:=true nav2:=true rviz:=true
  ros2 launch 41068_ignition_bringup ice_scenario.launch.py
  ros2 launch 41068_ignition_bringup husky_ice_inspect.launch.py
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    corridor_violation,
    husky_inspect_waypoints,
    min_cable_clearance,
)


Waypoint = Tuple[str, float, float, float]


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _save_ppm(path: Path, msg: Image) -> None:
    w, h = int(msg.width), int(msg.height)
    data = bytes(msg.data)
    step = int(msg.step) if msg.step else w * 3
    rows = []
    enc = (msg.encoding or "rgb8").lower()
    for r in range(h):
        row = data[r * step : r * step + w * 3]
        if enc in ("bgr8", "8uc3"):
            buf = bytearray(row)
            for i in range(0, len(buf), 3):
                buf[i], buf[i + 2] = buf[i + 2], buf[i]
            row = bytes(buf)
        rows.append(row)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        handle.write(("P6\n%d %d\n255\n" % (w, h)).encode("ascii"))
        handle.write(b"".join(rows))


class HuskyIceInspect(Node):
    def __init__(self) -> None:
        super().__init__("husky_ice_inspect")
        self.declare_parameter("robot_name", "husky1")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("camera_topic", "camera/image")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("goal_pause_seconds", 2.0)
        self.declare_parameter("image_dir", "")
        self.declare_parameter("envelope_y", -12.0)

        robot = str(self.get_parameter("robot_name").value).strip() or "husky1"
        self.map_frame = "%s_map" % robot
        self.pause_s = float(self.get_parameter("goal_pause_seconds").value)
        self.envelope_y = float(self.get_parameter("envelope_y").value)
        raw_dir = str(self.get_parameter("image_dir").value).strip()
        self.image_dir = (
            Path(raw_dir).expanduser()
            if raw_dir
            else Path.home() / ".ros" / "ice_inspect_husky"
        )
        self.image_dir.mkdir(parents=True, exist_ok=True)

        self.waypoints: List[Waypoint] = husky_inspect_waypoints()
        self.index = 0
        self.goal_active = False
        self.mission_done = False
        self.next_goal_time = 0.0
        self._last_image: Optional[Image] = None
        self._last_odom: Optional[Odometry] = None
        self.results: List[str] = []
        self._min_clearance = float("inf")
        self._violations = 0

        self.create_subscription(
            Image,
            self.get_parameter("camera_topic").value,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            20,
        )
        self.clearance_pub = self.create_publisher(
            Float32, "/ice/eval/cable_clearance", 10
        )
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("navigate_action").value,
        )
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            "Husky ice inspect in %s. Waypoints: %s  images → %s"
            % (self.map_frame, [w[0] for w in self.waypoints], self.image_dir)
        )

    def _on_image(self, msg: Image) -> None:
        self._last_image = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        p = msg.pose.pose.position
        dist, span, cable = min_cable_clearance(p.x, p.y, p.z)
        self._min_clearance = min(self._min_clearance, dist)
        self.clearance_pub.publish(Float32(data=float(dist)))
        if corridor_violation(p.y) or p.y < self.envelope_y:
            self._violations += 1
            self.get_logger().warn(
                "CORRIDOR VIOLATION y=%.2f (envelope %.1f) clearance=%.2f m (%s %s)"
                % (p.y, self.envelope_y, dist, span, cable)
            )

    def _tick(self) -> None:
        if self.mission_done:
            return
        if not self.nav_client.server_is_ready():
            return
        now = time.monotonic()
        if self.goal_active:
            return
        if now < self.next_goal_time:
            return
        if self.index >= len(self.waypoints):
            self._finish()
            return
        self._send_current()

    def _send_current(self) -> None:
        label, x, y, yaw = self.waypoints[self.index]
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_active = True
        self.get_logger().info("NAV %s → (%.2f, %.2f) yaw=%.2f" % (label, x, y, yaw))
        send = self.nav_client.send_goal_async(goal)
        send.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            label = self.waypoints[self.index][0]
            self.results.append("NAV %s: REJECTED" % label)
            self.get_logger().error("NAV %s rejected" % label)
            self.goal_active = False
            self.index += 1
            self.next_goal_time = time.monotonic() + self.pause_s
            return
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        label = self.waypoints[self.index][0]
        wrap = future.result()
        status = wrap.status if wrap is not None else GoalStatus.STATUS_UNKNOWN
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.results.append("NAV %s: %s" % (label, "PASS" if ok else "FAIL"))
        if ok:
            self._capture(label)
        self.goal_active = False
        self.index += 1
        self.next_goal_time = time.monotonic() + self.pause_s

    def _capture(self, label: str) -> None:
        img = self._last_image
        if img is None:
            self.results.append("IMG %s: FAIL (no camera)" % label)
            self.get_logger().warn("No camera image at %s" % label)
            return
        path = self.image_dir / ("%s.ppm" % label)
        try:
            _save_ppm(path, img)
            self.results.append("IMG %s: PASS %s" % (label, path))
            self.get_logger().info("Saved %s" % path)
        except OSError as exc:
            self.results.append("IMG %s: FAIL (%s)" % (label, exc))

    def _finish(self) -> None:
        self.mission_done = True
        clearance = self._min_clearance if math.isfinite(self._min_clearance) else -1.0
        summary = {
            "waypoints": self.results,
            "min_cable_clearance_m": clearance,
            "corridor_violations": self._violations,
            "pass": all("FAIL" not in r and "REJECTED" not in r for r in self.results)
            and self._violations == 0,
        }
        out = self.image_dir / "inspect_summary.json"
        out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info("INSPECT SUMMARY %s" % json.dumps(summary))
        print("===== HUSKY ICE INSPECT =====")
        for line in self.results:
            print(line)
        print(
            "min cable clearance: %.2f m   corridor violations: %d"
            % (clearance, self._violations)
        )
        print("wrote", out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HuskyIceInspect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
