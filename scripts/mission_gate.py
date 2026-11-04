#!/usr/bin/env python3
"""Operator safety gate between a recommended hazard and Husky motion.

Proposal requirements this node covers:

  R7  No intervention / dispatch without explicit operator approval.
  R8  Abort stops the current action and commands return-to-base.
  R9  Publishes mission state + recommended action for the operator UI.

The drone (or the demo_hazard_stub) publishes a target pose. This node
holds that target in AWAIT_APPROVAL until the operator console fires
``/operator/approve``. Only then is Nav2 given a NavigateToPose goal.
"""

from __future__ import annotations

import json
import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MissionGate(Node):
    """Tiny state machine: IDLE → AWAIT_APPROVAL → NAVIGATING → DONE."""

    def __init__(self) -> None:
        super().__init__('mission_gate')

        self.declare_parameter('robot_name', 'husky1')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('target_topic', '/mission/target')
        self.declare_parameter('label_topic', '/mission/hazard_label')
        self.declare_parameter('state_topic', '/operator/state')
        self.declare_parameter('action_topic', '/operator/recommended_action')
        self.declare_parameter('status_json_topic', '/operator/status_json')
        self.declare_parameter('deploy_x', -15.0)
        self.declare_parameter('deploy_y', 10.0)
        self.declare_parameter('deploy_yaw', math.pi)

        robot = str(self.get_parameter('robot_name').value).strip().strip('/')
        self.robot_name = robot or 'husky1'
        self.map_frame = f'{self.robot_name}_map'

        self.state = 'IDLE'
        self.recommended = 'Waiting for an aerial hazard report.'
        self.hazard_label = ''
        self.target: Optional[PoseStamped] = None
        self.goal_handle = None
        self.last_result = ''

        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10)
        self.action_pub = self.create_publisher(
            String, self.get_parameter('action_topic').value, 10)
        self.json_pub = self.create_publisher(
            String, self.get_parameter('status_json_topic').value, 10)

        self.create_subscription(
            PoseStamped, self.get_parameter('target_topic').value, self._on_target, 10)
        self.create_subscription(
            String, self.get_parameter('label_topic').value, self._on_label, 10)

        self.create_service(Trigger, '/operator/approve', self._on_approve)
        self.create_service(Trigger, '/operator/abort', self._on_abort)
        self.create_service(Trigger, '/operator/hold', self._on_hold)

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('navigate_action').value)
        self.create_timer(0.25, self._publish_status)

        self.get_logger().info(
            f'Mission gate ready for {self.robot_name}. '
            'Approval is required before any Nav2 dispatch.'
        )

    def _on_label(self, msg: String) -> None:
        self.hazard_label = msg.data.strip()

    def _on_target(self, msg: PoseStamped) -> None:
        self.target = msg
        if self.state in ('IDLE', 'DONE', 'AWAIT_APPROVAL', 'HOLD'):
            self.state = 'AWAIT_APPROVAL'
            label = self.hazard_label or 'reported hazard'
            x = msg.pose.position.x
            y = msg.pose.position.y
            self.recommended = (
                f'Dispatch {self.robot_name} to validate {label} '
                f'at ({x:.1f}, {y:.1f}). Awaiting operator approval.'
            )
            self.get_logger().warn(self.recommended)

    def _ok(self, text: str):
        res = Trigger.Response()
        res.success = True
        res.message = text
        return res

    def _fail(self, text: str):
        res = Trigger.Response()
        res.success = False
        res.message = text
        return res

    def _on_approve(self, _req, _res=None):
        if self.state != 'AWAIT_APPROVAL' or self.target is None:
            return self._fail(f'Nothing to approve (state={self.state}).')
        if not self._send_goal(self.target, returning=False):
            return self._fail('Nav2 action server is not ready.')
        self.state = 'NAVIGATING'
        self.recommended = 'Approved. Husky is following the preview path.'
        self.get_logger().info(self.recommended)
        return self._ok(self.recommended)

    def _on_hold(self, _req, _res=None):
        self._cancel_goal()
        self.state = 'HOLD'
        self.recommended = 'Hold. Robots stay put. Re-approve to continue.'
        self.get_logger().warn(self.recommended)
        return self._ok(self.recommended)

    def _on_abort(self, _req, _res=None):
        self._cancel_goal()
        home = PoseStamped()
        home.header.frame_id = self.map_frame
        home.header.stamp = self.get_clock().now().to_msg()
        home.pose.position.x = float(self.get_parameter('deploy_x').value)
        home.pose.position.y = float(self.get_parameter('deploy_y').value)
        qz_w = yaw_to_quat(float(self.get_parameter('deploy_yaw').value))
        home.pose.orientation.z = qz_w[2]
        home.pose.orientation.w = qz_w[3]
        self._send_goal(home, returning=True)
        self.state = 'ABORT_RTB'
        self.recommended = 'Abort. Returning to the deploy station.'
        self.get_logger().error(self.recommended)
        return self._ok(self.recommended)

    def _cancel_goal(self) -> None:
        handle = self.goal_handle
        self.goal_handle = None
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 — keep the gate alive
            self.get_logger().warn(f'Cancel failed: {exc}')

    def _send_goal(self, pose: PoseStamped, returning: bool) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error('navigate_to_pose is not available.')
            return False
        if not pose.header.frame_id:
            pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, returning=returning: self._goal_response(fut, returning))
        return True

    def _goal_response(self, future, returning: bool) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.last_result = 'Nav2 rejected the goal.'
            self.get_logger().error(self.last_result)
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, returning=returning: self._goal_done(fut, returning))

    def _goal_done(self, future, returning: bool) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: BLE001
            self.last_result = str(exc)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state = 'DONE' if not returning else 'DONE'
            self.recommended = (
                'Return-to-base complete.' if returning
                else 'Validation waypoint reached. Update the hazard map.'
            )
        elif self.state != 'HOLD':
            self.last_result = f'Nav2 finished with status {status}.'
        self.goal_handle = None

    def _publish_status(self) -> None:
        self.state_pub.publish(String(data=self.state))
        self.action_pub.publish(String(data=self.recommended))
        payload = {
            'state': self.state,
            'recommended': self.recommended,
            'hazard': self.hazard_label,
            'robot': self.robot_name,
            'has_target': self.target is not None,
            'target': None,
            'last_result': self.last_result,
        }
        if self.target is not None:
            payload['target'] = {
                'x': round(self.target.pose.position.x, 2),
                'y': round(self.target.pose.position.y, 2),
                'z': round(self.target.pose.position.z, 2),
            }
        self.json_pub.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
