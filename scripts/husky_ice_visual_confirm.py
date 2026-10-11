#!/usr/bin/env python3
"""Husky independent RGB image-based confirmation for Mission 3.

Request topic  /husky1/ice_confirm/request (std_msgs/String JSON):
    {"request_id":"hazard_1", "frame_id":"husky1_odom", "position":[x,y,z]}
Result topic   /husky1/ice_confirm/result  (std_msgs/String JSON):
    {"request_id":...,"status":"CONFIRMED|NOT_CONFIRMED|INCONCLUSIVE", ...}

Camera is the ONLY source of positive evidence. TF + UAV position select an
image region of interest; neither may directly produce CONFIRMED. Each
inspection requires five fresh frames and four pixel-positive frames by default.
No ground truth file is read. Debug images and JSONL results are saved under
~/.ros for the final demonstration and evaluation.
"""

from __future__ import annotations

import json
import math
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))
from husky_ice_vision_core import (  # noqa: E402
    VisionConfig, analyse_rgb_frame, multiframe_verdict, odometry_has_3d,
    project_point, project_point_with_pose, quaternion_to_rpy,
)


class HuskyVisualConfirm(Node):
    def __init__(self) -> None:
        super().__init__('husky_ice_visual_confirm')
        self.declare_parameter('image_topic', '/husky1/camera/image')
        self.declare_parameter('camera_info_topic', '/husky1/camera/camera_info')
        self.declare_parameter('request_topic', '/husky1/ice_confirm/request')
        self.declare_parameter('result_topic', '/husky1/ice_confirm/result')
        self.declare_parameter('debug_topic', '/husky1/ice_confirm/debug_image')
        self.declare_parameter('optical_frame', 'husky1_camera_rgb_optical_frame')
        self.declare_parameter('frames_required', 5)
        self.declare_parameter('positives_required', 4)
        self.declare_parameter('timeout_seconds', 8.0)
        self.declare_parameter('roi_half_width_px', 100)
        self.declare_parameter('roi_half_height_px', 75)
        self.declare_parameter('min_ice_area_px', 25.0)
        self.declare_parameter('output_dir', str(Path.home() / '.ros' / 'husky_evidence'))
        # Slope compensation: TF is planar (EKF two_d_mode), so on the ramp the
        # camera is drawn too low and the ROI lands above the ice. These use the
        # Husky's real height/roll/pitch from 3D odometry; x, y, yaw stay from TF.
        self.declare_parameter('use_pose_correction', True)
        self.declare_parameter('pose_odom_topic', '/husky1/odometry')
        self.declare_parameter('odom_frame', 'husky1_odom')
        self.declare_parameter('base_frame', 'husky1_base_link')
        self.declare_parameter('pose_max_age_s', 2.0)

        self._bridge = CvBridge()
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._camera_info: Optional[CameraInfo] = None
        self._active: Optional[Dict[str, Any]] = None
        self._started = 0.0
        self._positive = 0
        self._valid_frames = 0
        self._last_frame = None
        self._request_clock_ns = None
        self._last_debug = None
        self._last_reason = 'no_camera_image'
        # Latest Husky height/tilt (from 3D odometry) and how the ROI was placed.
        self._pose_z: Optional[float] = None
        self._pose_roll = 0.0
        self._pose_pitch = 0.0
        self._pose_rx_mono = 0.0
        self._pose_mode = 'flat_tf'
        self._pose_note = 'not_requested'
        self._cfg = VisionConfig(
            half_width_px=int(self.get_parameter('roi_half_width_px').value),
            half_height_px=int(self.get_parameter('roi_half_height_px').value),
            min_area_px=float(self.get_parameter('min_ice_area_px').value),
        )
        self._output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        self._result_pub = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), 10)
        self._debug_pub = self.create_publisher(
            Image, str(self.get_parameter('debug_topic').value), 10)
        self.create_subscription(String, str(self.get_parameter('request_topic').value), self._on_request, 10)
        self.create_subscription(CameraInfo, str(self.get_parameter('camera_info_topic').value),
                                 self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter('image_topic').value),
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self.get_parameter('pose_odom_topic').value),
                                 self._on_pose_odom, qos_profile_sensor_data)
        self.create_timer(0.2, self._check_timeout)
        self.get_logger().info(
            'Husky RGB visual confirmation ready (no ground-truth file access; '
            'slope compensation %s from %s).' % (
                'ON' if bool(self.get_parameter('use_pose_correction').value) else 'OFF',
                str(self.get_parameter('pose_odom_topic').value)))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_pose_odom(self, msg: Odometry) -> None:
        """Remember the Husky's real height, roll and pitch (3D odometry)."""
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            roll, pitch, _yaw = quaternion_to_rpy((o.x, o.y, o.z, o.w))
        except ValueError:
            return
        self._pose_z = float(p.z)
        self._pose_roll = roll
        self._pose_pitch = pitch
        self._pose_rx_mono = time.monotonic()

    def _project_target(self, request: Dict[str, Any], transform, intrinsics):
        """Pixel of the requested target, slope-compensated when possible.

        Falls back to the flat TF projection (old behaviour) and records why, so
        a missing or planar odometry reading is visible instead of silently wrong.
        """
        t, q = transform.transform.translation, transform.transform.rotation
        flat = project_point(
            tuple(request['position']), (t.x, t.y, t.z),
            (q.x, q.y, q.z, q.w), intrinsics)

        def fallback(note: str):
            self._pose_mode, self._pose_note = 'flat_tf', note
            return flat

        if not bool(self.get_parameter('use_pose_correction').value):
            return fallback('correction_disabled')
        odom_frame = str(self.get_parameter('odom_frame').value)
        if request['frame_id'] != odom_frame:
            return fallback('request_frame_%s_not_%s' % (request['frame_id'], odom_frame))
        if self._pose_z is None:
            return fallback('no_pose_odometry')
        age = time.monotonic() - self._pose_rx_mono
        if age > float(self.get_parameter('pose_max_age_s').value):
            return fallback('pose_odometry_stale_%.1fs' % age)
        if not odometry_has_3d(self._pose_z, self._pose_roll, self._pose_pitch):
            return fallback('odometry_is_planar_set_dimensions_3')
        base_frame = str(self.get_parameter('base_frame').value)
        optical = str(self.get_parameter('optical_frame').value)
        try:
            base_in_odom = self._tf.lookup_transform(odom_frame, base_frame, rclpy.time.Time())
            optical_in_base = self._tf.lookup_transform(base_frame, optical, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn('Pose-correction TF unavailable: %s' % exc,
                                   throttle_duration_sec=5.0)
            return fallback('tf_unavailable')
        bt, bq = base_in_odom.transform.translation, base_in_odom.transform.rotation
        ot, oq = optical_in_base.transform.translation, optical_in_base.transform.rotation
        corrected = project_point_with_pose(
            tuple(request['position']), (bt.x, bt.y, bt.z), (bq.x, bq.y, bq.z, bq.w),
            self._pose_z, self._pose_roll, self._pose_pitch,
            (ot.x, ot.y, ot.z), (oq.x, oq.y, oq.z, oq.w), intrinsics)
        self._pose_mode = 'corrected'
        self._pose_note = 'h=%.2fm pitch=%.1fdeg roll=%.1fdeg' % (
            self._pose_z, math.degrees(self._pose_pitch), math.degrees(self._pose_roll))
        return corrected

    def _on_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
            request_id = str(req['request_id'])
            frame_id = str(req['frame_id'])
            position = [float(v) for v in req['position']]
            if not request_id or not frame_id or len(position) != 3:
                raise ValueError('Missing ID/frame/XYZ')
            if not all(math.isfinite(v) for v in position):
                raise ValueError('Nonfinite coordinates')
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warn('Invalid Husky inspection request: %s' % exc)
            return
        if self._active is not None:
            self._finish('INCONCLUSIVE', 'superseded_by_next_request')
        self._active = {'request_id': request_id, 'frame_id': frame_id, 'position': position}
        self._started = time.monotonic()
        self._positive = 0
        self._valid_frames = 0
        self._last_frame = None
        self._request_clock_ns = self.get_clock().now().nanoseconds
        self._last_debug = None
        self._last_reason = 'waiting_for_camera'
        self.get_logger().info('Inspection %s: look toward %s in %s' %
                               (request_id, position, frame_id))

    def _on_image(self, msg: Image) -> None:
        if self._active is None:
            # Standby feed so rqt_image_view always has an active live stream
            try:
                raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.putText(raw, 'Husky VISUAL: [STANDBY - Waiting for confirm request]',
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
                out = self._bridge.cv2_to_imgmsg(raw, encoding='bgr8')
                out.header = msg.header
                self._debug_pub.publish(out)
            except Exception:
                pass
            return

        frame_id = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if frame_id != (0, 0) and frame_id == self._last_frame:
            return
        self._last_frame = frame_id

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._last_reason = 'image_conversion_failed'
            self.get_logger().warn('Husky image conversion failed: %s' % exc)
            return

        if self._camera_info is None:
            self._last_reason = 'waiting_for_camera_info'
            debug = image.copy()
            cv2.putText(debug, 'Husky VISUAL: Waiting for camera_info',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 165, 255), 1)
            out = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            out.header = msg.header
            self._debug_pub.publish(out)
            return

        info = self._camera_info
        fx, fy, cx, cy = float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
        if fx <= 0 or fy <= 0:
            self._last_reason = 'invalid_camera_calibration'
            return

        request = self._active
        try:
            transform = self._tf.lookup_transform(
                str(self.get_parameter('optical_frame').value), request['frame_id'],
                rclpy.time.Time())
        except TransformException as exc:
            self._last_reason = 'tf_unavailable'
            self.get_logger().warn('Husky optical TF unavailable: %s' % exc,
                                   throttle_duration_sec=5.0)
            debug = image.copy()
            cv2.putText(debug, 'Husky VISUAL: Waiting for TF...',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 165, 255), 1)
            out = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            out.header = msg.header
            self._debug_pub.publish(out)
            return

        centre = self._project_target(request, transform, (fx, fy, cx, cy))

        if centre is None:
            self._last_reason = 'target_behind_camera'
            debug = image.copy()
            cv2.putText(debug, 'Husky VISUAL %s: Target behind camera' % request['request_id'],
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 1)
            out = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            out.header = msg.header
            self._debug_pub.publish(out)
            return

        try:
            result = analyse_rgb_frame(image, centre, self._cfg)
        except Exception as exc:
            self._last_reason = 'image_processing_failed'
            self.get_logger().warn('Husky image processing failed: %s' % exc)
            return

        self._last_reason = str(result['reason'])

        if not result['valid']:
            debug = image.copy()
            overlay = 'Husky VISUAL %s: %s (proj: %.0f, %.0f)' % (
                request['request_id'], result['reason'], centre[0], centre[1])
            cv2.putText(debug, overlay, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (0, 0, 255), 1)
            out = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            out.header = msg.header
            self._debug_pub.publish(out)
            return

        self._valid_frames += 1
        self._positive += int(result['positive'])
        debug = image.copy()
        x0, y0, x1, y1 = result['roi']
        cv2.rectangle(debug, (x0, y0), (x1, y1), (0, 220, 255), 2)
        cv2.drawMarker(debug, (int(centre[0]), int(centre[1])), (0, 200, 255),
                       cv2.MARKER_CROSS, 14, 2)
        candidate = result.get('candidate')
        if candidate:
            x, y, w, h = candidate['bbox']
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
        overlay = 'Husky VISUAL %s %d/%d positive (%d/%d frames)' % (
            request['request_id'], self._positive, self._valid_frames,
            self._valid_frames, int(self.get_parameter('frames_required').value))
        cv2.putText(debug, overlay, (8, 23), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (0, 255, 0) if candidate else (0, 170, 255), 1)
        cv2.putText(debug, 'pose: %s %s' % (self._pose_mode, self._pose_note),
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    (255, 255, 0) if self._pose_mode == 'corrected' else (0, 140, 255), 1)
        self._last_debug = debug
        out = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        out.header = msg.header
        self._debug_pub.publish(out)
        decision = multiframe_verdict(
            self._positive, self._valid_frames,
            int(self.get_parameter('frames_required').value),
            int(self.get_parameter('positives_required').value))
        if decision is not None:
            self._finish(decision, 'rgb_pixels_in_target_roi')

    def _check_timeout(self) -> None:
        if self._active is None:
            return
        if time.monotonic() - self._started >= float(self.get_parameter('timeout_seconds').value):
            self._finish('INCONCLUSIVE', self._last_reason)

    def _finish(self, status: str, reason: str) -> None:
        if self._active is None:
            return
        req = self._active
        self._active = None
        result = {
            'request_id': req['request_id'],
            'position': req['position'],
            'frame_id': req['frame_id'],
            'status': status,
            'positive_frames': self._positive,
            'valid_frames': self._valid_frames,
            'frames_required': int(self.get_parameter('frames_required').value),
            'reason': reason,
            'method': 'husky_independent_rgb_roi_multiframe',
            'pose_mode': self._pose_mode,
            'pose_note': self._pose_note,
            'image_path': None,
        }
        try:
            self._output_dir.mkdir(parents=True, exist_ok=True)
            safe_id = re.sub(r'[^a-zA-Z0-9_-]', '_', str(req['request_id']))[:70]
            if self._last_debug is not None:
                name = '%s_%s.png' % (safe_id, status.lower())
                location = self._output_dir / name
                if cv2.imwrite(str(location), self._last_debug):
                    result['image_path'] = str(location)
            with (self._output_dir / 'results.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps(result) + '\n')
        except OSError as exc:
            self.get_logger().error('Failed to save Husky inspection evidence: %s' % exc)
        self._result_pub.publish(String(data=json.dumps(result)))
        self.get_logger().info('%s: %s (%d/%d pixel-positive images)' % (
            req['request_id'], status, self._positive, self._valid_frames))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HuskyVisualConfirm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

