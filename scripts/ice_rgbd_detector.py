#!/usr/bin/env python3
"""RGB-D perception node for alpine cable ice detection

Fuses RGB color segmentation (pale cyan/ice-blue material spectrum) with
3D depth point cloud geometry. Validates the physical "surface going up"
radial bulge where ice sleeve radius (0.22m - 0.45m) exceeds the bare
cable radius (0.17m).

Publishes:
  /fde/detection         (geometry_msgs/PoseStamped)
  /fde/detection_status  (std_msgs/String: none | candidate | confirmed)
  /fde/markers           (visualization_msgs/MarkerArray)
  /fde/debug_image       (sensor_msgs/Image)
  /mission/target        (geometry_msgs/PoseStamped)
  /mission/hazard_label  (std_msgs/String)

Also exports detections to ~/.ros/ice_detections.json for scoring
with scripts/ice_evaluator.py.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from ice_cable_geometry import (  # noqa: E402
    CABLE_RADIUS,
    min_cable_clearance,
)


def _quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix."""
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


class IceRgbdDetector(Node):
    """Fuses RGB color segmentation with 3D point cloud surface elevation."""

    def __init__(self) -> None:
        super().__init__('ice_rgbd_detector')

        # Parameters
        self.declare_parameter('parrot_namespace', 'parrot1')
        self.declare_parameter('husky_namespace', 'husky1')
        self.declare_parameter('camera_image_topic', '/parrot1/camera/image')
        self.declare_parameter('camera_points_topic', '/parrot1/camera/depth/points')
        self.declare_parameter('odom_topic', '/parrot1/odometry')
        self.declare_parameter('detection_topic', '/fde/detection')
        self.declare_parameter('status_topic', '/fde/detection_status')
        self.declare_parameter('markers_topic', '/fde/markers')
        self.declare_parameter('debug_image_topic', '/fde/debug_image')
        self.declare_parameter('target_frame', 'husky1_map')
        self.declare_parameter('fallback_frame', 'parrot1_odom')
        self.declare_parameter('output_json_path', str(Path.home() / '.ros' / 'ice_detections.json'))
        self.declare_parameter('parrot_spawn_x', -8.0)
        self.declare_parameter('parrot_spawn_y', 0.0)
        self.declare_parameter('parrot_spawn_z', 4.0)

        # Tuning thresholds
        self.declare_parameter('min_ice_area_px', 35)
        self.declare_parameter('bare_cable_radius_m', CABLE_RADIUS)  # 0.17 m
        self.declare_parameter('min_bulge_radius_m', 0.20)          # >0.20 m verifies surface swelling
        self.declare_parameter('max_cable_proximity_m', 3.0)        # must be near cableway
        self.declare_parameter('min_cluster_points', 12)
        self.declare_parameter('camera_hfov_rad', 2.0944)           # 120 deg
        self.declare_parameter('camera_pitch_rad', 0.785398)        # 45 deg down
        self.declare_parameter('confidence_confirmed', 0.95)
        self.declare_parameter('confidence_candidate', 0.50)

        # HSV thresholds for pale-blue ice material
        # In SDF: ambient=(0.52, 0.70, 0.80), diffuse=(0.76, 0.90, 0.97)
        self.declare_parameter('hsv_h_min', 80)
        self.declare_parameter('hsv_h_max', 125)
        self.declare_parameter('hsv_s_min', 28)
        self.declare_parameter('hsv_s_max', 240)
        self.declare_parameter('hsv_v_min', 90)
        self.declare_parameter('hsv_v_max', 255)

        self._bridge = CvBridge()
        self._latest_points_msg: Optional[PointCloud2] = None
        self._latest_odom: Optional[Odometry] = None
        self._confirmed_detections: List[Dict[str, Any]] = []
        self._last_json_save = 0.0

        # Subscriptions
        img_topic = str(self.get_parameter('camera_image_topic').value)
        pts_topic = str(self.get_parameter('camera_points_topic').value)
        odom_top = str(self.get_parameter('odom_topic').value)

        self.create_subscription(Image, img_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, pts_topic, self._on_points, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_top, self._on_odom, qos_profile_sensor_data)

        # Publishers
        det_topic = str(self.get_parameter('detection_topic').value)
        stat_topic = str(self.get_parameter('status_topic').value)
        mrk_topic = str(self.get_parameter('markers_topic').value)
        dbg_topic = str(self.get_parameter('debug_image_topic').value)

        self._det_pub = self.create_publisher(PoseStamped, det_topic, 10)
        self._stat_pub = self.create_publisher(String, stat_topic, 10)
        self._mrk_pub = self.create_publisher(MarkerArray, mrk_topic, 10)
        self._dbg_pub = self.create_publisher(Image, dbg_topic, 10)

        # Compatibility with mission_gate / UI
        self._mission_target_pub = self.create_publisher(PoseStamped, '/mission/target', 10)
        self._mission_label_pub = self.create_publisher(String, '/mission/hazard_label', 10)

        self.get_logger().info(
            f'Ice RGB-D Perception initialized. Listening to RGB [{img_topic}] + PointCloud [{pts_topic}]'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_points(self, msg: PointCloud2) -> None:
        self._latest_points_msg = msg

    def _on_image(self, msg: Image) -> None:
        """Process incoming RGB image and fuse with 3D point cloud."""
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            return

        h, w = cv_img.shape[:2]
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # Step 1: Color segmentation for ice pale-cyan
        h_min = int(self.get_parameter('hsv_h_min').value)
        h_max = int(self.get_parameter('hsv_h_max').value)
        s_min = int(self.get_parameter('hsv_s_min').value)
        s_max = int(self.get_parameter('hsv_s_max').value)
        v_min = int(self.get_parameter('hsv_v_min').value)
        v_max = int(self.get_parameter('hsv_v_max').value)

        lower_ice = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper_ice = np.array([h_max, s_max, v_max], dtype=np.uint8)

        raw_mask = cv2.inRange(hsv, lower_ice, upper_ice)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        ice_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, kernel)
        ice_mask = cv2.morphologyEx(ice_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(ice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(self.get_parameter('min_ice_area_px').value)

        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area >= min_area:
                bx, by, bw, bh = cv2.boundingRect(c)
                mc = cv2.moments(c)
                if mc['m00'] > 0:
                    cx = int(mc['m10'] / mc['m00'])
                    cy = int(mc['m01'] / mc['m00'])
                else:
                    cx, cy = bx + bw // 2, by + bh // 2
                candidates.append({
                    'contour': c,
                    'bbox': (bx, by, bw, bh),
                    'centroid_px': (cx, cy),
                    'area_px': area,
                })

        # Step 2: Fuse with 3D point cloud
        detections = []
        status_str = 'none'

        if candidates:
            status_str = 'candidate'
            pts_msg = self._latest_points_msg

            for cand in candidates:
                verified, det_info = self._verify_candidate_geometry(cand, pts_msg, w, h)
                if verified:
                    status_str = 'confirmed'
                    cand['confirmed'] = True
                    cand['info'] = det_info
                    detections.append(det_info)
                else:
                    cand['confirmed'] = False

        # Step 3: Publish results
        self._publish_status(status_str)

        if detections:
            # Publish highest confidence detection
            best_det = max(detections, key=lambda d: d.get('confidence', 0.0))
            self._publish_detection(best_det)
            self._publish_markers(detections)
            self._record_detections(detections)
        elif status_str == 'candidate':
            # Publish candidate indication
            cand = candidates[0]
            cx, cy = cand['centroid_px']
            self.get_logger().info(f'Ice candidate detected at pixel ({cx}, {cy}), awaiting 3D geometric confirmation...')

        # Step 4: Generate debug visualization image
        self._publish_debug_image(cv_img, candidates, status_str)

    def _verify_candidate_geometry(
        self,
        candidate: Dict[str, Any],
        pts_msg: Optional[PointCloud2],
        img_w: int,
        img_h: int,
    ) -> Tuple[bool, Dict[str, Any]]:
        """Verify the 3D surface elevation and radial swelling of the candidate."""
        if pts_msg is None:
            return False, {}

        bx, by, bw, bh = candidate['bbox']
        hfov = float(self.get_parameter('camera_hfov_rad').value)
        fx = (img_w / 2.0) / math.tan(hfov / 2.0)
        fy = fx
        cx_img = img_w / 2.0
        cy_img = img_h / 2.0

        # Extract 3D points inside the candidate bounding box
        matched_3d: List[Tuple[float, float, float]] = []

        try:
            # Read 3D points
            for pt in pc2.read_points(pts_msg, field_names=('x', 'y', 'z'), skip_nans=True):
                px, py, pz = float(pt[0]), float(pt[1]), float(pt[2])
                if pz <= 0.2 or pz > 35.0:
                    continue
                # Camera projection to image coordinates
                u = cx_img + fx * (px / pz)
                v = cy_img + fy * (py / pz)
                if bx <= u <= (bx + bw) and by <= v <= (by + bh):
                    matched_3d.append((px, py, pz))
                    if len(matched_3d) > 200:
                        break
        except Exception as exc:
            self.get_logger().debug(f'PointCloud read error: {exc}')
            return False, {}

        min_pts = int(self.get_parameter('min_cluster_points').value)
        if len(matched_3d) < min_pts:
            return False, {}

        # 3D cluster centroid in camera coordinates
        arr_pts = np.array(matched_3d)
        mean_cam = np.mean(arr_pts, axis=0)

        # Transform 3D point to world/odom frame
        world_pt = self._transform_cam_to_world(mean_cam)
        if world_pt is None:
            return False, {}

        wx, wy, wz = world_pt

        # Cable geometry clearance & span verification
        dist_cable, span_id, cable_id = min_cable_clearance(wx, wy, wz)
        max_cable_prox = float(self.get_parameter('max_cable_proximity_m').value)

        if dist_cable > max_cable_prox:
            # Not close to cable corridor
            return False, {}

        # Transform sample 3D points to world coordinates
        world_pts = [self._transform_cam_to_world(np.array(pt)) for pt in matched_3d[:40]]
        valid_world_pts = [pt for pt in world_pts if pt is not None]
        if not valid_world_pts:
            return False, {}

        # Evaluate the "surface going up" radial swelling from cable centerline
        # Bare cable radius is 0.17m; ice blocks swell to 0.22m (small) up to 0.45m (large).
        bare_r = float(self.get_parameter('bare_cable_radius_m').value)
        min_bulge_r = float(self.get_parameter('min_bulge_radius_m').value)

        # Distance of surface points from cable centerline gives physical surface radius
        radial_dists = [min_cable_clearance(pt[0], pt[1], pt[2])[0] for pt in valid_world_pts]
        est_surface_radius = float(np.percentile(radial_dists, 75))

        # Check if surface bulges above bare cable baseline
        surface_going_up = est_surface_radius >= min_bulge_r

        if not surface_going_up:
            return False, {}

        conf = float(self.get_parameter('confidence_confirmed').value)
        return True, {
            'span_id': span_id,
            'cable_id': cable_id,
            'position': [float(wx), float(wy), float(wz)],
            'estimated_radius': float(max(est_surface_radius, min_bulge_r)),
            'protrusion_m': float(max(0.0, est_surface_radius - bare_r)),
            'dist_to_cable': float(dist_cable),
            'point_count': len(matched_3d),
            'confidence': conf,
            'status': 'confirmed',
        }

    def _transform_cam_to_world(self, cam_pt: np.ndarray) -> Optional[Tuple[float, float, float]]:
        """Transform point from camera coordinate frame to world/odom frame."""
        if self._latest_odom is None:
            # Fallback estimation using known drone inspection altitude
            return None

        p = self._latest_odom.pose.pose.position
        q = self._latest_odom.pose.pose.orientation
        R_drone = _quat_to_rot_matrix(q.x, q.y, q.z, q.w)

        # Camera mounting on Parrot: offset [0.25, 0, 0.25], pitched down by 45 deg
        pitch = float(self.get_parameter('camera_pitch_rad').value)
        R_pitch = np.array([
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ], dtype=np.float64)

        cam_offset = np.array([0.25, 0.0, 0.25], dtype=np.float64)

        # Optical frame to camera link: x_cam=Z, y_cam=-X, z_cam=-Y or standard optical
        # In ROS standard optical frame: Z forward, X right, Y down
        # In camera link: X forward, Y left, Z up
        p_cam_link = np.array([cam_pt[2], -cam_pt[0], -cam_pt[1]], dtype=np.float64)

        # Rotate by camera pitch joint
        p_body = cam_offset + R_pitch.dot(p_cam_link)

        # Rotate by drone attitude and translate (in parrot1_odom)
        p_world = np.array([p.x, p.y, p.z], dtype=np.float64) + R_drone.dot(p_body)
        
        # parrot1_odom is already absolute world coordinates, no offset needed!

        return float(p_world[0]), float(p_world[1]), float(p_world[2])

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._stat_pub.publish(msg)

    def _publish_detection(self, det: Dict[str, Any]) -> None:
        pos = det['position']
        frame = str(self.get_parameter('target_frame').value)
        if self._latest_odom is not None and not self._latest_odom.header.frame_id:
            frame = str(self.get_parameter('fallback_frame').value)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.position.z = pos[2]
        ps.pose.orientation.w = 1.0

        self._det_pub.publish(ps)

        # Downstream mission handoff publishers
        self._mission_target_pub.publish(ps)
        label_msg = String()
        label_msg.data = f"ice_sleeve_{det.get('span_id', 'span_01')}"
        self._mission_label_pub.publish(label_msg)

    def _publish_markers(self, detections: List[Dict[str, Any]]) -> None:
        arr = MarkerArray()
        frame = str(self.get_parameter('target_frame').value)

        for i, det in enumerate(detections):
            pos = det['position']
            rad = det.get('estimated_radius', 0.32)

            # Marker 1: Ice cylinder / sphere
            m_geo = Marker()
            m_geo.header.stamp = self.get_clock().now().to_msg()
            m_geo.header.frame_id = frame
            m_geo.ns = 'ice_detection_geometry'
            m_geo.id = i * 2
            m_geo.type = Marker.CYLINDER
            m_geo.action = Marker.ADD
            m_geo.pose.position.x = pos[0]
            m_geo.pose.position.y = pos[1]
            m_geo.pose.position.z = pos[2]
            m_geo.pose.orientation.w = 1.0
            m_geo.scale.x = rad * 2.0
            m_geo.scale.y = rad * 2.0
            m_geo.scale.z = 1.0
            # Cyan icy appearance
            m_geo.color.r = 0.52
            m_geo.color.g = 0.85
            m_geo.color.b = 0.98
            m_geo.color.a = 0.85
            arr.markers.append(m_geo)

            # Marker 2: Text annotation
            m_txt = Marker()
            m_txt.header.stamp = self.get_clock().now().to_msg()
            m_txt.header.frame_id = frame
            m_txt.ns = 'ice_detection_labels'
            m_txt.id = i * 2 + 1
            m_txt.type = Marker.TEXT_VIEW_FACING
            m_txt.action = Marker.ADD
            m_txt.pose.position.x = pos[0]
            m_txt.pose.position.y = pos[1]
            m_txt.pose.position.z = pos[2] + 0.8
            m_txt.scale.z = 0.4
            m_txt.color.r = 1.0
            m_txt.color.g = 1.0
            m_txt.color.b = 1.0
            m_txt.color.a = 1.0
            m_txt.text = f"CONFIRMED ICE ({det['span_id']}) r={rad:.2f}m"
            arr.markers.append(m_txt)

        self._mrk_pub.publish(arr)

    def _record_detections(self, detections: List[Dict[str, Any]]) -> None:
        now = time.monotonic()
        if now - self._last_json_save < 1.0:
            return
        self._last_json_save = now

        out_path = Path(str(self.get_parameter('output_json_path').value)).expanduser()
        out_path.parent.mkdir(parents=True, exist_ok=True)

        payload = {'detections': detections}
        try:
            out_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        except OSError as exc:
            self.get_logger().debug(f'Failed writing detections JSON: {exc}')

    def _publish_debug_image(
        self,
        bgr: np.ndarray,
        candidates: List[Dict[str, Any]],
        status: str,
    ) -> None:
        annotated = bgr.copy()

        # Banner at top
        banner_color = (0, 0, 200) if status == 'none' else ((0, 180, 255) if status == 'candidate' else (0, 220, 0))
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 36), (20, 20, 20), -1)
        cv2.putText(
            annotated,
            f"PERCEPTION: {status.upper()} | CANDIDATES: {len(candidates)}",
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            banner_color,
            2,
        )

        for cand in candidates:
            bx, by, bw, bh = cand['bbox']
            is_conf = cand.get('confirmed', False)
            box_col = (0, 255, 0) if is_conf else (0, 165, 255)

            cv2.drawContours(annotated, [cand['contour']], -1, box_col, 2)
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), box_col, 2)

            cx, cy = cand['centroid_px']
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

            if is_conf:
                info = cand.get('info', {})
                lbl = f"CONFIRMED ({info.get('span_id', '')}) r={info.get('estimated_radius', 0):.2f}m"
                cv2.putText(annotated, lbl, (bx, max(15, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                cv2.putText(annotated, "CANDIDATE (COLOR OK)", (bx, max(15, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)

        try:
            msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self._dbg_pub.publish(msg)
        except Exception:
            pass


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = IceRgbdDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
