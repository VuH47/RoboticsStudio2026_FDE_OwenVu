#!/usr/bin/env python3
import sys
import json
import os
import uuid


from pathlib import Path
CURRENT_DIR = Path(__file__).resolve().parent
# Running from source:
# fde_gui/scripts/operator_gui.py
source_package_root = CURRENT_DIR.parent
# Running through ros2 run:
# install/fde_gui/lib/fde_gui/operator_gui.py
installed_package_root = CURRENT_DIR
if (CURRENT_DIR / "operator_interface").exists():
    PACKAGE_ROOT = installed_package_root
else:
    PACKAGE_ROOT = source_package_root
sys.path.insert(0, str(PACKAGE_ROOT))
from operator_interface.data.ros_target_sync import RosTargetSync
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from PySide6.QtGui import QImage
from rclpy.node import Node
from PySide6.QtCore import Qt, Signal, QPointF
from std_msgs.msg import String
from operator_interface.widgets.missions.mission_summary import MissionSummary
from operator_interface.widgets.map.map_overlay_panel import MapOverlayPanel
from operator_interface.widgets.operations.live_operations import LiveOperations
from operator_interface.widgets.hazards.documented_hazards import (
    DocumentedHazards,
)
from operator_interface.widgets.map.map_tool_panel import MapToolPanel
from operator_interface.widgets.map.selected_hazard_panel import SelectedHazardPanel
from operator_interface.widgets.missions.mission_progress import MissionProgress
from operator_interface.database.mission_database import MissionDatabase
from operator_interface.data.hazard_store import HazardStore
from PySide6.QtCore import QPointF, Qt, QTimer, Signal, QEvent
from PySide6.QtGui import QBrush, QColor, QFont, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGraphicsEllipseItem,
    QGraphicsPathItem,
    QGraphicsPolygonItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from nav_msgs.msg import Odometry
from operator_interface.widgets.waypoint_store import WaypointStore
from operator_interface.widgets.map.renderer.map_3d_widget import Map3DWidget
from PySide6.QtWidgets import QDialog
from operator_interface.widgets.map.manual_waypoint_dialog import (
    ManualWaypointDialog,
)
import math
from vtkmodules.vtkRenderingCore import vtkCellPicker


# =========================================================
# ROS NODE
# =========================================================
class OperatorRosNode(Node):
    def __init__(self):
        super().__init__("fde_operator_gui")
        self.mission_state = "IDLE"
        self.latest_detection = None

        self.targets_callback_gui = None

        self.targets_sub = self.create_subscription(
            String,
            "/fde/ice_targets",
            self.targets_callback,
            10,
        )

        self.mission_progress_callback_gui = None
        self.command_pub = self.create_publisher(
            String,
            "/fde/mission_command",
            10,
        )
        self.create_subscription(
            String,
            "/fde/mission_state",
            self.mission_state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/fde/detection",
            self.detection_callback,
            10,
        )
        self.mission_progress_sub = self.create_subscription(
            String,
            "/fde/mission_progress",
            self.mission_progress_callback,
            10,
)
        self.get_logger().info("PySide6 operator GUI started")
    def mission_state_callback(self, msg):
        self.mission_state = msg.data
    def detection_callback(self, msg):
        self.latest_detection = msg

    def targets_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(
                "Invalid JSON received on /fde/ice_targets"
            )
            return

        if self.targets_callback_gui is not None:
            self.targets_callback_gui(data)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"Mission command: {command}")
    def mission_progress_callback(self, msg):
        if self.mission_progress_callback_gui is not None:
            self.mission_progress_callback_gui(msg.data)
# =========================================================
# CLICKABLE MAP ITEM
# =========================================================
class HazardMarker(QGraphicsEllipseItem):
    def __init__(
        self,
        hazard_id,
        x,
        y,
        colour,
        callback,
        diameter=28,
    ):
        super().__init__(
            x - diameter / 2,
            y - diameter / 2,
            diameter,
            diameter,
        )
        self.hazard_id = hazard_id
        self.callback = callback
        self.setBrush(QBrush(QColor(colour)))
        self.setPen(QPen(QColor("#f4f4f4"), 3))
        self.setZValue(20)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip(f"Open {hazard_id}")
    def mousePressEvent(self, event):
        self.callback(self.hazard_id)
        super().mousePressEvent(event)

class MissionMapView(QGraphicsView):

    map_clicked = Signal(float, float)

    def mousePressEvent(self, event):

        if event.button() == Qt.LeftButton:

            scene_pos = self.mapToScene(
                event.position().toPoint()
            )

            self.map_clicked.emit(
                scene_pos.x(),
                scene_pos.y(),
            )

        super().mousePressEvent(event)
# =========================================================
# MAIN WINDOW
# =========================================================
class OperatorGUI(QMainWindow):
    STATUS_COLOURS = {
        "Detected": "#ffc107",
        "Husky Required": "#ffa16b",
        "In Progress": "#2f80c9",
        "Resolved / Safe": "#49ad2f",
        "Confirmed Hazard": "#ff1616",
    }
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("FDE Alpine Hazard Inspection")
        self.resize(1500, 850)
        self.setMinimumSize(1200, 700)
        self.selected_hazard_id = None
        self.map_items = {
            "drone": [],
            "husky": [],
            "waypoints": [],
            "hazards": [],
        }
        self.hazard_markers = {}
        self.database = MissionDatabase()

        self.hazard_store = HazardStore(self.database)

        self.mission_run_id = os.environ.get(
            "FDE_MISSION_RUN_ID"
        ) or uuid.uuid4().hex[:8].upper()

        self.ros_target_sync = RosTargetSync(
            self.hazard_store,
            self.mission_run_id,
        )

        self.ros_node.targets_callback_gui = (
            self.ros_target_sync.process
        )

        self.hazards = self.hazard_store.get_all()

        # Shared Drone/Husky waypoint store
        self.waypoint_store = WaypointStore(self)


        self.build_ui()
        self.sync_hazard_markers()
        self.hazard_store.hazards_changed.connect(
            self.on_hazards_changed
        )
        self.apply_style()
        self.update_outcome_counts()
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.process_ros)
        self.ros_timer.start(20)
        QTimer.singleShot(0, self.position_map_overlays)
    # =====================================================
    # UI
    # =====================================================
    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)
        main_layout.addWidget(self.create_left_panel(), 6)
        main_layout.addWidget(self.create_right_panel(), 5)
    # =====================================================
    # LEFT PANEL
    # =====================================================
    def create_left_panel(self):
        panel = QFrame()
        panel.setObjectName("mainPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 6, 8, 8)
        layout.setSpacing(5)
        header = QHBoxLayout()
        header.setContentsMargins(
            0,
            0,
            0,
            0,
        )
        header.setSpacing(15)
        title = QLabel(
            "Mission Interface\nMap"
        )
        title.setObjectName("missionInterfaceTitle")
        title.setFont(
            QFont("Arial", 19)
        )
        header.addWidget(
            title
        )
        self.mission_progress = MissionProgress()
        header.addWidget(
            self.mission_progress,
            1,
            Qt.AlignVCenter,
        )
        layout.addLayout(
            header
        )
        self.ros_node.mission_progress_callback_gui = (
            self.mission_progress.process_event
        )
        # Live Gazebo world map, sharing the existing WaypointStore.
        self.map_view = Map3DWidget(
            repo_root=PACKAGE_ROOT.parent,
            waypoint_store=self.waypoint_store,
            parent=panel,
        )
        self.map_view.waypoint_markers.on_hazard_waypoint_clicked = (
            self.on_hazard_waypoint_clicked
        )
        self.map_view.setObjectName("mapView")
        self.map_view.setMinimumHeight(430)
        layout.addWidget(self.map_view, 1)
        # Overlays are parented to the actual VTK viewport.
        self.map_view.plotter.interactor.installEventFilter(self)
        # Marker refresh must never cover floating controls.
        self.map_view.marker_overlay.on_refresh = self.refresh_map_overlays
        self.create_map_overlay()
        self.create_map_tool_panel()
        self.create_selected_hazard_overlay()
        # Drone / Husky / Both map selector
        robot_selector = QHBoxLayout()
        self.map_drone_button = QPushButton("Drone")
        self.map_husky_button = QPushButton("Husky")
        self.map_both_button = QPushButton("Both")
        for button in (
            self.map_drone_button,
            self.map_husky_button,
            self.map_both_button,
        ):
            button.setCheckable(True)
            button.setObjectName("selectorButton")
            robot_selector.addWidget(button)
        self.map_drone_button.setChecked(True)
        self.map_drone_button.clicked.connect(lambda: self.set_map_robot_view("Drone"))
        self.map_husky_button.clicked.connect(lambda: self.set_map_robot_view("Husky"))
        self.map_both_button.clicked.connect(lambda: self.set_map_robot_view("Both"))
        layout.addLayout(robot_selector)
        # Mission outcomes / objectives
        self.mission_summary = MissionSummary()
        layout.addWidget(self.mission_summary)
        # Mission buttons
        commands = QHBoxLayout()
        self.start_button = QPushButton("Start Mission")
        self.start_button.setObjectName("startButton")
        self.start_button.clicked.connect(lambda: self.ros_node.send_command("start"))
        self.abort_button = QPushButton("Return Home / Abort")
        self.abort_button.setObjectName("abortButton")
        self.abort_button.clicked.connect(lambda: self.ros_node.send_command("abort"))
        commands.addWidget(self.start_button)
        commands.addWidget(self.abort_button)
        layout.addLayout(commands)
        return panel

    def refresh_map_overlays(self):
        """Keep floating GUI panels above all SVG markers and number labels."""
        self.map_view.waypoint_markers.update_label_positions()
        for name in ("map_overlay", "map_tool_panel", "selected_hazard_panel"):
            panel = getattr(self, name, None)
            if panel is not None:
                panel.raise_()

    def pick_map_position(self, screen_x, screen_y):
        """Convert a viewport click into a Gazebo world position."""
        renderer = self.map_view.plotter.renderer
        viewport = self.map_view.plotter.interactor
        vtk_x = float(screen_x)
        vtk_y = float(viewport.height() - screen_y - 1)

        picker = vtkCellPicker()
        picker.SetTolerance(0.0005)
        if picker.Pick(vtk_x, vtk_y, 0.0, renderer):
            x, y, z = picker.GetPickPosition()
            return float(x), float(y), float(z)

        # Fallback: intersect the camera ray with the z=0 ground plane.
        ray = []
        for depth in (0.0, 1.0):
            renderer.SetDisplayPoint(vtk_x, vtk_y, depth)
            renderer.DisplayToWorld()
            wx, wy, wz, w = renderer.GetWorldPoint()
            if abs(w) < 1e-9:
                return None
            ray.append((wx / w, wy / w, wz / w))
        near, far = ray
        dz = far[2] - near[2]
        if abs(dz) < 1e-9:
            return None
        t = -near[2] / dz
        if not 0.0 <= t <= 1.0:
            return None
        return (near[0] + t * (far[0] - near[0]),
                near[1] + t * (far[1] - near[1]), 0.0)

    def on_map_clicked(self, screen_x, screen_y):
        """Add or remove a waypoint with the currently selected map tool."""
        mode = getattr(self, "current_map_tool_mode", "select")
        if mode == "remove":
            nearest = None
            best_distance = 24.0 ** 2
            for robot, waypoints in (
                ("drone", self.waypoint_store.get_drone_waypoints()),
                ("husky", self.waypoint_store.get_husky_waypoints()),
            ):
                for index, wp in enumerate(waypoints):
                    sx, sy = self.map_view.marker_overlay.world_to_screen(
                        wp["x"], wp["y"], wp.get("z", 0.0) + 1.0
                    )
                    distance = (sx - screen_x) ** 2 + (sy - screen_y) ** 2
                    if distance < best_distance:
                        nearest = (robot, index)
                        best_distance = distance
            if nearest is not None:
                self.waypoint_store.delete_waypoint(*nearest)
                self.map_tool_panel.reset_to_select()
            return

        if mode not in ("add_drone", "add_husky"):
            return
        position = self.pick_map_position(screen_x, screen_y)
        if position is None:
            return
        x, y, z = position
        robot = "drone" if mode == "add_drone" else "husky"
        self.waypoint_store.add_manual_waypoint(
            robot=robot, x=x, y=y, z=0.0
        )
        self.map_tool_panel.reset_to_select()

    # =====================================================
    # MAP OVERLAYS
    # =====================================================
    def create_map_overlay(self):

        self.map_overlay = MapOverlayPanel(
            status_colours=self.STATUS_COLOURS,
            waypoint_store=self.waypoint_store,
            layer_callback=self.set_layer_visible,
            parent=self.map_view.plotter.interactor,
        )
    def create_map_tool_panel(self):
        self.map_tool_panel = MapToolPanel(
            parent=self.map_view.plotter.interactor,
        )
        self.map_tool_panel.mode_changed.connect(
            self.set_map_tool_mode
        )

    def create_selected_hazard_overlay(self):
        self.selected_hazard_panel = SelectedHazardPanel(
            status_colours=self.STATUS_COLOURS,
            parent=self.map_view.plotter.interactor,
        )
        self.selected_hazard_panel.view_full_record_requested.connect(
            self.open_selected_full_record
        )
        self.selected_hazard_panel.centre_on_map_requested.connect(
            self.center_selected_hazard
        )
    def position_map_overlays(self):
        if not all(hasattr(self, name) for name in (
            "map_view", "map_overlay", "map_tool_panel", "selected_hazard_panel"
        )):
            return
        viewport = self.map_view.plotter.interactor
        width = viewport.width()
        height = viewport.height()
        # Map overlay: Legend + Waypoints
        overlay_width = min(
            305,
            max(275, int(width * 0.36)),
        )
        overlay_height = min(
            535,
            height - 20,
        )
        self.map_overlay.position_in_viewport(
            y=10,
            width=overlay_width,
            height=overlay_height,
            left=10,
        )
        # =====================================================
        # TOP-RIGHT MAP TOOL PANEL
        # =====================================================
        tool_width = 155
        tool_height = 132
        tool_x = max(10, width - tool_width - 12)
        tool_y = 10
        if hasattr(self, "map_tool_panel"):
            self.map_tool_panel.setGeometry(
                tool_x,
                tool_y,
                tool_width,
                tool_height,
            )
        # =====================================================
        # SELECTED HAZARD OVERLAY
        # =====================================================
        quick_width = min(
            255,
            max(220, int(width * 0.30)),
        )
        quick_height = min(
            350,
            max(300, height - 90),
        )
        quick_x = max(
            10,
            width - quick_width - 12,
        )
        # Position underneath map tools
        quick_y = tool_y + tool_height + 8
        self.selected_hazard_panel.setGeometry(
            quick_x,
            quick_y,
            quick_width,
            min(
                quick_height,
                max(100, height - quick_y - 10),
            ),
        )
        self.refresh_map_overlays()
    # =====================================================
    # MAP CONTENT
    # =====================================================
    def set_layer_visible(self, layer_name, visible):
        """Translate existing legend layer names to 3D SVG layers."""
        if layer_name == "drone":
            for name in ("drone_location", "drone_home", "drone_waypoint"):
                self.map_view.set_layer_visible(name, visible)
        elif layer_name == "husky":
            for name in ("husky_location", "husky_home", "husky_waypoint"):
                self.map_view.set_layer_visible(name, visible)
        else:
            self.map_view.set_layer_visible(layer_name, visible)

    def set_map_robot_view(self, view_name):
        for button, name in (
            (self.map_drone_button, "Drone"),
            (self.map_husky_button, "Husky"),
            (self.map_both_button, "Both"),
        ):
            button.blockSignals(True)
            button.setChecked(name == view_name)
            button.blockSignals(False)
        show_drone = view_name in (
            "Drone",
            "Both",
        )
        show_husky = view_name in (
            "Husky",
            "Both",
        )
        self.set_layer_visible(
            "drone",
            show_drone,
        )
        self.set_layer_visible(
            "husky",
            show_husky,
        )
        legend = self.map_overlay.legend_panel
        legend.drone_layer_check.blockSignals(True)
        legend.husky_layer_check.blockSignals(True)
        legend.drone_layer_check.setChecked(
            show_drone
        )
        legend.husky_layer_check.setChecked(
            show_husky
        )
        legend.drone_layer_check.blockSignals(False)
        legend.husky_layer_check.blockSignals(False)
    # =====================================================
    # HAZARD SELECTION / STATE
    # =====================================================
    def select_hazard_from_map(self, hazard_id):
        """Show the existing hazard detail overlay for a valid hazard."""
        hazard_id = str(hazard_id).strip()
        if not self.hazard_store.exists(hazard_id):
            return False

        hazard = self.hazard_store.get(hazard_id)
        self.selected_hazard_id = hazard_id
        self.selected_hazard_panel.set_hazard(hazard_id, hazard)
        self.selected_hazard_panel.show()
        self.selected_hazard_panel.raise_()
        return True

    def on_hazard_waypoint_clicked(self, hazard_id):
        """Open a linked hazard only when the map's Select tool is active."""
        if getattr(self, "current_map_tool_mode", "select") != "select":
            return False
        if hazard_id is None:
            return False
        return self.select_hazard_from_map(hazard_id)

    def open_selected_full_record(self):
        if not self.selected_hazard_id:
            return
        self.tabs.setCurrentIndex(1)
        self.show_hazard_details(self.selected_hazard_id)
    def center_selected_hazard(self):
        if not self.selected_hazard_id:
            return
        # 3D hazard centring requires Gazebo-world hazard coordinates.
        # The previous QGraphicsScene marker is no longer used.
        pass
    def dismiss_selected_hazard(self):
        if not self.selected_hazard_id:
            return
        self.append_log(f"{self.selected_hazard_id} dismissed by operator")
        self.selected_hazard_panel.hide()
    def update_outcome_counts(self):
        self.mission_summary.update_outcomes(self.hazards)

    def sync_hazard_markers(self):
        """Synchronise database hazards with their 3D map SVG markers."""
        overlay = self.map_view.marker_overlay
        icon_dir = PACKAGE_ROOT / "assets" / "hazard_icons"

        # HazardStore supplies the type stored in the SQLite database.
        # RosTargetSync currently creates drone targets with type "Ice".
        icon_files = {
            "ice": "line_ice.svg",
            "ice on line": "line_ice.svg",
            "husky obstacle": "ground_hazard.svg",
            "ground hazard": "ground_hazard.svg",
        }

        active_ids = set()

        for hazard_id, hazard in self.hazards.items():
            marker_id = f"hazard_{hazard_id}"
            coords = hazard.get("coords")
            if not coords or len(coords) != 3:
                continue

            try:
                x, y, z = map(float, coords)
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue

            hazard_type = str(hazard.get("type") or "").strip().lower()
            icon_name = icon_files.get(hazard_type)
            if icon_name is None:
                print(f"Unknown hazard type {hazard_type!r} for {hazard_id}")
                continue

            icon_path = icon_dir / icon_name
            if not icon_path.is_file():
                print(f"Missing hazard SVG: {icon_path}")
                continue

            active_ids.add(marker_id)
            icon_key = str(icon_path)

            # MarkerOverlay fixes the SVG when a marker is created.
            # Recreate it if a hazard's type (and icon) changes.
            if (marker_id in overlay.markers
                    and self.hazard_markers.get(marker_id) != icon_key):
                overlay.remove_marker(marker_id)

            if marker_id not in overlay.markers:
                overlay.add_marker(
                    marker_id=marker_id,
                    svg_path=icon_key,
                    size=24,
                )

            overlay.set_position(marker_id, x, y, z)
            self.hazard_markers[marker_id] = icon_key

        for marker_id in list(self.hazard_markers):
            if marker_id not in active_ids:
                overlay.remove_marker(marker_id)
                del self.hazard_markers[marker_id]

    def on_hazards_changed(self):
        self.hazards = self.hazard_store.get_all()
        self.update_outcome_counts()
        self.sync_hazard_markers()

        if (
            self.selected_hazard_id
            and self.hazard_store.exists(self.selected_hazard_id)
        ):
            self.selected_hazard_panel.set_hazard(
                self.selected_hazard_id,
                self.hazard_store.get(self.selected_hazard_id),
            )

    # =====================================================
    # RIGHT PANEL
    # =====================================================
    def create_right_panel(self):
        panel = QFrame()
        panel.setObjectName("mainPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 6, 8, 8)
        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_live_tab(), "Live Operations")
        self.tabs.addTab(self.create_hazard_tab(), "Documented Hazards")
        layout.addWidget(self.tabs)
        return panel
    # =====================================================
    # LIVE OPERATIONS
    # =====================================================
    def create_live_tab(self):
        self.live_operations = LiveOperations()

        self.husky_camera_sub = self.ros_node.create_subscription(
            Image,
            "/husky1/camera/image",
            self.husky_camera_callback,
            10,
        )

        self.husky_odom_sub = self.ros_node.create_subscription(
            Odometry,
            "/husky1/odometry",
            self.husky_odometry_callback,
            10,
        )

        self.drone_camera_sub = self.ros_node.create_subscription(
            Image,
            "/fde/debug_image",
            self.drone_camera_callback,
            10,
        )

        self.drone_odom_sub = self.ros_node.create_subscription(
            Odometry,
            "/parrot1/odometry",
            self.drone_odometry_callback,
            10,
        )

        return self.live_operations
    # =====================================================
    # DOCUMENTED HAZARDS
    # =====================================================
    def create_hazard_tab(self):
        self.documented_hazards = DocumentedHazards(
            hazard_store=self.hazard_store,
            status_colours=self.STATUS_COLOURS,
            waypoint_store=self.waypoint_store,
        )
        self.documented_hazards.hazard_selected.connect(
            self.select_hazard_from_map
        )
        return self.documented_hazards

    def show_hazard_details(self, hazard_id):
        if not self.hazard_store.exists(hazard_id):
            return

        self.selected_hazard_id = hazard_id
        self.documented_hazards.show_hazard(hazard_id)

    # =====================================================
    # ROS
    # =====================================================
    def process_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.setWindowTitle(
            "FDE Alpine Hazard Inspection"
            f" — {self.ros_node.mission_state}"
        )
    def append_log(self, text):
        if hasattr(self, "live_operations"):
            self.live_operations.append_log(text)
    # =====================================================
    # WINDOW EVENTS
    # =====================================================
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.position_map_overlays()

        # Keep map controls above SVG markers and waypoint numbers.
        for name in ("map_overlay", "map_tool_panel", "selected_hazard_panel"):
            panel = getattr(self, name, None)
            if panel is not None:
                panel.raise_()


    def eventFilter(self, watched, event):
        if (hasattr(self, "map_view")
                and watched is self.map_view.plotter.interactor):
            if event.type() == QEvent.Resize:
                QTimer.singleShot(0, self.position_map_overlays)
            elif (event.type() == QEvent.MouseButtonPress
                  and event.button() == Qt.LeftButton
                  and getattr(self, "current_map_tool_mode", "select")
                  in ("add_drone", "add_husky", "remove")):
                position = event.position()
                self.on_map_clicked(position.x(), position.y())
                return True
        return super().eventFilter(watched, event)

    def closeEvent(self, event):
        self.ros_timer.stop()
        if hasattr(self, "map_view"):
            self.map_view.marker_overlay.on_refresh = None
            self.map_view.close()
        self.ros_node.destroy_node()
        if hasattr(self, "database"):
            self.database.close()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()
    # =====================================================
    # STYLE
    # =====================================================
    def apply_style(self):
        theme_path = (
            PACKAGE_ROOT
            / "operator_interface"
            / "styles"
            / "theme.qss"
        )
        if not theme_path.exists():
            print(f"WARNING: Theme file not found: {theme_path}")
            return
        with open(theme_path, "r", encoding="utf-8") as file:
            self.setStyleSheet(file.read())

    def husky_camera_callback(self, msg):

        image = QImage(
            msg.data,
            msg.width,
            msg.height,
            msg.step,
            QImage.Format_RGB888,
        ).copy()

        self.live_operations.update_husky_camera(
            image
        )

    def drone_camera_callback(self, msg):

        image = QImage(
            msg.data,
            msg.width,
            msg.height,
            msg.step,
            QImage.Format_RGB888,
        ).copy()

        self.live_operations.update_drone_camera(
            image
        )

    def quaternion_to_heading(self, orientation):

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Quaternion -> yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        heading = math.degrees(yaw)

        # Convert -180..180 to 0..360
        heading = (heading + 360.0) % 360.0

        return heading

    def husky_odometry_callback(self, msg):

        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        velocity = math.sqrt(
            vx ** 2 +
            vy ** 2
        )

        # Heading
        heading = self.quaternion_to_heading(
            msg.pose.pose.orientation
        )

        self.live_operations.update_husky_telemetry(
            x,
            y,
            velocity,
            heading,
        )

    def drone_odometry_callback(self, msg):

        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Relative altitude from odometry origin
        relative_altitude = msg.pose.pose.position.z

        # Velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        velocity = math.sqrt(
            vx ** 2 +
            vy ** 2 +
            vz ** 2
        )

        # Heading
        heading = self.quaternion_to_heading(
            msg.pose.pose.orientation
        )

        self.live_operations.update_drone_telemetry(
            x,
            y,
            relative_altitude,
            velocity,
            heading,
        )

    def set_map_tool_mode(self, mode):
        self.current_map_tool_mode = mode

        if mode == "select":
            self.map_view.plotter.interactor.setCursor(Qt.ArrowCursor)

        elif mode in ("add_drone", "add_husky"):
            self.map_view.plotter.interactor.setCursor(Qt.CrossCursor)

        elif mode == "remove":
            self.map_view.plotter.interactor.setCursor(Qt.CrossCursor)

    def open_manual_waypoint_dialog(
        self,
        robot,
    ):

        dialog = ManualWaypointDialog(
            robot=robot,
            parent=self,
        )

        if dialog.exec() != QDialog.Accepted:
            return

        x, y, z = dialog.coordinates()

        waypoint = (
            self.waypoint_store.add_manual_waypoint(
                robot=robot,
                x=x,
                y=y,
                z=z,
            )
        )

        if waypoint is None:
            return

        print(
            f"[Waypoint] Added "
            f"{waypoint['id']} "
            f"to {robot}: "
            f"({x:.2f}, {y:.2f}, {z:.2f})"
        )


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    ros_node = OperatorRosNode()
    app = QApplication(sys.argv)
    window = OperatorGUI(ros_node)
    window.show()
    exit_code = app.exec()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)
if __name__ == "__main__":
    main()
