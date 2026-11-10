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
import time
import math
from operator_interface.data.ros_target_sync import RosTargetSync
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from PySide6.QtGui import QImage
try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None
from rclpy.node import Node
from PySide6.QtCore import Qt, Signal, QPointF
from std_msgs.msg import String
from operator_interface.widgets.missions.mission_summary import MissionSummary
from operator_interface.widgets.map.map_overlay_panel import MapOverlayPanel
from operator_interface.widgets.operations.live_operations import LiveOperations
from operator_interface.widgets.hazards.documented_hazards import (
    DocumentedHazards,
)
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
        self.mission_state_callback_gui = None
        self.command_pub = self.create_publisher(
            String,
            "/fde/mission_command",
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/fde/markers",
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
        previous = self.mission_state
        self.mission_state = msg.data
        if self.mission_state_callback_gui is not None and previous != msg.data:
            self.mission_state_callback_gui(msg.data)
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

    def clear_markers(self):
        arr = MarkerArray()
        stale = Marker()
        stale.action = Marker.DELETEALL
        arr.markers.append(stale)
        self.marker_pub.publish(arr)
        self.get_logger().info("Dispatched Marker.DELETEALL on /fde/markers")

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
        "Inconclusive": "#ffc107",
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

        # Clear stale records and markers from previous runs so every launch starts clean
        self.hazard_store.clear_all()
        self.ros_node.clear_markers()

        self.mission_run_id = os.environ.get(
            "FDE_MISSION_RUN_ID"
        ) or uuid.uuid4().hex[:8].upper()

        self.ros_target_sync = RosTargetSync(
            self.hazard_store,
            self.mission_run_id,
        )

        self.ros_node.targets_callback_gui = self.process_target_update
        self.ros_node.mission_state_callback_gui = self.on_mission_state_event
        self._last_target_states = {}
        self._last_progress_event = None
        self._robot_motion = {}

        # Image evidence: one camera snapshot per newly observed ROS target.
        # Files persist across GUI restarts and are keyed by the full hazard ID.
        self.snapshot_dir = Path.home() / ".local" / "share" / "fde_gui" / "hazard_snapshots"
        self._latest_drone_frame = None  # (QImage, monotonic_time, source)
        self._pending_snapshots = {}      # hazard_id -> first_observed_time
        self._snapshot_seen_targets = set()

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
        self.ros_node.mission_progress_callback_gui = self.on_mission_progress_event
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
        self.start_button.clicked.connect(self.on_start_mission)
        self.abort_button = QPushButton("Return Home / Abort")
        self.abort_button.setObjectName("abortButton")
        self.abort_button.clicked.connect(lambda: self.ros_node.send_command("abort"))
        commands.addWidget(self.start_button)
        commands.addWidget(self.abort_button)
        layout.addLayout(commands)
        return panel

    def on_start_mission(self):
        self._last_target_states.clear()
        self._last_progress_event = None
        self.append_robot_log("SYSTEM", "Operator requested mission start")
        if hasattr(self, "hazard_store"):
            self.hazard_store.clear_all()
        if hasattr(self, "ros_node"):
            self.ros_node.clear_markers()
            self.ros_node.send_command("start")

    def refresh_map_overlays(self):
        """Keep floating GUI panels above all SVG markers and number labels."""
        self.map_view.waypoint_markers.update_label_positions()
        for name in ("map_overlay", "selected_hazard_panel"):
            panel = getattr(self, name, None)
            if panel is not None:
                panel.raise_()

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

        legend = self.map_overlay.legend_panel

        # Hazard status filters
        self.status_switches = {
            "Inconclusive": legend.detected_check,
            "Husky Required": legend.husky_required_check,
            "In Progress": legend.in_progress_check,
            "Resolved / Safe": legend.resolved_check,
            "Confirmed Hazard": legend.confirmed_check,
        }

        # Hazard type filters
        self.type_switches = {
            "ice": legend.line_ice_check,
            "ice on line": legend.line_ice_check,
        }

        # Refresh marker visibility whenever a switch changes.
        switches = [
            legend.hazard_status_check,
            legend.hazard_layer_check,
            *self.status_switches.values(),
            *self.type_switches.values(),
        ]

        for switch in set(switches):
            switch.toggled.connect(lambda _checked: self.sync_hazard_markers())

        # Individual robot markers must respect both their own toggle and
        # the parent Drone/Husky toggle. The parent toggles are already wired
        # to set_layer_visible by LegendPanel.
        robot_children = (
            legend.drone_location_check,
            legend.drone_waypoint_check,
            legend.drone_home_check,
            legend.husky_location_check,
            legend.husky_waypoint_check,
            legend.husky_home_check,
        )
        for switch in robot_children:
            switch.toggled.connect(
                lambda _checked: self.apply_robot_layer_filters()
            )

        # Initialise visibility from the actual legend state.
        self.apply_robot_layer_filters()

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
            "map_view", "map_overlay", "selected_hazard_panel"
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
        # Position near the top-right now that the map tool panel is removed.
        quick_y = 10
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
    def apply_robot_layer_filters(self):
        """Apply master AND child switches to each robot marker layer."""
        legend = self.map_overlay.legend_panel

        robot_layers = {
            "drone": {
                "drone_location": legend.drone_location_check,
                "drone_waypoint": legend.drone_waypoint_check,
                "drone_home": legend.drone_home_check,
            },
            "husky": {
                "husky_location": legend.husky_location_check,
                "husky_waypoint": legend.husky_waypoint_check,
                "husky_home": legend.husky_home_check,
            },
        }

        for robot_name, children in robot_layers.items():
            master = (
                legend.drone_layer_check if robot_name == "drone"
                else legend.husky_layer_check
            )
            for layer_name, child_switch in children.items():
                self.map_view.set_layer_visible(
                    layer_name,
                    master.isChecked() and child_switch.isChecked(),
                )

    def set_layer_visible(self, layer_name, visible):
        """Handle legend layer changes while preserving child toggle states."""
        robot_layers = {
            "drone", "husky",
            "drone_location", "drone_waypoint", "drone_home",
            "husky_location", "husky_waypoint", "husky_home",
        }
        if layer_name in robot_layers:
            self.apply_robot_layer_filters()
        elif layer_name != "hazards":
            self.map_view.set_layer_visible(layer_name, visible)
        # Hazard marker visibility is controlled separately by
        # sync_hazard_markers(), with status and type filters combined.

    def set_map_robot_view(self, view_name):
        """Synchronise Drone/Husky/Both buttons with legend master toggles."""
        for button, name in (
            (self.map_drone_button, "Drone"),
            (self.map_husky_button, "Husky"),
            (self.map_both_button, "Both"),
        ):
            button.blockSignals(True)
            button.setChecked(name == view_name)
            button.blockSignals(False)

        show_drone = view_name in ("Drone", "Both")
        show_husky = view_name in ("Husky", "Both")
        legend = self.map_overlay.legend_panel

        # The legend master switches already have their own callbacks;
        # suppress those briefly and apply the combined state once.
        legend.drone_layer_check.blockSignals(True)
        legend.husky_layer_check.blockSignals(True)
        legend.drone_layer_check.setChecked(show_drone)
        legend.husky_layer_check.setChecked(show_husky)
        legend.drone_layer_check.blockSignals(False)
        legend.husky_layer_check.blockSignals(False)
        self.apply_robot_layer_filters()

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

    def hazard_marker_visible(self, hazard):

        legend = self.map_overlay.legend_panel

        # Master switches
        if not legend.hazard_status_check.isChecked():
            return False

        if not legend.hazard_layer_check.isChecked():
            return False

        # Hazard status
        status = str(
            hazard.get("status") or "Inconclusive"
        ).strip()

        status_switch = self.status_switches.get(status)

        if status_switch is None:
            return False

        if not status_switch.isChecked():
            return False

        # Hazard type
        hazard_type = str(
            hazard.get("type") or ""
        ).strip().lower()

        type_switch = self.type_switches.get(hazard_type)

        if type_switch is None:
            return False

        if not type_switch.isChecked():
            return False

        return True

    def update_hazard_status_indicator(
        self,
        marker_id,
        status,
    ):
        overlay = self.map_view.marker_overlay

        marker = overlay.markers.get(marker_id)

        if marker is None:
            return

        widget = marker["widget"]

        # Create status indicator if it doesn't exist.
        badge = getattr(widget, "status_badge", None)

        if badge is None:

            badge = QLabel("●", widget)

            badge.setFixedSize(14, 14)

            badge.setAlignment(Qt.AlignCenter)

            badge.setAttribute(
                Qt.WA_TransparentForMouseEvents,
                True,
            )

            widget.status_badge = badge

        colour = self.STATUS_COLOURS.get(
            status,
            "#ffffff",
        )

        badge.setStyleSheet(
            f"""
            QLabel {{
                color: {colour};
                font-size: 15px;
                background: transparent;
            }}
            """
        )

        # Bottom-right corner of the existing SVG.
        badge.move(
            widget.width() - badge.width(),
            widget.height() - badge.height(),
        )

        badge.show()
        badge.raise_()

    def sync_hazard_markers(self):
        """Synchronise database hazards with filtered 3D SVG markers."""
        overlay = self.map_view.marker_overlay
        icon_dir = PACKAGE_ROOT / "assets" / "hazard_icons"

        # Match hazard types from the database to existing SVG assets.
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

            # MarkerOverlay fixes the icon when created. Replace only if
            # the underlying hazard type / SVG asset changes.
            if (marker_id in overlay.markers
                    and self.hazard_markers.get(marker_id) != icon_key):
                overlay.remove_marker(marker_id)

            if marker_id not in overlay.markers:
                overlay.add_marker(
                    marker_id=marker_id,
                    svg_path=icon_key,
                    size=24,
                )

            status = str(hazard.get("status") or "Inconclusive").strip()
            self.update_hazard_status_indicator(marker_id, status)

            widget = overlay.markers[marker_id]["widget"]
            widget.setToolTip(
                f"{hazard_id} | {hazard.get('type', 'Unknown')} | {status}"
            )

            overlay.set_position(marker_id, x, y, z)
            overlay.set_visible(marker_id, self.hazard_marker_visible(hazard))
            self.hazard_markers[marker_id] = icon_key

        # Remove markers whose records disappeared or became invalid.
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
        self._cv_bridge = CvBridge() if CvBridge else None
        self.last_drone_debug_time = 0.0
        self.last_husky_debug_time = 0.0

        # Husky camera: listen to confirm debug video and fallback to raw camera
        self.husky_debug_sub = self.ros_node.create_subscription(
            Image,
            "/husky1/ice_confirm/debug_image",
            self.husky_debug_camera_callback,
            qos_profile_sensor_data,
        )
        self.husky_camera_sub = self.ros_node.create_subscription(
            Image,
            "/husky1/camera/image",
            self.husky_camera_callback,
            qos_profile_sensor_data,
        )
        self.husky_odom_sub = self.ros_node.create_subscription(
            Odometry,
            "/husky1/odometry",
            self.husky_odometry_callback,
            qos_profile_sensor_data,
        )

        # Drone camera: listen to detector debug video and fallback to raw camera
        self.drone_debug_sub = self.ros_node.create_subscription(
            Image,
            "/fde/debug_image",
            self.drone_debug_camera_callback,
            qos_profile_sensor_data,
        )
        self.drone_raw_sub = self.ros_node.create_subscription(
            Image,
            "/parrot1/camera/image",
            self.drone_raw_camera_callback,
            qos_profile_sensor_data,
        )
        self.drone_odom_sub = self.ros_node.create_subscription(
            Odometry,
            "/parrot1/odometry",
            self.drone_odometry_callback,
            qos_profile_sensor_data,
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
    # LIVE ROBOT ACTION LOGS (EVENT DRIVEN)
    # =====================================================
    def append_robot_log(self, robot, message):
        """Append an event observed from ROS, with a local timestamp."""
        stamp = time.strftime("%H:%M:%S")
        self.append_log(f"[{stamp}] [{robot}] {message}")

    def log_robot_motion(self, robot, x, y, velocity, altitude=None):
        """Log actual odometry activity without spamming on every ROS update."""
        now = time.monotonic()
        record = self._robot_motion.get(robot)
        moving = velocity >= 0.35

        if record is None:
            self.append_robot_log(
                robot, f"Odometry received at X={x:.1f}, Y={y:.1f}"
            )
            record = {"moving": False, "last_log": now, "x": x, "y": y}
            self._robot_motion[robot] = record

        if moving != record["moving"]:
            if moving:
                self.append_robot_log(
                    robot,
                    f"Movement detected: {velocity:.1f} m/s at X={x:.1f}, Y={y:.1f}"
                )
            else:
                self.append_robot_log(
                    robot, f"Robot stationary at X={x:.1f}, Y={y:.1f}"
                )
            record.update(moving=moving, last_log=now, x=x, y=y)
            return

        distance = math.hypot(x - record["x"], y - record["y"])
        if moving and now - record["last_log"] >= 15.0 and distance >= 2.0:
            detail = f"Moving: {velocity:.1f} m/s, X={x:.1f}, Y={y:.1f}"
            if altitude is not None:
                detail += f", relative alt={altitude:.1f} m"
            self.append_robot_log(robot, detail)
            record.update(last_log=now, x=x, y=y)

    def queue_new_target_snapshots(self, data):
        """Capture evidence only once, when a ROS ice target first appears."""
        if not isinstance(data, dict):
            return
        targets = data.get("targets", [])
        if not isinstance(targets, list):
            return

        now = time.monotonic()
        for target in targets:
            if not isinstance(target, dict):
                continue
            target_id = str(target.get("id", "")).strip()
            if not (target_id.startswith("T") and target_id[1:].isdigit()):
                continue
            # RosTargetSync ignores targets without finite world coordinates.
            try:
                coords = (float(target["x"]), float(target["y"]), float(target["z"]))
                if not all(math.isfinite(v) for v in coords):
                    continue
            except (KeyError, TypeError, ValueError):
                continue

            hazard_id = f"H-ROS-{self.mission_run_id}-{target_id}"
            if hazard_id in self._snapshot_seen_targets:
                continue
            self._snapshot_seen_targets.add(hazard_id)
            destination = self.snapshot_dir / f"{hazard_id}.png"
            if not destination.exists():
                self._pending_snapshots[hazard_id] = now

        self.capture_pending_snapshots()

    def capture_pending_snapshots(self):
        """Use a nearby camera frame; do not attach stale frames to hazards."""
        if not self._pending_snapshots:
            return

        now = time.monotonic()
        # Discard requests if no camera image arrives close to detection.
        for hazard_id, detected_at in list(self._pending_snapshots.items()):
            if now - detected_at > 3.0:
                del self._pending_snapshots[hazard_id]

        frame_record = self._latest_drone_frame
        if frame_record is None:
            return
        frame, frame_time, source = frame_record
        if now - frame_time > 1.5 or frame.isNull():
            return

        for hazard_id in list(self._pending_snapshots):
            try:
                self.snapshot_dir.mkdir(parents=True, exist_ok=True)
                path = self.snapshot_dir / f"{hazard_id}.png"
                if path.exists() or frame.save(str(path), "PNG"):
                    del self._pending_snapshots[hazard_id]
                    self.append_robot_log(
                        "DRONE", f"Snapshot saved for {hazard_id} ({source})"
                    )
            except OSError as exc:
                self.append_robot_log("DRONE", f"Snapshot save failed: {exc}")
                del self._pending_snapshots[hazard_id]

    def cache_drone_frame(self, msg, source):
        """Cache the latest decoded QImage for the hazard evidence pipeline."""
        img = self._imgmsg_to_qimage(msg)
        if img is None or img.isNull():
            return None
        self._latest_drone_frame = (img, time.monotonic(), source)
        self.capture_pending_snapshots()
        return img

    def on_mission_state_event(self, state):
        state_text = str(state).strip()
        if state_text:
            self.append_robot_log("MISSION", f"State changed to {state_text}")

    def on_mission_progress_event(self, message):
        # Preserve the existing mission-progress widget updates.
        self.mission_progress.process_event(message)
        event = str(message).strip()
        if event and event != self._last_progress_event:
            self._last_progress_event = event
            # Raw ROS event: do not infer unreported robot actions.
            self.append_robot_log("MISSION", event)

    def process_target_update(self, data):
        """Synchronize target records and log only changes in target state."""
        # Preserve the existing hazard synchronization.
        self.ros_target_sync.process(data)
        self.queue_new_target_snapshots(data)
        if not isinstance(data, dict):
            return
        targets = data.get("targets", [])
        if not isinstance(targets, list):
            return

        state_messages = {
            "tracking": ("DRONE", "tracking ice target"),
            "ready": ("DRONE", "ice target ready for Husky intervention"),
            "active": ("HUSKY", "intervention active for ice target"),
            "confirmed": ("HUSKY", "ice target confirmed"),
            "not_confirmed": ("HUSKY", "ice target not confirmed"),
            "inconclusive": ("HUSKY", "ice target assessment inconclusive"),
            "missed": ("MISSION", "ice target missed"),
        }
        for target in targets:
            if not isinstance(target, dict):
                continue
            target_id = str(target.get("id", "")).strip()
            state = str(target.get("state", "")).strip().lower()
            if not (target_id.startswith("T") and target_id[1:].isdigit()):
                continue
            if state not in state_messages:
                continue
            if self._last_target_states.get(target_id) == state:
                continue
            self._last_target_states[target_id] = state
            robot, description = state_messages[state]
            coords = ""
            try:
                x = float(target["x"])
                y = float(target["y"])
                if math.isfinite(x) and math.isfinite(y):
                    coords = f" at X={x:.1f}, Y={y:.1f}"
            except (KeyError, TypeError, ValueError):
                pass
            self.append_robot_log(robot, f"{description} {target_id}{coords}")

    # =====================================================
    # ROS
    # =====================================================
    def process_ros(self):
        for _ in range(5):
            rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.setWindowTitle(
            "FDE Alpine Hazard Inspection"
            f" — {self.ros_node.mission_state}"
        )
        if hasattr(self, "live_operations"):
            self.live_operations.update_timestamp()
    def append_log(self, text):
        if hasattr(self, "live_operations"):
            log = self.live_operations.action_log
            if log.toPlainText().strip() == "Waiting for mission...":
                log.clear()
            self.live_operations.append_log(text)
            # Bound log history to avoid unbounded growth during long missions.
            document = log.document()
            document.setMaximumBlockCount(500)
    # =====================================================
    # WINDOW EVENTS
    # =====================================================
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.position_map_overlays()

        # Keep map controls above SVG markers and waypoint numbers.
        for name in ("map_overlay", "selected_hazard_panel"):
            panel = getattr(self, name, None)
            if panel is not None:
                panel.raise_()


    def eventFilter(self, watched, event):
        if (hasattr(self, "map_view")
                and watched is self.map_view.plotter.interactor):
            if event.type() == QEvent.Resize:
                QTimer.singleShot(0, self.position_map_overlays)
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

    def _imgmsg_to_qimage(self, msg):
        try:
            if not msg or not msg.data or msg.width <= 0 or msg.height <= 0:
                return None
            import numpy as np
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            encoding = str(getattr(msg, "encoding", "") or "").lower()
            if "bgr8" in encoding:
                rgb = np.ascontiguousarray(raw.reshape((msg.height, msg.width, 3))[:, :, ::-1])
                return QImage(rgb.data, msg.width, msg.height, 3 * msg.width, QImage.Format_RGB888).copy()
            elif "rgb8" in encoding:
                rgb = np.ascontiguousarray(raw.reshape((msg.height, msg.width, 3)))
                return QImage(rgb.data, msg.width, msg.height, 3 * msg.width, QImage.Format_RGB888).copy()
            elif "bgra8" in encoding:
                rgba = np.ascontiguousarray(raw.reshape((msg.height, msg.width, 4))[:, :, [2, 1, 0, 3]])
                return QImage(rgba.data, msg.width, msg.height, 4 * msg.width, QImage.Format_RGBA8888).copy()
            elif "rgba8" in encoding:
                rgba = np.ascontiguousarray(raw.reshape((msg.height, msg.width, 4)))
                return QImage(rgba.data, msg.width, msg.height, 4 * msg.width, QImage.Format_RGBA8888).copy()
            elif "mono8" in encoding or "8uc1" in encoding:
                mono = np.ascontiguousarray(raw.reshape((msg.height, msg.width)))
                return QImage(mono.data, msg.width, msg.height, msg.width, QImage.Format_Grayscale8).copy()
            else:
                return QImage(bytes(msg.data), msg.width, msg.height, msg.step, QImage.Format_RGB888).copy()
        except Exception:
            return None

    def husky_debug_camera_callback(self, msg):
        self.last_husky_debug_time = time.monotonic()
        img = self._imgmsg_to_qimage(msg)
        if img is not None:
            self.live_operations.update_husky_camera(img)

    def husky_camera_callback(self, msg):
        if time.monotonic() - getattr(self, "last_husky_debug_time", 0.0) > 1.5:
            img = self._imgmsg_to_qimage(msg)
            if img is not None:
                self.live_operations.update_husky_camera(img)

    def drone_debug_camera_callback(self, msg):
        self.last_drone_debug_time = time.monotonic()
        img = self.cache_drone_frame(msg, "detector debug")
        if img is not None:
            self.live_operations.update_drone_camera(img)

    def drone_raw_camera_callback(self, msg):
        # Prefer annotated detector frames while they are available.
        if time.monotonic() - getattr(self, "last_drone_debug_time", 0.0) > 1.5:
            img = self.cache_drone_frame(msg, "raw camera")
            if img is not None:
                self.live_operations.update_drone_camera(img)

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
        self.log_robot_motion("HUSKY", x, y, velocity)

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
        self.log_robot_motion("DRONE", x, y, velocity, relative_altitude)

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