
from pathlib import Path

import rclpy
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QWidget, QVBoxLayout
from pyvistaqt import QtInteractor

from .world_loader import WorldLoader
from .camera_controller import BoundedTopDownCamera
from .robot_pose_subscriber import RobotPoseSubscriber

from .markers.marker_overlay import MarkerOverlay
from .markers.robot_markers import RobotMarkers
from .markers.waypoint_markers import WaypointMarkers


class Map3DWidget(QWidget):
    """Reusable 3D environment viewport for the mission GUI."""

    def __init__(
        self,
        repo_root=None,
        parent=None,
        waypoint_store=None,
    ):
        super().__init__(parent)

        # =====================================================
        # REPOSITORY PATHS
        # =====================================================

        if repo_root is None:
            repo_root = Path(__file__).resolve().parents[5]

        self.repo_root = Path(repo_root)
        self.world_file = self.repo_root / "worlds" / "snow_forest.sdf"
        self.models_dir = self.repo_root / "models"

        if not self.world_file.is_file():
            raise FileNotFoundError(
                f"World SDF not found: {self.world_file}"
            )

        self.setFocusPolicy(Qt.StrongFocus)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # =====================================================
        # PYVISTA RENDERER
        # =====================================================

        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)

        self.plotter.set_background("#e9eef3")

        # =====================================================
        # STATIC GAZEBO ENVIRONMENT
        # =====================================================

        self.world_loader = WorldLoader(self.models_dir)

        self.world_loader.load_world(
            self.plotter,
            self.world_file,
        )

        # =====================================================
        # CAMERA CONTROLLER
        # =====================================================

        self.camera_controller = BoundedTopDownCamera(
            self.plotter,
            world_bounds=(-30.0, 90.0, -55.0, 55.0),
            max_zoom=8.0,
        )

        # =====================================================
        # SHARED SVG MARKER OVERLAY
        # =====================================================

        self.marker_overlay = MarkerOverlay(
            plotter=self.plotter,
            parent=self,
        )

        # =====================================================
        # ROBOT LOCATION AND HOME MARKERS
        # =====================================================

        self.robot_markers = RobotMarkers(
            marker_overlay=self.marker_overlay,
        )

        # Temporary simulation home coordinates.
        # Replace with mission-configured home positions.
        self.robot_markers.set_drone_home(
            x=2.0,
            y=0.0,
            z=0.0,
        )

        self.robot_markers.set_husky_home(
            x=0.0,
            y=0.0,
            z=0.0,
        )

        # =====================================================
        # WAYPOINT SVG MARKERS
        # =====================================================

        self.waypoint_markers = WaypointMarkers(
            marker_overlay=self.marker_overlay,
        )

        # Keep waypoint numbers aligned during pan/zoom.
        self.marker_overlay.on_refresh = (
            self.waypoint_markers.update_label_positions
        )

        # =====================================================
        # SHARED WAYPOINT STORE
        # =====================================================

        self.waypoint_store = None

        if waypoint_store is not None:
            self.connect_waypoint_store(waypoint_store)

        # =====================================================
        # ROS 2 INITIALIZATION
        # =====================================================

        self.owns_ros_context = not rclpy.ok()

        if self.owns_ros_context:
            rclpy.init()

        self.robot_subscriber = RobotPoseSubscriber()

        # =====================================================
        # ROS UPDATE TIMER
        # =====================================================

        self.robot_timer = QTimer(self)

        self.robot_timer.timeout.connect(
            self.update_robot_positions
        )

        self.robot_timer.start(50)

    # =========================================================
    # ROBOT POSITION UPDATES
    # =========================================================

    def update_robot_positions(self):
        """Update Drone and Husky markers from ROS 2 odometry."""

        rclpy.spin_once(
            self.robot_subscriber,
            timeout_sec=0.0,
        )

        for name, pose in self.robot_subscriber.latest_poses.items():

            if name == "parrot1":
                self.robot_markers.update_drone_location(
                    *pose
                )

            elif name == "husky1":
                self.robot_markers.update_husky_location(
                    *pose
                )

        if self.robot_subscriber.latest_poses:
            self.plotter.render()

    # =========================================================
    # WAYPOINT STORE CONNECTION
    # =========================================================

    def connect_waypoint_store(self, waypoint_store):
        """
        Connect the 3D map to the existing shared WaypointStore.

        The store remains the single source of truth.
        """

        if waypoint_store is self.waypoint_store:
            self.refresh_store_waypoints()
            return

        # Disconnect a previously connected store.
        if self.waypoint_store is not None:
            try:
                self.waypoint_store.changed.disconnect(
                    self.refresh_store_waypoints
                )
            except (RuntimeError, TypeError):
                pass

        self.waypoint_store = waypoint_store

        if self.waypoint_store is not None:
            self.waypoint_store.changed.connect(
                self.refresh_store_waypoints
            )

        # Populate the map immediately.
        self.refresh_store_waypoints()

    # =========================================================
    # REFRESH FROM WAYPOINT STORE
    # =========================================================

    def refresh_store_waypoints(self):
        """
        Synchronise map markers with the current waypoint queues.

        The order of each list determines the displayed
        waypoint numbers: D1, D2, H1, H2, etc.
        """

        if self.waypoint_store is None:
            self.update_waypoints([], [])
            return

        self.update_waypoints(
            drone_waypoints=(
                self.waypoint_store.get_drone_waypoints()
            ),
            husky_waypoints=(
                self.waypoint_store.get_husky_waypoints()
            ),
        )

    # =========================================================
    # WAYPOINT MARKER UPDATES
    # =========================================================

    def update_waypoints(
        self,
        drone_waypoints,
        husky_waypoints,
    ):
        """Update numbered SVG markers using ordered waypoint lists."""

        self.waypoint_markers.refresh(
            drone_waypoints=drone_waypoints,
            husky_waypoints=husky_waypoints,
        )

    # =========================================================
    # MAP LAYER VISIBILITY
    # =========================================================

    def set_layer_visible(self, layer_name, visible):
        """Control robot and waypoint SVG visibility."""

        # Robot location and home markers.
        self.robot_markers.set_layer_visible(
            layer_name,
            visible,
        )

        # Drone waypoint markers.
        if layer_name in (
            "drone_waypoint",
            "drone_waypoints",
        ):
            self.waypoint_markers.set_layer_visible(
                "drone",
                visible,
            )

        # Husky waypoint markers.
        elif layer_name in (
            "husky_waypoint",
            "husky_waypoints",
        ):
            self.waypoint_markers.set_layer_visible(
                "husky",
                visible,
            )

    # =========================================================
    # CAMERA CONTROLS
    # =========================================================

    def reset_view(self):
        """Reset the overhead camera and marker positions."""

        self.camera_controller.reset_view()
        self.marker_overlay.refresh()

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts."""

        if event.key() == Qt.Key_R:
            self.reset_view()
            event.accept()
            return

        super().keyPressEvent(event)

    # =========================================================
    # CLEANUP
    # =========================================================

    def closeEvent(self, event):
        """Clean up ROS 2, waypoint connections and VTK."""

        self.robot_timer.stop()

        # Disconnect from the shared waypoint store.
        if self.waypoint_store is not None:
            try:
                self.waypoint_store.changed.disconnect(
                    self.refresh_store_waypoints
                )
            except (RuntimeError, TypeError):
                pass

            self.waypoint_store = None

        # Remove numbered waypoint labels and SVG markers.
        self.marker_overlay.on_refresh = None
        self.waypoint_markers.clear()

        # Remove remaining robot markers and observers.
        self.marker_overlay.cleanup()

        self.robot_subscriber.destroy_node()

        if self.owns_ros_context and rclpy.ok():
            rclpy.shutdown()

        self.plotter.close()

        super().closeEvent(event)
