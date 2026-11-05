
from PySide6.QtCore import QObject, QEvent

from .svg_marker import SvgMarker


class MarkerOverlay(QObject):
    """
    Manages fixed-size SVG markers over a PyVista viewport.

    Each marker has:
        - An SVG widget
        - A world position (x, y, z)
        - A visibility state

    Markers follow the PyVista camera during pan/zoom.
    """

    def __init__(self, plotter, parent=None):
        super().__init__(parent)


        self.plotter = plotter
        self.viewport = plotter.interactor

        self.markers = {}

        self.on_refresh = None

        # Update marker positions when the viewport resizes.
        self.viewport.installEventFilter(self)

        # Update marker positions when the camera changes.
        self.camera = self.plotter.renderer.GetActiveCamera()

        self.camera_observer_id = self.camera.AddObserver(
            "ModifiedEvent",
            self._on_camera_modified,
        )

    # =========================================================
    # CREATE MARKER
    # =========================================================

    def add_marker(
        self,
        marker_id,
        svg_path,
        size=34,
        visible=True,
    ):
        """Create a marker with a unique identifier."""

        if marker_id in self.markers:
            raise ValueError(
                f"Marker already exists: {marker_id}"
            )

        widget = SvgMarker(
            svg_path=svg_path,
            size=size,
            parent=self.viewport,
        )

        self.markers[marker_id] = {
            "widget": widget,
            "position": None,
            "visible": bool(visible),
        }

        return widget

    # =========================================================
    # UPDATE POSITION
    # =========================================================

    def set_position(self, marker_id, x, y, z=0.0):
        """Update a marker's Gazebo world coordinates."""

        marker = self.markers[marker_id]

        marker["position"] = (
            float(x),
            float(y),
            float(z),
        )

        self.refresh_marker(marker_id)

    # =========================================================
    # UPDATE HEADING
    # =========================================================

    def set_heading(self, marker_id, degrees):
        """Rotate an SVG marker."""

        marker = self.markers[marker_id]

        marker["widget"].set_heading(degrees)

    # =========================================================
    # VISIBILITY
    # =========================================================

    def set_visible(self, marker_id, visible):
        """Show or hide a marker without deleting it."""

        marker = self.markers[marker_id]

        marker["visible"] = bool(visible)

        self.refresh_marker(marker_id)

    # =========================================================
    # REMOVE MARKER
    # =========================================================

    def remove_marker(self, marker_id):
        """Remove a marker and its Qt widget."""

        marker = self.markers.pop(marker_id, None)

        if marker is None:
            return

        marker["widget"].hide()
        marker["widget"].deleteLater()

    # =========================================================
    # WORLD TO SCREEN
    # =========================================================

    def world_to_screen(self, x, y, z):
        """Project a 3D world point into viewport pixels."""

        renderer = self.plotter.renderer

        renderer.SetWorldPoint(x, y, z, 1.0)
        renderer.WorldToDisplay()

        px, py, _ = renderer.GetDisplayPoint()

        # VTK uses a bottom-left origin.
        # Qt uses a top-left origin.
        height = self.viewport.height()

        return (
            px,
            height - py,
        )

    # =========================================================
    # REFRESH SINGLE MARKER
    # =========================================================

    def refresh_marker(self, marker_id):
        """Reposition and update one marker."""

        marker = self.markers[marker_id]
        widget = marker["widget"]
        position = marker["position"]

        if position is None or not marker["visible"]:
            widget.hide()
            return

        screen_x, screen_y = self.world_to_screen(
            *position
        )

        viewport_width = self.viewport.width()
        viewport_height = self.viewport.height()

        # Hide markers outside the visible viewport.
        if not (
            0 <= screen_x <= viewport_width
            and 0 <= screen_y <= viewport_height
        ):
            widget.hide()
            return

        widget.move(
            round(screen_x - widget.width() / 2),
            round(screen_y - widget.height() / 2),
        )

        widget.show()

        # Ensure robot location icons appear above home markers.
        if marker_id.endswith("_location"):
            widget.raise_()

    # =========================================================
    # REFRESH ALL MARKERS
    # =========================================================

    def refresh(self):
        """Reposition markers and maintain stacking order."""

        for marker_id in tuple(self.markers):
            self.refresh_marker(marker_id)

        drone = self.markers.get("drone_location")

        if drone is not None:
            drone["widget"].raise_()

        if self.on_refresh is not None:
            self.on_refresh()

    # =========================================================
    # CAMERA CALLBACK
    # =========================================================

    def _on_camera_modified(self, obj, event):
        self.refresh()

    # =========================================================
    # VIEWPORT RESIZE
    # =========================================================

    def eventFilter(self, watched, event):
        if (
            watched is self.viewport
            and event.type() == QEvent.Resize
        ):
            self.refresh()

        return super().eventFilter(watched, event)

    # =========================================================
    # CLEANUP
    # =========================================================

    def cleanup(self):
        """Disconnect VTK callbacks and remove Qt markers."""

        self.camera.RemoveObserver(
            self.camera_observer_id
        )

        self.viewport.removeEventFilter(self)

        for marker_id in tuple(self.markers):
            self.remove_marker(marker_id)
