
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QPainter
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import QWidget

from fde_gui.operator_interface.paths import ASSETS_DIR


class SvgMapMarker(QWidget):
    """Fixed-pixel SVG marker positioned over the PyVista viewport."""

    def __init__(self, svg_path, size, parent):
        super().__init__(parent)

        self.renderer = QSvgRenderer(str(svg_path))

        if not self.renderer.isValid():
            raise FileNotFoundError(f"Invalid SVG: {svg_path}")

        self.setFixedSize(size, size)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WA_TranslucentBackground)

        self.heading = 0.0
        self.hide()

    def set_heading(self, degrees):
        self.heading = degrees
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(self.heading)

        svg_size = self.renderer.defaultSize()

        scale = min(
            self.width() / svg_size.width(),
            self.height() / svg_size.height(),
        )

        width = svg_size.width() * scale
        height = svg_size.height() * scale

        self.renderer.render(
            painter,
            QRectF(-width / 2, -height / 2, width, height),
        )


class DroneMapIcons:
    """Drone location and home overlays."""

    def __init__(self, viewport, plotter):
        self.viewport = viewport
        self.plotter = plotter

        self.location_icon = SvgMapMarker(
            ASSETS_DIR / "drone_icons" / "drone_location.svg",
            34,
            viewport,
        )

        self.home_icon = SvgMapMarker(
            ASSETS_DIR / "drone_icons" / "drone_home.svg",
            32,
            viewport,
        )

        self.location_position = None
        self.home_position = None

        self.location_enabled = True
        self.home_enabled = True

    def set_location(self, x, y, z, heading=0.0):
        self.location_position = (x, y, z)
        self.location_icon.set_heading(heading)
        self.refresh()

    def set_home(self, x, y, z=0.0):
        self.home_position = (x, y, z)
        self.refresh()

    def set_layer_visible(self, layer, visible):
        if layer == "drone_location":
            self.location_enabled = visible
        elif layer == "drone_home":
            self.home_enabled = visible

        self.refresh()

    def _position_marker(self, marker, position, enabled):
        if position is None or not enabled:
            marker.hide()
            return

        # Convert world coordinates into viewport pixel coordinates.
        display = self.plotter.renderer.SetWorldPoint(
            position[0],
            position[1],
            position[2],
            1.0,
        )
        self.plotter.renderer.WorldToDisplay()
        px, py, _ = self.plotter.renderer.GetDisplayPoint()

        width = self.viewport.width()
        height = self.viewport.height()

        # VTK uses bottom-left origin; Qt uses top-left origin.
        qt_x = px
        qt_y = height - py

        if not (0 <= qt_x <= width and 0 <= qt_y <= height):
            marker.hide()
            return

        marker.move(
            round(qt_x - marker.width() / 2),
            round(qt_y - marker.height() / 2),
        )

        marker.show()
        marker.raise_()

    def refresh(self):
        self._position_marker(
            self.location_icon,
            self.location_position,
            self.location_enabled,
        )

        self._position_marker(
            self.home_icon,
            self.home_position,
            self.home_enabled,
        )
