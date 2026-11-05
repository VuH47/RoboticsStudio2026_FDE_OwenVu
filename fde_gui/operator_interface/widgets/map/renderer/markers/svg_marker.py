
from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QPainter
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import QWidget


class SvgMarker(QWidget):
    """
    Fixed-size SVG map marker.

    Handles:
    - SVG rendering
    - Fixed pixel dimensions
    - Rotation
    - Transparent background

    World-coordinate positioning is handled separately
    by MarkerOverlay.
    """

    def __init__(self, svg_path, size=34, parent=None):
        super().__init__(parent)

        self.svg_renderer = QSvgRenderer(str(svg_path))

        if not self.svg_renderer.isValid():
            raise ValueError(
                f"Unable to load SVG marker: {svg_path}"
            )

        self.setFixedSize(size, size)

        self.setAttribute(
            Qt.WA_TransparentForMouseEvents,
            True,
        )

        self.setAttribute(
            Qt.WA_TranslucentBackground,
            True,
        )

        self.heading = 0.0
        self.hide()

    def set_heading(self, degrees):
        """Rotate the marker around its centre."""

        self.heading = float(degrees)
        self.update()

    def paintEvent(self, event):
        """Render the SVG at a fixed pixel size."""

        painter = QPainter(self)

        painter.setRenderHint(
            QPainter.Antialiasing,
            True,
        )

        painter.setRenderHint(
            QPainter.SmoothPixmapTransform,
            True,
        )

        painter.translate(
            self.width() / 2,
            self.height() / 2,
        )

        painter.rotate(self.heading)

        svg_size = self.svg_renderer.defaultSize()

        if svg_size.isEmpty():
            painter.end()
            return

        scale = min(
            self.width() / svg_size.width(),
            self.height() / svg_size.height(),
        )

        width = svg_size.width() * scale
        height = svg_size.height() * scale

        self.svg_renderer.render(
            painter,
            QRectF(
                -width / 2,
                -height / 2,
                width,
                height,
            ),
        )

        painter.end()
