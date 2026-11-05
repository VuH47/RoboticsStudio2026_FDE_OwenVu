from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QCheckBox


class ToggleSwitch(QCheckBox):
    """Simple two-state ON/OFF toggle switch."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setCursor(Qt.PointingHandCursor)
        self.setFixedSize(34, 18)
        self.setChecked(True)

        # Remove the normal QCheckBox indicator.
        self.setStyleSheet("""
            QCheckBox {
                spacing: 0px;
            }

            QCheckBox::indicator {
                width: 0px;
                height: 0px;
            }
        """)

    def hitButton(self, pos):
        """
        Make the entire custom switch clickable,
        including the white knob.
        """
        return self.rect().contains(pos)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        track_rect = QRectF(0, 1, 34, 16)

        if self.isChecked():
            track_colour = QColor("#1976D2")
            knob_x = 18
        else:
            track_colour = QColor("#777777")
            knob_x = 2

        # Track
        painter.setPen(Qt.NoPen)
        painter.setBrush(track_colour)
        painter.drawRoundedRect(track_rect, 8, 8)

        # White knob
        painter.setBrush(QColor("#FFFFFF"))
        painter.drawEllipse(QRectF(knob_x, 2, 14, 14))

        painter.end()