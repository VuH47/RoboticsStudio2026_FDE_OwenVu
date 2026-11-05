
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QWidget


class NumberedWaypoint(QWidget):
    """Displays a waypoint's queue number beside its SVG."""

    def __init__(self, number, robot, parent=None):
        super().__init__(parent)

        self.label = QLabel(self)

        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFixedSize(24, 20)

        self.setStyleSheet("""
            QLabel {
                background-color: #202b3a;
                color: white;
                border: 1px solid white;
                border-radius: 5px;
                font-family: Arial;
                font-size: 9px;
                font-weight: bold;
                padding: 0px 2px;
            }
        """)

        prefix = "D" if robot == "drone" else "H"
        self.label.setText(str(number))

        self.setFixedSize(38, 20)
        self.setAttribute(
            Qt.WA_TransparentForMouseEvents,
            True,
        )
        self.setAttribute(
            Qt.WA_TranslucentBackground,
            True,
        )
