from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QPixmap
from pathlib import Path
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class EvidencePhoto(QLabel):
    """Scale a saved detection image to fit, preserving its aspect ratio."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._original = None

    def set_snapshot(self, path):
        pixmap = QPixmap(str(path))
        if pixmap.isNull():
            self._original = None
            self.clear()
            self.setText("Detection Snapshot\n\n[ Image unavailable ]")
            return
        self._original = pixmap
        self._render_snapshot()

    def show_placeholder(self):
        self._original = None
        self.clear()
        self.setText("Detection Snapshot\n\n[ No captured image ]")

    def _render_snapshot(self):
        if self._original is not None and self.width() > 4 and self.height() > 4:
            self.setPixmap(self._original.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            ))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_snapshot()


class HazardDetails(QFrame):

    def __init__(
        self,
        status_colours,
        confirm_safe_callback,
        confirm_hazard_callback,
        primary_action_callback,
        secondary_action_callback,
        parent=None,
    ):
        super().__init__(parent)

        self.status_colours = status_colours

        self.setObjectName("hazardDetailsPanel")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)

        # =========================================================
        # HAZARD TITLE
        # =========================================================

        self.hazard_title = QLabel("Select a hazard")
        self.hazard_title.setFont(
            QFont("Arial", 19, QFont.Bold)
        )

        layout.addWidget(self.hazard_title)

        # =========================================================
        # HAZARD STATUS
        # =========================================================

        self.hazard_status = QLabel("Status: —")
        self.hazard_status.setObjectName(
            "hazardStatusBadge"
        )

        layout.addWidget(
            self.hazard_status,
            alignment=Qt.AlignRight,
        )

        # =========================================================
        # HAZARD INFORMATION
        # =========================================================

        self.hazard_info = QLabel(
            "Detected by: —\n"
            "Time Detected: —\n"
            "Detection Confidence: —\n"
            "Husky Solvable: —\n"
            "Husky Assignment: —\n"
            "Assigned to Husky: —\n"
            "Resolved by: —\n"
            "Time Resolved: —\n"
            "Coordinates: —"
        )

        layout.addWidget(self.hazard_info)

        # =========================================================
        # EVIDENCE TABS
        # =========================================================

        self.evidence_tabs = QTabWidget()
        self.evidence_tabs.setObjectName(
            "evidenceTabs"
        )

        # ---------------------------------------------------------
        # DRONE SHOTS
        # ---------------------------------------------------------

        drone_shots = QWidget()
        drone_layout = QVBoxLayout(
            drone_shots
        )

        self.drone_photo = EvidencePhoto()
        self.drone_photo.show_placeholder()

        self.drone_photo.setAlignment(
            Qt.AlignCenter
        )

        self.drone_photo.setObjectName(
            "photoPlaceholder"
        )

        drone_layout.addWidget(
            self.drone_photo
        )

        # ---------------------------------------------------------
        # HUSKY SHOTS
        # ---------------------------------------------------------

        husky_shots = QWidget()
        husky_layout = QHBoxLayout(
            husky_shots
        )

        self.ground_photo = QLabel(
            "Ground Inspection\n\n"
            "[ Husky photo ]"
        )

        self.ground_photo.setAlignment(
            Qt.AlignCenter
        )

        self.ground_photo.setObjectName(
            "photoPlaceholder"
        )

        self.post_photo = QLabel(
            "Post Clearance\n\n"
            "[ Only shown when applicable ]"
        )

        self.post_photo.setAlignment(
            Qt.AlignCenter
        )

        self.post_photo.setObjectName(
            "photoPlaceholder"
        )

        husky_layout.addWidget(
            self.ground_photo
        )

        husky_layout.addWidget(
            self.post_photo
        )

        self.evidence_tabs.addTab(
            drone_shots,
            "Drone Shots",
        )

        self.evidence_tabs.addTab(
            husky_shots,
            "Husky Shots",
        )

        layout.addWidget(
            self.evidence_tabs,
            1,
        )

        # =========================================================
        # NOTES
        # =========================================================

        layout.addWidget(
            QLabel("Notes")
        )

        self.notes = QTextEdit()

        self.notes.setPlaceholderText(
            "Operator notes..."
        )

        self.notes.setMaximumHeight(
            110
        )

        layout.addWidget(
            self.notes
        )

        # =========================================================
        # CONFIRM SAFE
        # =========================================================

        self.confirm_safe_action = QPushButton(
            "Confirm Safe"
        )

        self.confirm_safe_action.setObjectName(
            "confirmSafeAction"
        )

        self.confirm_safe_action.clicked.connect(
            confirm_safe_callback
        )

        layout.addWidget(
            self.confirm_safe_action
        )

        # =========================================================
        # CONFIRM HAZARD
        # =========================================================

        self.confirm_hazard_action = QPushButton(
            "Confirm Hazard"
        )

        self.confirm_hazard_action.setObjectName(
            "confirmHazardAction"
        )

        self.confirm_hazard_action.clicked.connect(
            confirm_hazard_callback
        )

        layout.addWidget(
            self.confirm_hazard_action
        )

        # =========================================================
        # PRIMARY ACTION
        # =========================================================

        self.primary_action = QPushButton(
            "Select a hazard"
        )

        self.primary_action.setObjectName(
            "primaryHazardAction"
        )

        self.primary_action.clicked.connect(
            primary_action_callback
        )

        layout.addWidget(
            self.primary_action
        )

        # =========================================================
        # SECONDARY ACTION
        # =========================================================

        self.secondary_action = QPushButton(
            "Dismiss Hazard"
        )

        self.secondary_action.setObjectName(
            "secondaryHazardAction"
        )

        self.secondary_action.clicked.connect(
            secondary_action_callback
        )

        layout.addWidget(
            self.secondary_action
        )

        # Hide actions until a hazard is selected.
        self.confirm_safe_action.hide()
        self.confirm_hazard_action.hide()
        self.primary_action.hide()
        self.secondary_action.hide()

    # =============================================================
    # SHOW HAZARD
    # =============================================================

    def show_hazard(
        self,
        hazard_id,
        hazard,
    ):

        # =========================================================
        # TITLE
        # =========================================================

        self.hazard_title.setText(
            f"{hazard_id}    "
            f"{hazard['type']}"
        )

        # =========================================================
        # STATUS
        # =========================================================

        status = hazard.get(
            "status",
            "Detected",
        )

        colour = self.status_colours.get(
            status,
            "#ffffff",
        )

        self.hazard_status.setText(
            f"<span "
            f"style='color:{colour}; "
            f"font-size:18px;'>"
            f"●"
            f"</span> "
            f"{status}"
        )

        self.hazard_status.setTextFormat(
            Qt.RichText
        )

        # =========================================================
        # CONFIDENCE
        # =========================================================

        detection_confidence = hazard.get(
            "detection_confidence"
        )

        husky_solvable = hazard.get(
            "husky_solvable"
        )

        if detection_confidence is None:
            detection_text = "—"
        else:
            detection_text = (
                f"{detection_confidence:.0f}%"
            )

        if husky_solvable is None:
            husky_text = "—"
        else:
            husky_text = (
                f"{husky_solvable:.0f}%"
            )

        # =========================================================
        # INFORMATION
        # =========================================================

        x, y, z = hazard.get(
            "coords",
            ("—", "—", "—"),
        )

        self.hazard_info.setText(
            f"Detected by:              "
            f"{hazard.get('detected_by', '—')}\n"

            f"Time Detected:            "
            f"{hazard.get('time_detected', '—')}\n"

            f"Detection Confidence:     "
            f"{detection_text}\n"

            f"Husky Solvable:           "
            f"{husky_text}\n"

            f"Husky Assignment:         "
            f"{hazard.get('husky_assignment', '—')}\n"

            f"Assigned to Husky:        "
            f"{hazard.get('assigned_time', '—')}\n"

            f"Resolved by:              "
            f"{hazard.get('resolved_by', '—')}\n"

            f"Time Resolved:            "
            f"{hazard.get('resolved_time', '—')}\n"

            f"Coordinates:              "
            f"X {x}  Y {y}  Z {z}"
        )

        # =========================================================
        # DETECTION SNAPSHOT (stored by the OperatorGUI)
        # =========================================================

        snapshot = (
            Path.home() / ".local" / "share" / "fde_gui" /
            "hazard_snapshots" / f"{hazard_id}.png"
        )
        if snapshot.is_file():
            self.drone_photo.set_snapshot(snapshot)
        else:
            self.drone_photo.show_placeholder()

        # =========================================================
        # NOTES
        # =========================================================

        self.notes.setPlainText(
            hazard.get(
                "notes",
                "",
            )
        )

        # =========================================================
        # RESET BUTTONS
        # =========================================================

        self.confirm_safe_action.hide()
        self.confirm_hazard_action.hide()
        self.primary_action.hide()
        self.secondary_action.show()

        self.secondary_action.setText(
            "Dismiss Hazard"
        )

        # Remove previous primary action type.
        self.primary_action.setProperty(
            "actionType",
            "",
        )

        # =========================================================
        # DETECTED
        # =========================================================

        if status == "Inconclusive":

            self.confirm_safe_action.show()
            self.confirm_hazard_action.show()
            self.primary_action.show()

            self.confirm_safe_action.setText(
                "Confirm Safe"
            )

            self.confirm_hazard_action.setText(
                "Confirm Hazard"
            )

            self.primary_action.setText(
                "Send Husky"
            )

            self.primary_action.setProperty(
                "actionType",
                "husky",
            )

        # =========================================================
        # HUSKY REQUIRED
        # =========================================================

        elif status == "Husky Required":

            self.primary_action.show()

            self.primary_action.setText(
                "Cancel Assignment"
            )

            self.primary_action.setProperty(
                "actionType",
                "cancel",
            )

        # =========================================================
        # IN PROGRESS
        # =========================================================

        elif status == "In Progress":

            self.primary_action.show()

            self.primary_action.setText(
                "View Live Husky"
            )

            self.primary_action.setProperty(
                "actionType",
                "live",
            )

        # =========================================================
        # RESOLVED / CONFIRMED
        # =========================================================

        elif status in (
            "Resolved / Safe",
            "Confirmed Hazard",
        ):

            self.primary_action.show()

            self.primary_action.setText(
                "Reopen Hazard"
            )

            self.primary_action.setProperty(
                "actionType",
                "reopen",
            )

        # =========================================================
        # FALLBACK
        # =========================================================

        else:

            self.primary_action.show()

            self.primary_action.setText(
                "Reopen Hazard"
            )

            self.primary_action.setProperty(
                "actionType",
                "reopen",
            )

        # =========================================================
        # REFRESH DYNAMIC QSS
        # =========================================================

        self._refresh_button_style(
            self.primary_action
        )

    # =============================================================
    # REFRESH BUTTON STYLE
    # =============================================================

    def _refresh_button_style(
        self,
        button,
    ):

        style = button.style()

        style.unpolish(
            button
        )

        style.polish(
            button
        )

        button.update()