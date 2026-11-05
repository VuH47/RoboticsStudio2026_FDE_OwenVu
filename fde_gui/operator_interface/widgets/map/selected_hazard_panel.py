from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)


class SelectedHazardPanel(QFrame):
    """
    Floating panel showing the currently selected hazard.
    """

    view_full_record_requested = Signal()
    centre_on_map_requested = Signal()

    def __init__(
        self,
        status_colours,
        parent=None,
    ):
        super().__init__(parent)

        self.status_colours = status_colours

        self.setObjectName("quickHazardPanel")

        # =========================================================
        # MAIN LAYOUT
        # =========================================================

        layout = QVBoxLayout(self)

        layout.setContentsMargins(
            10,
            8,
            10,
            8,
        )

        layout.setSpacing(5)

        # =========================================================
        # HEADER
        # =========================================================

        header = QHBoxLayout()

        header.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        self.title = QLabel("Selected Hazard")
        self.title.setObjectName("quickTitle")

        header.addWidget(self.title)
        header.addStretch()

        self.close_button = QPushButton("×")
        self.close_button.setObjectName("smallCloseButton")

        self.close_button.setFixedSize(
            22,
            22,
        )

        self.close_button.clicked.connect(
            self.hide
        )

        header.addWidget(self.close_button)

        layout.addLayout(header)

        # =========================================================
        # SEPARATOR
        # =========================================================

        layout.addWidget(
            self._create_separator()
        )

        # =========================================================
        # HAZARD ID
        # =========================================================

        self.hazard_id = QLabel("H-00")
        self.hazard_id.setObjectName("quickHazardId")

        layout.addWidget(self.hazard_id)

        layout.addWidget(
            self._create_separator()
        )

        # =========================================================
        # STATUS
        # =========================================================

        self.status = QLabel("● Status")
        self.status.setObjectName("quickHazardStatus")

        layout.addWidget(self.status)

        layout.addWidget(
            self._create_separator()
        )

        # =========================================================
        # HAZARD INFORMATION
        # =========================================================

        self.info = QLabel(
            "Type: —\n"
            "Coordinates: —"
        )

        self.info.setObjectName("quickHazardInfo")

        layout.addWidget(self.info)

        layout.addWidget(
            self._create_separator()
        )

        # =========================================================
        # EVIDENCE TITLE
        # =========================================================

        self.evidence_label = QLabel(
            "Latest Evidence:"
        )

        self.evidence_label.setObjectName(
            "quickEvidenceTitle"
        )

        layout.addWidget(
            self.evidence_label
        )

        # =========================================================
        # EVIDENCE IMAGE / PLACEHOLDER
        # =========================================================

        self.evidence = QLabel(
            "No image loaded"
        )

        self.evidence.setObjectName(
            "evidencePlaceholder"
        )

        self.evidence.setAlignment(
            Qt.AlignCenter
        )

        self.evidence.setMinimumHeight(
            90
        )

        layout.addWidget(
            self.evidence
        )

        # =========================================================
        # ACTIONS
        # =========================================================

        self.full_record_button = QPushButton(
            "View Full Record"
        )

        self.full_record_button.setObjectName(
            "hazardOverlayAction"
        )

        self.full_record_button.clicked.connect(
            self.view_full_record_requested.emit
        )

        layout.addWidget(
            self.full_record_button
        )

        self.centre_button = QPushButton(
            "Centre on Map"
        )

        self.centre_button.setObjectName(
            "hazardOverlayAction"
        )

        self.centre_button.clicked.connect(
            self.centre_on_map_requested.emit
        )

        layout.addWidget(
            self.centre_button
        )

        # Hidden until a hazard is selected.
        self.hide()

    # =============================================================
    # CREATE SEPARATOR
    # =============================================================

    def _create_separator(self):
        line = QFrame()

        line.setObjectName(
            "quickHazardSeparator"
        )

        line.setFrameShape(
            QFrame.HLine
        )

        line.setFixedHeight(
            1
        )

        return line

    # =============================================================
    # DISPLAY HAZARD
    # =============================================================

    def set_hazard(
        self,
        hazard_id,
        hazard,
    ):
        # =========================================================
        # HAZARD ID
        # =========================================================

        self.hazard_id.setText(
            hazard_id
        )

        # =========================================================
        # STATUS
        # =========================================================

        status = hazard["status"]

        colour = self.status_colours.get(
            status,
            "#ffffff",
        )

        self.status.setText(
            f"<span style='color:{colour}; "
            f"font-size:18px;'>●</span> "
            f"{status}"
        )

        self.status.setTextFormat(
            Qt.RichText
        )

        # =========================================================
        # HAZARD INFORMATION
        # =========================================================

        x, y, z = hazard["coords"]

        self.info.setText(
            f"Type: {hazard['type']}\n"
            f"Coordinates: X: {x} Y: {y} Z: {z}"
        )

        # =========================================================
        # EVIDENCE TYPE
        # =========================================================

        if status == "Resolved / Safe":
            evidence_text = (
                "Post-Clearance Photo"
            )

        elif hazard["husky_assignment"] in (
            "En Route",
            "Inspecting",
            "Clearing",
            "Completed",
        ):
            evidence_text = (
                "Ground Inspection Photo"
            )

        else:
            evidence_text = (
                "Detection Snapshot"
            )

        self.evidence.setText(
            evidence_text
        )

        # =========================================================
        # SHOW PANEL
        # =========================================================

        self.show()
        self.raise_()