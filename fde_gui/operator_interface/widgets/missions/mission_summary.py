from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)


class MissionSummary(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        # =========================================================
        # MAIN LAYOUT
        # =========================================================

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # =========================================================
        # MISSION OUTCOMES PANEL
        # =========================================================

        outcomes = QFrame()
        outcomes.setObjectName("infoPanel")

        outcomes_layout = QVBoxLayout(outcomes)
        outcomes_layout.setContentsMargins(10, 5, 10, 5)
        outcomes_layout.setSpacing(5)

        # Title
        outcomes_title = QLabel("Mission Outcomes")
        outcomes_title.setObjectName("sectionTitle")
        outcomes_layout.addWidget(outcomes_title)

        # =========================================================
        # HORIZONTAL STATISTICS
        # =========================================================

        stats_layout = QHBoxLayout()
        stats_layout.setSpacing(20)

        self.detected_label = QLabel()
        self.awaiting_label = QLabel()
        self.resolved_label = QLabel()
        self.confirmed_label = QLabel()

        labels = [
            self.detected_label,
            self.awaiting_label,
            self.resolved_label,
            self.confirmed_label,
        ]

        for label in labels:
            label.setAlignment(Qt.AlignCenter)
            stats_layout.addWidget(label, 1)

        outcomes_layout.addLayout(stats_layout)

        # Full-width panel
        layout.addWidget(outcomes)

        # Initialise counters
        self.update_outcomes({})

    # =============================================================
    # UPDATE MISSION OUTCOMES
    # =============================================================

    def update_outcomes(self, hazards):

        detected = len(hazards)

        awaiting = sum(
            1 for hazard in hazards.values()
            if hazard["status"] == "Husky Required"
        )

        resolved = sum(
            1 for hazard in hazards.values()
            if hazard["status"] == "Resolved / Safe"
        )

        confirmed = sum(
            1 for hazard in hazards.values()
            if hazard["status"] == "Confirmed Hazard"
        )

        self.detected_label.setText(
            f"Detected Hazards: {detected}"
        )

        self.awaiting_label.setText(
            f"Awaiting Husky: {awaiting}"
        )

        self.resolved_label.setText(
            f"Resolved / Safe: {resolved}"
        )

        self.confirmed_label.setText(
            f"Confirmed Hazards: {confirmed}"
        )

    # =============================================================
    # LEGACY COMPATIBILITY
    # =============================================================

    def set_objectives(
        self,
        hazards_reached,
        total_hazards,
        current_task,
    ):
        pass