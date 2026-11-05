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

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Mission Outcomes
        outcomes = QFrame()
        outcomes.setObjectName("infoPanel")

        outcomes_layout = QVBoxLayout(outcomes)
        outcomes_layout.setContentsMargins(10, 5, 10, 5)

        outcomes_title = QLabel("Mission Outcomes")
        outcomes_title.setObjectName("sectionTitle")
        outcomes_layout.addWidget(outcomes_title)

        self.outcomes_label = QLabel()
        outcomes_layout.addWidget(self.outcomes_label)

        # Mission Objectives
        objectives = QFrame()
        objectives.setObjectName("infoPanel")

        objectives_layout = QVBoxLayout(objectives)
        objectives_layout.setContentsMargins(10, 5, 10, 5)

        objectives_title = QLabel("Mission Objectives")
        objectives_title.setObjectName("sectionTitle")
        objectives_layout.addWidget(objectives_title)

        self.objectives_label = QLabel(
            "Hazards reached:  3 / 10\n"
            "Current task:      H-02 — Queued"
        )
        objectives_layout.addWidget(self.objectives_label)

        layout.addWidget(outcomes)
        layout.addWidget(objectives)

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

        self.outcomes_label.setText(
            f"Detected Hazards:        {detected}\n"
            f"Awaiting Husky:          {awaiting}\n"
            f"Resolved / Safe:         {resolved}\n"
            f"Confirmed Hazards:       {confirmed}"
        )

    def set_objectives(self, hazards_reached, total_hazards, current_task):
        self.objectives_label.setText(
            f"Hazards reached:  {hazards_reached} / {total_hazards}\n"
            f"Current task:      {current_task}"
        )