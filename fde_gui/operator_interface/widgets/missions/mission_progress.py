from PySide6.QtCore import Qt
from PySide6.QtGui import (
    QColor,
    QPainter,
    QPen,
)
from PySide6.QtWidgets import (
    QLabel,
    QVBoxLayout,
    QWidget,
)


# =================================================================
# PROGRESS TRACK
# =================================================================

class ProgressTrack(QWidget):
    """
    Draws the mission progress circles and connector lines.

    Drawing the track directly ensures that connector lines meet
    the circle edges exactly.
    """

    CIRCLE_SIZE = 36
    CONNECTOR_HEIGHT = 3

    # Pulls the first and last stages away from the panel edges.
    SIDE_MARGIN = 50

    def __init__(
        self,
        stage_count,
        parent=None,
    ):
        super().__init__(parent)

        self.stage_count = stage_count

        self.completed_index = -1
        self.skipped_indices = set()

        self.setMinimumHeight(
            self.CIRCLE_SIZE + 4
        )

    # =============================================================
    # SET PROGRESS
    # =============================================================

    def set_progress(
        self,
        completed_index,
        skipped_indices,
    ):
        self.completed_index = completed_index

        self.skipped_indices = set(
            skipped_indices
        )

        self.update()

    # =============================================================
    # GET STAGE CENTRES
    # =============================================================

    def _stage_centres(self):
        """
        Calculate the horizontal centre of every stage circle.
        """

        width = self.width()

        radius = (
            self.CIRCLE_SIZE / 2
        )

        left_x = (
            radius
            + self.SIDE_MARGIN
        )

        right_x = (
            width
            - radius
            - self.SIDE_MARGIN
        )

        if self.stage_count <= 1:

            return [
                width / 2
            ]

        spacing = (
            right_x - left_x
        ) / (
            self.stage_count - 1
        )

        return [
            left_x
            + index * spacing

            for index in range(
                self.stage_count
            )
        ]

    # =============================================================
    # PAINT
    # =============================================================

    def paintEvent(
        self,
        event,
    ):
        painter = QPainter(self)

        painter.setRenderHint(
            QPainter.Antialiasing
        )

        height = self.height()

        radius = (
            self.CIRCLE_SIZE / 2
        )

        centre_y = (
            height / 2
        )

        centres = (
            self._stage_centres()
        )

        # =========================================================
        # CONNECTORS
        # =========================================================

        for index in range(
            self.stage_count - 1
        ):

            # Start exactly at the right edge
            # of the current circle.
            x1 = (
                centres[index]
                + radius
            )

            # Finish exactly at the left edge
            # of the next circle.
            x2 = (
                centres[index + 1]
                - radius
            )

            # Connector becomes complete once
            # the stage to its right is complete.
            if index < self.completed_index:

                colour = QColor(
                    "#49ad2f"
                )

            else:

                colour = QColor(
                    "#d8d8d8"
                )

            pen = QPen(
                colour,
                self.CONNECTOR_HEIGHT,
            )

            pen.setCapStyle(
                Qt.FlatCap
            )

            painter.setPen(
                pen
            )

            painter.drawLine(
                int(x1),
                int(centre_y),
                int(x2),
                int(centre_y),
            )

        # =========================================================
        # CIRCLES
        # =========================================================

        for index, centre_x in enumerate(
            centres
        ):

            # -----------------------------------------------------
            # SKIPPED
            # -----------------------------------------------------

            if index in self.skipped_indices:

                fill_colour = QColor(
                    "#777777"
                )

                symbol = "—"

            # -----------------------------------------------------
            # COMPLETE
            # -----------------------------------------------------

            elif index <= self.completed_index:

                fill_colour = QColor(
                    "#49ad2f"
                )

                symbol = "✓"

            # -----------------------------------------------------
            # PENDING
            # -----------------------------------------------------

            else:

                fill_colour = QColor(
                    "#273b55"
                )

                symbol = ""

            # -----------------------------------------------------
            # DRAW CIRCLE
            # -----------------------------------------------------

            painter.setBrush(
                fill_colour
            )

            painter.setPen(
                QPen(
                    QColor("#d8d8d8"),
                    2,
                )
            )

            painter.drawEllipse(
                int(
                    centre_x - radius
                ),
                int(
                    centre_y - radius
                ),
                self.CIRCLE_SIZE,
                self.CIRCLE_SIZE,
            )

            # -----------------------------------------------------
            # DRAW SYMBOL
            # -----------------------------------------------------

            if symbol:

                painter.setPen(
                    QColor("#ffffff")
                )

                font = painter.font()

                font.setBold(True)
                font.setPointSize(12)

                painter.setFont(
                    font
                )

                painter.drawText(
                    int(
                        centre_x - radius
                    ),
                    int(
                        centre_y - radius
                    ),
                    self.CIRCLE_SIZE,
                    self.CIRCLE_SIZE,
                    Qt.AlignCenter,
                    symbol,
                )


# =================================================================
# MISSION PROGRESS
# =================================================================

class MissionProgress(QWidget):
    """
    Mission progress indicator.

    Flow:
        Trail Selected
        -> Drone Scanning
        -> Manual Inspection (optional)
        -> Husky Intervention
        -> Mission Complete

    Each ROS event represents a completed mission stage.
    """

    STAGES = [
        (
            "trail_selected",
            "Trail\nSelected",
        ),
        (
            "drone_scanning",
            "Drone\nScanning",
        ),
        (
            "manual_inspection",
            "Manual\nInspection",
        ),
        (
            "husky_intervention",
            "Husky\nIntervention",
        ),
        (
            "mission_complete",
            "Mission\nComplete",
        ),
    ]

    LABEL_HEIGHT = 32
    LABEL_WIDTH = 100

    def __init__(
        self,
        parent=None,
    ):
        super().__init__(parent)

        self.setObjectName(
            "missionProgress"
        )

        self.current_stage = None

        self.skipped_stages = set()

        self.stage_labels = {}
        self.label_widgets = []

        self._build_ui()

        self.reset()

    # =============================================================
    # BUILD UI
    # =============================================================

    def _build_ui(self):

        layout = QVBoxLayout(self)

        layout.setContentsMargins(
            8,
            0,
            8,
            0,
        )

        layout.setSpacing(2)

        # =========================================================
        # LABEL CONTAINER
        # =========================================================

        self.label_container = QWidget()

        self.label_container.setObjectName(
            "missionProgressLabels"
        )

        self.label_container.setFixedHeight(
            self.LABEL_HEIGHT
        )

        for stage_id, stage_text in self.STAGES:

            label = QLabel(
                stage_text,
                self.label_container,
            )

            label.setObjectName(
                "missionStageLabel"
            )

            label.setAlignment(
                Qt.AlignCenter
            )

            label.setFixedSize(
                self.LABEL_WIDTH,
                self.LABEL_HEIGHT,
            )

            self.stage_labels[
                stage_id
            ] = label

            self.label_widgets.append(
                label
            )

        layout.addWidget(
            self.label_container
        )

        # =========================================================
        # PROGRESS TRACK
        # =========================================================

        self.track = ProgressTrack(
            len(self.STAGES)
        )

        layout.addWidget(
            self.track
        )

    # =============================================================
    # RESIZE
    # =============================================================

    def resizeEvent(
        self,
        event,
    ):
        super().resizeEvent(
            event
        )

        self._position_labels()

    # =============================================================
    # POSITION LABELS
    # =============================================================

    def _position_labels(self):
        """
        Position each label directly above its corresponding
        progress circle.
        """

        if not self.label_widgets:
            return

        width = (
            self.label_container.width()
        )

        stage_count = len(
            self.label_widgets
        )

        radius = (
            ProgressTrack.CIRCLE_SIZE
            / 2
        )

        # IMPORTANT:
        # These calculations match ProgressTrack exactly.
        left_x = (
            radius
            + ProgressTrack.SIDE_MARGIN
        )

        right_x = (
            width
            - radius
            - ProgressTrack.SIDE_MARGIN
        )

        if stage_count <= 1:

            centres = [
                width / 2
            ]

        else:

            spacing = (
                right_x - left_x
            ) / (
                stage_count - 1
            )

            centres = [
                left_x
                + index * spacing

                for index in range(
                    stage_count
                )
            ]

        # =========================================================
        # POSITION EACH LABEL
        # =========================================================

        for index, label in enumerate(
            self.label_widgets
        ):

            centre_x = (
                centres[index]
            )

            x = int(
                centre_x
                - self.LABEL_WIDTH / 2
            )

            label.setGeometry(
                x,
                0,
                self.LABEL_WIDTH,
                self.LABEL_HEIGHT,
            )

    # =============================================================
    # SET COMPLETED STAGE
    # =============================================================

    def set_stage(
        self,
        stage,
    ):
        """
        Mark a stage as completed.

        For example:

            TRAIL_SELECTED
                -> Trail Selected receives a tick.

            DRONE_SCANNING
                -> Trail Selected and Drone Scanning receive ticks.
        """

        stage_ids = (
            self._stage_ids()
        )

        if stage not in stage_ids:

            print(
                f"[MissionProgress] "
                f"Unknown stage: {stage}"
            )

            return

        self.current_stage = stage

        self._update_track()

    # =============================================================
    # SKIP STAGE
    # =============================================================

    def skip_stage(
        self,
        stage,
    ):
        """
        Mark a stage as skipped.

        Manual Inspection can be skipped when the drone
        detection confidence is sufficient.
        """

        if stage not in self._stage_ids():

            print(
                f"[MissionProgress] "
                f"Cannot skip unknown stage: "
                f"{stage}"
            )

            return

        self.skipped_stages.add(
            stage
        )

        self._update_track()

    # =============================================================
    # CLEAR SKIPPED STAGE
    # =============================================================

    def clear_skipped_stage(
        self,
        stage,
    ):

        self.skipped_stages.discard(
            stage
        )

        self._update_track()

    # =============================================================
    # RESET
    # =============================================================

    def reset(self):
        """
        Return all mission stages to pending.
        """

        self.current_stage = None

        self.skipped_stages.clear()

        self._update_track()

    # =============================================================
    # UPDATE TRACK
    # =============================================================

    def _update_track(self):

        stage_ids = (
            self._stage_ids()
        )

        # ---------------------------------------------------------
        # COMPLETED STAGE
        # ---------------------------------------------------------

        if self.current_stage is None:

            completed_index = -1

        else:

            completed_index = (
                stage_ids.index(
                    self.current_stage
                )
            )

        # ---------------------------------------------------------
        # SKIPPED STAGES
        # ---------------------------------------------------------

        skipped_indices = {
            stage_ids.index(stage)

            for stage
            in self.skipped_stages

            if stage in stage_ids
        }

        self.track.set_progress(
            completed_index,
            skipped_indices,
        )

    # =============================================================
    # PROCESS ROS EVENT
    # =============================================================

    def process_event(
        self,
        event,
    ):

        event = (
            event
            .strip()
            .upper()
        )

        events = {
            "TRAIL_SELECTED":
                "trail_selected",

            "DRONE_SCANNING":
                "drone_scanning",

            "MANUAL_INSPECTION":
                "manual_inspection",

            "HUSKY_INTERVENTION":
                "husky_intervention",

            "MISSION_COMPLETE":
                "mission_complete",
        }

        # ---------------------------------------------------------
        # MANUAL INSPECTION SKIPPED
        # ---------------------------------------------------------

        if (
            event
            == "MANUAL_INSPECTION_SKIPPED"
        ):

            self.skip_stage(
                "manual_inspection"
            )

            return

        # ---------------------------------------------------------
        # RESET
        # ---------------------------------------------------------

        if event == "RESET":

            self.reset()

            return

        # ---------------------------------------------------------
        # NORMAL STAGE EVENT
        # ---------------------------------------------------------

        stage = events.get(
            event
        )

        if stage is None:

            print(
                f"[MissionProgress] "
                f"Unknown event: {event}"
            )

            return

        self.set_stage(
            stage
        )

    # =============================================================
    # HELPERS
    # =============================================================

    def _stage_ids(self):

        return [
            stage_id

            for stage_id, _
            in self.STAGES
        ]