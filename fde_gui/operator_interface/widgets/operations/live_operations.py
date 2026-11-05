from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QPixmap
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class LiveOperations(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(6)

        # =========================================================
        # ROBOT SELECTOR
        # =========================================================

        selector = QHBoxLayout()

        self.drone_button = QPushButton("Drone")
        self.husky_button = QPushButton("Husky")
        self.both_button = QPushButton("Both")

        for button in (
            self.drone_button,
            self.husky_button,
            self.both_button,
        ):
            button.setObjectName("selectorButton")
            button.setCheckable(True)
            selector.addWidget(button)

        self.drone_button.setChecked(True)

        self.drone_button.clicked.connect(
            lambda: self.set_view("Drone")
        )

        self.husky_button.clicked.connect(
            lambda: self.set_view("Husky")
        )

        self.both_button.clicked.connect(
            lambda: self.set_view("Both")
        )

        layout.addLayout(selector)

        # =========================================================
        # CAMERA STACK
        # =========================================================

        self.camera_stack = QStackedWidget()

        # ---------------------------------------------------------
        # DRONE PAGE
        # ---------------------------------------------------------

        self.drone_page = QWidget()

        drone_page_layout = QVBoxLayout(
            self.drone_page
        )

        drone_page_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        self.drone_camera = self.create_camera_view(
            robot="Drone",
            telemetry="POS: X —   Y —   REL ALT: —   VEL: —   HDG: —"
        )

        drone_page_layout.addWidget(
            self.drone_camera
        )

        # ---------------------------------------------------------
        # HUSKY PAGE
        # ---------------------------------------------------------

        self.husky_page = QWidget()

        husky_page_layout = QVBoxLayout(
            self.husky_page
        )

        husky_page_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        self.husky_camera = self.create_camera_view(
            robot="Husky",
            telemetry="POS: X —   Y —   VEL: —   HDG: —"
        )

        husky_page_layout.addWidget(
            self.husky_camera
        )

        # ---------------------------------------------------------
        # BOTH PAGE
        # ---------------------------------------------------------

        self.both_page = QWidget()

        both_layout = QVBoxLayout(
            self.both_page
        )

        both_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        both_layout.setSpacing(6)

        # Drone camera - top
        self.both_drone_camera = self.create_camera_view(
            robot="Drone",
            telemetry="AGL: 35 m   VEL: 12 m/s   HDG: 072°",
            compact=True,
        )

        # Husky camera - bottom
        self.both_husky_camera = self.create_camera_view(
            robot="Husky",
            telemetry="VEL: 0.8 m/s   HDG: 072°",
            compact=True,
        )

        both_layout.addWidget(
            self.both_drone_camera,
            1,
        )

        both_layout.addWidget(
            self.both_husky_camera,
            1,
        )

        # ---------------------------------------------------------
        # ADD PAGES
        # ---------------------------------------------------------

        self.camera_stack.addWidget(
            self.drone_page
        )

        self.camera_stack.addWidget(
            self.husky_page
        )

        self.camera_stack.addWidget(
            self.both_page
        )

        layout.addWidget(
            self.camera_stack,
            3,
        )

        # =========================================================
        # ROBOT ACTION LOG
        # =========================================================

        log_header = QHBoxLayout()

        log_title = QLabel(
            "Robot Action Log"
        )

        log_title.setObjectName(
            "sectionTitle"
        )

        log_header.addWidget(
            log_title
        )

        log_header.addStretch()

        layout.addLayout(
            log_header
        )

        self.action_log = QTextEdit()

        self.action_log.setReadOnly(
            True
        )

        self.action_log.setPlainText(
            "Waiting for mission..."
        )

        layout.addWidget(
            self.action_log,
            1,
        )

        # =========================================================
        # MANUAL CONTROL
        # =========================================================

        self.manual_button = QPushButton(
            "Manual Control"
        )

        layout.addWidget(
            self.manual_button
        )

        # Start in Drone view.
        self.set_view("Drone")

    # =============================================================
    # CREATE CAMERA VIEW
    # =============================================================

    def create_camera_view(
        self,
        robot,
        telemetry,
        compact=False,
    ):

        camera = QFrame()

        camera.setObjectName(
            "cameraFrame"
        )

        camera_layout = QVBoxLayout(
            camera
        )

        if compact:

            camera_layout.setContentsMargins(
                8,
                8,
                8,
                8,
            )

        else:

            camera_layout.setContentsMargins(
                10,
                10,
                10,
                10,
            )

        # =========================================================
        # CAMERA HEADER
        # =========================================================

        top_camera = QHBoxLayout()

        live_label = QLabel(
            "● LIVE"
        )

        live_label.setObjectName(
            "liveLabel"
        )

        top_camera.addWidget(
            live_label
        )

        top_camera.addStretch()

        camera_source = QLabel(
            f"{robot} Cam"
        )

        camera_source.setObjectName(
            "cameraBadge"
        )

        top_camera.addWidget(
            camera_source
        )

        camera_layout.addLayout(
            top_camera
        )

        # =========================================================
        # CAMERA IMAGE / PLACEHOLDER
        # =========================================================

        camera_title = QLabel(
            f"{robot} POV"
        )

        camera_title.setObjectName(
            "cameraPlaceholder"
        )

        camera_title.setAlignment(
            Qt.AlignCenter
        )

        if compact:

            camera_title.setFont(
                QFont(
                    "Arial",
                    17,
                )
            )

        else:

            camera_title.setFont(
                QFont(
                    "Arial",
                    22,
                )
            )

        camera_layout.addWidget(
            camera_title,
            1,
        )

        # =========================================================
        # TELEMETRY
        # =========================================================

        telemetry_row = QHBoxLayout()

        telemetry_label = QLabel(
            telemetry
        )

        telemetry_label.setObjectName(
            "telemetryBadge"
        )

        timestamp = QLabel(
            "12 Sep 2026 14:32:40"
        )

        timestamp.setObjectName(
            "telemetryBadge"
        )

        telemetry_row.addWidget(
            telemetry_label
        )

        telemetry_row.addStretch()

        telemetry_row.addWidget(
            timestamp
        )

        camera_layout.addLayout(
            telemetry_row
        )

        # Keep references so we can replace these with real ROS
        # camera images and telemetry later.

        camera.camera_title = camera_title
        camera.telemetry_label = telemetry_label
        camera.timestamp_label = timestamp
        camera.camera_source = camera_source
        camera.live_label = live_label

        return camera

    # =============================================================
    # CHANGE CAMERA VIEW
    # =============================================================

    def set_view(
        self,
        view_name,
    ):

        # ---------------------------------------------------------
        # SELECTOR BUTTON STATE
        # ---------------------------------------------------------

        for button, name in (
            (
                self.drone_button,
                "Drone",
            ),
            (
                self.husky_button,
                "Husky",
            ),
            (
                self.both_button,
                "Both",
            ),
        ):

            button.blockSignals(
                True
            )

            button.setChecked(
                name == view_name
            )

            button.blockSignals(
                False
            )

        # ---------------------------------------------------------
        # CAMERA PAGE
        # ---------------------------------------------------------

        if view_name == "Drone":

            self.camera_stack.setCurrentWidget(
                self.drone_page
            )

        elif view_name == "Husky":

            self.camera_stack.setCurrentWidget(
                self.husky_page
            )

        elif view_name == "Both":

            self.camera_stack.setCurrentWidget(
                self.both_page
            )

    # =============================================================
    # ACTION LOG
    # =============================================================

    def append_log(
        self,
        text,
    ):

        self.action_log.append(
            text
        )

    def update_husky_camera(self, image):

        pixmap = QPixmap.fromImage(image)

        # Husky-only view
        self.husky_camera.camera_title.setPixmap(
            pixmap.scaled(
                self.husky_camera.camera_title.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

        # Husky camera in Both view
        self.both_husky_camera.camera_title.setPixmap(
            pixmap.scaled(
                self.both_husky_camera.camera_title.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def update_drone_camera(self, image):

        pixmap = QPixmap.fromImage(image)

        # Drone-only view
        self.drone_camera.camera_title.setPixmap(
            pixmap.scaled(
                self.drone_camera.camera_title.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

        # Drone feed in Both view
        self.both_drone_camera.camera_title.setPixmap(
            pixmap.scaled(
                self.both_drone_camera.camera_title.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def update_husky_telemetry(
        self,
        x,
        y,
        velocity,
        heading,
    ):

        text = (
            f"POS: X {x:.1f}   Y {y:.1f}   "
            f"VEL: {velocity:.1f} m/s   "
            f"HDG: {heading:03.0f}°"
        )

        self.husky_camera.telemetry_label.setText(
            text
        )

        self.both_husky_camera.telemetry_label.setText(
            text
        )

    def update_drone_telemetry(
        self,
        x,
        y,
        relative_altitude,
        velocity,
        heading,
    ):

        text = (
            f"POS: X {x:.1f}   Y {y:.1f}   "
            f"REL ALT: {relative_altitude:.1f} m   "
            f"VEL: {velocity:.1f} m/s   "
            f"HDG: {heading:03.0f}°"
        )

        self.drone_camera.telemetry_label.setText(
            text
        )

        self.both_drone_camera.telemetry_label.setText(
            text
        )