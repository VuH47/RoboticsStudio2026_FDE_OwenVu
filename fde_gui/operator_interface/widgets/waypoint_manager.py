from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QRadioButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class WaypointManager(QFrame):
    def __init__(
        self,
        waypoint_store,
        parent=None,
    ):
        super().__init__(parent)

        self.waypoint_store = waypoint_store

        self.setObjectName("waypointManager")

        # =========================================================
        # MAIN LAYOUT
        # =========================================================

        main = QVBoxLayout(self)
        main.setContentsMargins(8, 6, 8, 6)
        main.setSpacing(5)

        # =========================================================
        # HEADER
        # =========================================================

        header = QHBoxLayout()

        self.title = QLabel("Waypoint Manager")
        self.title.setObjectName("waypointManagerTitle")

        header.addWidget(self.title)
        header.addStretch()

        self.clear_button = QPushButton("Clear Manual")
        self.clear_button.setObjectName("waypointClearButton")

        header.addWidget(self.clear_button)

        main.addLayout(header)

        # =========================================================
        # DRONE WAYPOINTS
        # =========================================================

        self.drone_label = QLabel("Drone")
        self.drone_label.setObjectName("waypointRobotHeader")

        main.addWidget(self.drone_label)

        self.drone_table = self._create_waypoint_table()

        main.addWidget(
            self.drone_table,
            1,
        )

        # =========================================================
        # SEPARATOR
        # =========================================================

        separator = QFrame()
        separator.setObjectName("legendSeparator")
        separator.setFrameShape(QFrame.HLine)
        separator.setFixedHeight(1)

        main.addWidget(separator)

        # =========================================================
        # HUSKY WAYPOINTS
        # =========================================================

        self.husky_label = QLabel("Husky")
        self.husky_label.setObjectName("waypointRobotHeader")

        main.addWidget(self.husky_label)

        self.husky_table = self._create_waypoint_table()

        main.addWidget(
            self.husky_table,
            1,
        )

        # =========================================================
        # BOTTOM CONTROLS
        # =========================================================

        bottom = QHBoxLayout()

        self.add_button = QPushButton("+  Add Waypoint")
        self.add_button.setObjectName("waypointAddButton")
        self.add_button.setMinimumWidth(150)

        bottom.addWidget(self.add_button)

        bottom.addSpacing(20)

        self.after_mission_radio = QRadioButton(
            "After current mission"
        )
        self.after_mission_radio.setChecked(True)

        self.execute_now_radio = QRadioButton(
            "Execute now"
        )

        bottom.addWidget(self.after_mission_radio)
        bottom.addSpacing(12)
        bottom.addWidget(self.execute_now_radio)
        bottom.addStretch()

        main.addLayout(bottom)

        # =========================================================
        # STORE CONNECTIONS
        # =========================================================

        self.waypoint_store.changed.connect(
            self.refresh
        )

        self.clear_button.clicked.connect(
            self.waypoint_store.clear_manual
        )

        # Initial table population.
        self.refresh()

    # =============================================================
    # CREATE WAYPOINT TABLE
    # =============================================================

    def _create_waypoint_table(self):
        table = QTableWidget()

        table.setObjectName("waypointTable")

        table.setColumnCount(6)

        table.setHorizontalHeaderLabels([
            "#",
            "Type",
            "Name",
            "Coordinates (m)",
            "Target / Description",
            "Actions",
        ])

        table.verticalHeader().setVisible(False)

        table.setShowGrid(True)

        table.setSelectionBehavior(
            QTableWidget.SelectRows
        )

        table.setSelectionMode(
            QTableWidget.SingleSelection
        )

        table.setEditTriggers(
            QTableWidget.NoEditTriggers
        )

        table.setVerticalScrollBarPolicy(
            Qt.ScrollBarAsNeeded
        )

        table.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAsNeeded
        )

        header = table.horizontalHeader()

        header.setSectionResizeMode(
            0,
            QHeaderView.ResizeToContents,
        )

        header.setSectionResizeMode(
            1,
            QHeaderView.ResizeToContents,
        )

        header.setSectionResizeMode(
            2,
            QHeaderView.ResizeToContents,
        )

        header.setSectionResizeMode(
            3,
            QHeaderView.ResizeToContents,
        )

        header.setSectionResizeMode(
            4,
            QHeaderView.Stretch,
        )

        header.setSectionResizeMode(
            5,
            QHeaderView.ResizeToContents,
        )

        return table

    # =============================================================
    # REFRESH
    # =============================================================

    def refresh(self):
        """
        Reload both tables from the shared WaypointStore.
        """

        drone_waypoints = (
            self.waypoint_store.get_drone_waypoints()
        )

        husky_waypoints = (
            self.waypoint_store.get_husky_waypoints()
        )

        self._populate_table(
            self.drone_table,
            drone_waypoints,
            "drone",
        )

        self._populate_table(
            self.husky_table,
            husky_waypoints,
            "husky",
        )

    # =============================================================
    # POPULATE TABLE
    # =============================================================

    def _populate_table(
        self,
        table,
        waypoints,
        robot,
    ):
        table.setRowCount(
            len(waypoints)
        )

        for row, waypoint in enumerate(waypoints):

            # -----------------------------------------------------
            # QUEUE NUMBER
            # -----------------------------------------------------

            self._set_text(
                table,
                row,
                0,
                str(row + 1),
            )

            # -----------------------------------------------------
            # TYPE
            # -----------------------------------------------------

            self._set_type(
                table,
                row,
                waypoint["type"],
            )

            # -----------------------------------------------------
            # NAME / ID
            # -----------------------------------------------------

            self._set_text(
                table,
                row,
                2,
                waypoint["id"],
            )

            # -----------------------------------------------------
            # COORDINATES
            # -----------------------------------------------------

            if robot == "drone":
                coords = (
                    f"({waypoint['x']:.1f}, "
                    f"{waypoint['y']:.1f}, "
                    f"{waypoint['z']:.1f})"
                )

            else:
                coords = (
                    f"({waypoint['x']:.1f}, "
                    f"{waypoint['y']:.1f})"
                )

            self._set_text(
                table,
                row,
                3,
                coords,
            )

            # -----------------------------------------------------
            # DESCRIPTION
            # -----------------------------------------------------

            self._set_text(
                table,
                row,
                4,
                waypoint.get(
                    "description",
                    "",
                ),
            )

            # -----------------------------------------------------
            # ACTIONS
            # -----------------------------------------------------

            table.setCellWidget(
                row,
                5,
                self._create_action_buttons(
                    robot=robot,
                    index=row,
                    can_delete=waypoint.get(
                        "can_delete",
                        True,
                    ),
                ),
            )

        table.resizeRowsToContents()

    # =============================================================
    # TEXT ITEM
    # =============================================================

    def _set_text(
        self,
        table,
        row,
        column,
        text,
    ):
        item = QTableWidgetItem(
            str(text)
        )

        item.setTextAlignment(
            Qt.AlignLeft | Qt.AlignVCenter
        )

        table.setItem(
            row,
            column,
            item,
        )

    # =============================================================
    # WAYPOINT TYPE
    # =============================================================

    def _set_type(
        self,
        table,
        row,
        waypoint_type,
    ):
        item = QTableWidgetItem()

        if waypoint_type == "Mission":

            item.setText(
                "●  Mission"
            )

            item.setForeground(
                QColor("#ff9d45")
            )

        else:

            item.setText(
                "●  Manual"
            )

            item.setForeground(
                QColor("#4aa3ff")
            )

        item.setTextAlignment(
            Qt.AlignLeft | Qt.AlignVCenter
        )

        table.setItem(
            row,
            1,
            item,
        )

    # =============================================================
    # ACTION BUTTONS
    # =============================================================

    def _create_action_buttons(
        self,
        robot,
        index,
        can_delete=True,
    ):
        widget = QWidget()

        layout = QHBoxLayout(widget)

        layout.setContentsMargins(
            2,
            0,
            2,
            0,
        )

        layout.setSpacing(2)

        # =========================================================
        # MOVE UP
        # =========================================================

        up = QPushButton("↑")

        up.setObjectName(
            "waypointActionButton"
        )

        up.setFixedSize(
            24,
            22,
        )

        up.clicked.connect(
            lambda checked=False,
            r=robot,
            i=index:
                self.waypoint_store.move_up(
                    r,
                    i,
                )
        )

        # =========================================================
        # MOVE DOWN
        # =========================================================

        down = QPushButton("↓")

        down.setObjectName(
            "waypointActionButton"
        )

        down.setFixedSize(
            24,
            22,
        )

        down.clicked.connect(
            lambda checked=False,
            r=robot,
            i=index:
                self.waypoint_store.move_down(
                    r,
                    i,
                )
        )

        layout.addWidget(up)
        layout.addWidget(down)

        # =========================================================
        # DELETE
        # =========================================================

        if can_delete:

            delete = QPushButton("×")

            delete.setObjectName(
                "waypointActionButton"
            )

            delete.setFixedSize(
                24,
                22,
            )

            delete.clicked.connect(
                lambda checked=False,
                r=robot,
                i=index:
                    self.waypoint_store.delete_waypoint(
                        r,
                        i,
                    )
            )

            layout.addWidget(delete)

        else:

            # Keep Mission rows aligned with Manual rows.
            placeholder = QWidget()

            placeholder.setFixedSize(
                24,
                22,
            )

            layout.addWidget(
                placeholder
            )

        return widget