from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from .map_overlay_panel import (
    FIRST_COLUMN_WIDTH,
    SECOND_COLUMN_WIDTH,
    THIRD_COLUMN_WIDTH,
    CHILD_INDENT,
    SECTION_LEFT_MARGIN,
    RIGHT_MARGIN,
    COLUMN_SPACING,
)


class WaypointPanel(QWidget):
    """
    Compact waypoint page shown in the floating map overlay.

    Drone and Husky each have their own independent scrollable list.
    All waypoint data comes from the shared WaypointStore.
    """

    def __init__(
        self,
        waypoint_store=None,
        parent=None,
    ):
        super().__init__(parent)

        self.waypoint_store = waypoint_store

        # =========================================================
        # PAGE LAYOUT
        # =========================================================

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(4)

        # =========================================================
        # FIXED COLUMN HEADER
        # =========================================================

        header = QGridLayout()

        header.setContentsMargins(
            SECTION_LEFT_MARGIN,
            0,
            RIGHT_MARGIN,
            1,
        )

        header.setHorizontalSpacing(
            COLUMN_SPACING
        )

        waypoint_header = QLabel("WAYPOINT")
        type_header = QLabel("TYPE")
        action_header = QLabel("ACTION")

        for label in (
            waypoint_header,
            type_header,
            action_header,
        ):
            label.setObjectName(
                "legendColumnHeader"
            )

        waypoint_header.setAlignment(
            Qt.AlignLeft | Qt.AlignVCenter
        )

        type_header.setAlignment(
            Qt.AlignCenter
        )

        action_header.setAlignment(
            Qt.AlignCenter
        )

        header.addWidget(
            waypoint_header,
            0,
            0,
        )

        header.addWidget(
            type_header,
            0,
            1,
        )

        header.addWidget(
            action_header,
            0,
            2,
        )

        self._set_column_widths(
            header
        )

        outer.addLayout(
            header
        )

        self._add_separator(
            outer
        )

        # =========================================================
        # DRONE
        # =========================================================

        drone_header = QLabel("Drone")

        drone_header.setObjectName(
            "legendSectionLabel"
        )

        outer.addWidget(
            drone_header
        )

        self.drone_scroll_area = (
            self._create_waypoint_scroll_area(
                "droneWaypointScrollArea"
            )
        )

        self.drone_waypoint_content = QWidget()

        self.drone_waypoint_content.setObjectName(
            "waypointScrollContent"
        )

        self.drone_waypoint_layout = QVBoxLayout(
            self.drone_waypoint_content
        )

        self.drone_waypoint_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        self.drone_waypoint_layout.setSpacing(
            1
        )

        self.drone_waypoint_layout.setAlignment(
            Qt.AlignTop
        )

        self.drone_scroll_area.setWidget(
            self.drone_waypoint_content
        )

        outer.addWidget(
            self.drone_scroll_area,
            1,
        )

        # =========================================================
        # SEPARATOR
        # =========================================================

        self._add_separator(
            outer
        )

        # =========================================================
        # HUSKY
        # =========================================================

        husky_header = QLabel("Husky")

        husky_header.setObjectName(
            "legendSectionLabel"
        )

        outer.addWidget(
            husky_header
        )

        self.husky_scroll_area = (
            self._create_waypoint_scroll_area(
                "huskyWaypointScrollArea"
            )
        )

        self.husky_waypoint_content = QWidget()

        self.husky_waypoint_content.setObjectName(
            "waypointScrollContent"
        )

        self.husky_waypoint_layout = QVBoxLayout(
            self.husky_waypoint_content
        )

        self.husky_waypoint_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        self.husky_waypoint_layout.setSpacing(
            1
        )

        self.husky_waypoint_layout.setAlignment(
            Qt.AlignTop
        )

        self.husky_scroll_area.setWidget(
            self.husky_waypoint_content
        )

        outer.addWidget(
            self.husky_scroll_area,
            1,
        )

        # =========================================================
        # WAYPOINT STORE
        # =========================================================

        if self.waypoint_store is not None:

            self.waypoint_store.changed.connect(
                self.refresh
            )

        self.refresh()

    # =============================================================
    # REFRESH FROM WAYPOINT STORE
    # =============================================================

    def refresh(self):

        self._clear_waypoint_layout(
            self.drone_waypoint_layout
        )

        self._clear_waypoint_layout(
            self.husky_waypoint_layout
        )

        if self.waypoint_store is None:
            return

        # =========================================================
        # DRONE
        # =========================================================

        drone_waypoints = (
            self.waypoint_store.get_drone_waypoints()
        )

        for index, waypoint in enumerate(
            drone_waypoints
        ):

            self._add_waypoint_row(
                parent_layout=self.drone_waypoint_layout,
                name=(
                    f"{index + 1}   "
                    f"{waypoint['id']}"
                ),
                waypoint_type=waypoint["type"],
                can_delete=waypoint.get(
                    "can_delete",
                    True,
                ),
                robot="drone",
                index=index,
            )

        # =========================================================
        # HUSKY
        # =========================================================

        husky_waypoints = (
            self.waypoint_store.get_husky_waypoints()
        )

        for index, waypoint in enumerate(
            husky_waypoints
        ):

            self._add_waypoint_row(
                parent_layout=self.husky_waypoint_layout,
                name=(
                    f"{index + 1}   "
                    f"{waypoint['id']}"
                ),
                waypoint_type=waypoint["type"],
                can_delete=waypoint.get(
                    "can_delete",
                    True,
                ),
                robot="husky",
                index=index,
            )

    # =============================================================
    # CLEAR WAYPOINT LAYOUT
    # =============================================================

    def _clear_waypoint_layout(
        self,
        layout,
    ):

        while layout.count():

            item = layout.takeAt(0)

            widget = item.widget()

            if widget is not None:
                widget.deleteLater()

    # =============================================================
    # WAYPOINT SCROLL AREA
    # =============================================================

    def _create_waypoint_scroll_area(
        self,
        object_name,
    ):

        scroll = QScrollArea()

        scroll.setObjectName(
            object_name
        )

        scroll.setWidgetResizable(
            True
        )

        scroll.setFrameShape(
            QFrame.NoFrame
        )

        scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )

        scroll.setVerticalScrollBarPolicy(
            Qt.ScrollBarAlwaysOn
        )

        return scroll

    # =============================================================
    # COLUMN WIDTHS
    # =============================================================

    def _set_column_widths(
        self,
        layout,
    ):

        layout.setColumnStretch(
            0,
            0,
        )

        layout.setColumnStretch(
            1,
            0,
        )

        layout.setColumnStretch(
            2,
            0,
        )

        layout.setColumnMinimumWidth(
            0,
            FIRST_COLUMN_WIDTH,
        )

        layout.setColumnMinimumWidth(
            1,
            SECOND_COLUMN_WIDTH,
        )

        layout.setColumnMinimumWidth(
            2,
            THIRD_COLUMN_WIDTH,
        )

    # =============================================================
    # WAYPOINT ROW
    # =============================================================

    def _add_waypoint_row(
        self,
        parent_layout,
        name,
        waypoint_type,
        can_delete=True,
        robot=None,
        index=None,
    ):

        container = QWidget()

        container.setObjectName(
            "waypointRow"
        )

        layout = QGridLayout(
            container
        )

        layout.setContentsMargins(
            SECTION_LEFT_MARGIN,
            2,
            RIGHT_MARGIN,
            2,
        )

        layout.setHorizontalSpacing(
            COLUMN_SPACING
        )

        # ---------------------------------------------------------
        # WAYPOINT NAME
        # ---------------------------------------------------------

        name_label = QLabel(
            name
        )

        name_label.setObjectName(
            "legendItemLabel"
        )

        name_label.setContentsMargins(
            CHILD_INDENT - SECTION_LEFT_MARGIN,
            0,
            0,
            0,
        )

        # ---------------------------------------------------------
        # TYPE
        # ---------------------------------------------------------

        type_label = QLabel(
            waypoint_type
        )

        type_label.setObjectName(
            "waypointTypeLabel"
        )

        type_label.setAlignment(
            Qt.AlignCenter
        )

        # ---------------------------------------------------------
        # ACTIONS
        # ---------------------------------------------------------

        actions = QWidget()

        action_layout = QHBoxLayout(
            actions
        )

        action_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        action_layout.setSpacing(
            3
        )

        up_button = QPushButton("↑")
        down_button = QPushButton("↓")

        for button in (
            up_button,
            down_button,
        ):

            button.setObjectName(
                "waypointSmallButton"
            )

            button.setFixedSize(
                18,
                18,
            )

        # Disable impossible moves at the top/bottom of each queue.
        queue_length = 0

        if (
            self.waypoint_store is not None
            and robot == "drone"
        ):
            queue_length = len(
                self.waypoint_store.get_drone_waypoints()
            )

        elif (
            self.waypoint_store is not None
            and robot == "husky"
        ):
            queue_length = len(
                self.waypoint_store.get_husky_waypoints()
            )

        if index is not None:

            up_button.setEnabled(
                index > 0
            )

            down_button.setEnabled(
                index < queue_length - 1
            )

        # Connect reorder operations.
        if (
            self.waypoint_store is not None
            and robot is not None
            and index is not None
        ):

            up_button.clicked.connect(
                lambda checked=False,
                r=robot,
                i=index:
                    self.waypoint_store.move_up(
                        r,
                        i,
                    )
            )

            down_button.clicked.connect(
                lambda checked=False,
                r=robot,
                i=index:
                    self.waypoint_store.move_down(
                        r,
                        i,
                    )
            )

        action_layout.addWidget(
            up_button
        )

        action_layout.addWidget(
            down_button
        )

        # ---------------------------------------------------------
        # DELETE
        # ---------------------------------------------------------

        if can_delete:

            delete_button = QPushButton(
                "×"
            )

            delete_button.setObjectName(
                "waypointDeleteButton"
            )

            delete_button.setFixedSize(
                18,
                18,
            )

            if (
                self.waypoint_store is not None
                and robot is not None
                and index is not None
            ):

                delete_button.clicked.connect(
                    lambda checked=False,
                    r=robot,
                    i=index:
                        self.waypoint_store.delete_waypoint(
                            r,
                            i,
                        )
                )

            action_layout.addWidget(
                delete_button
            )

        else:

            # Mission waypoints cannot be manually deleted.
            # Reserve the same width so action columns stay aligned.
            delete_placeholder = QWidget()

            delete_placeholder.setFixedSize(
                18,
                18,
            )

            action_layout.addWidget(
                delete_placeholder
            )

        # ---------------------------------------------------------
        # ADD TO ROW
        # ---------------------------------------------------------

        layout.addWidget(
            name_label,
            0,
            0,
        )

        layout.addWidget(
            type_label,
            0,
            1,
        )

        layout.addWidget(
            actions,
            0,
            2,
        )

        self._set_column_widths(
            layout
        )

        parent_layout.addWidget(
            container
        )

    # =============================================================
    # SEPARATOR
    # =============================================================

    def _add_separator(
        self,
        parent_layout,
    ):

        line = QFrame()

        line.setObjectName(
            "legendSeparator"
        )

        line.setFrameShape(
            QFrame.HLine
        )

        line.setFixedHeight(
            1
        )

        parent_layout.addWidget(
            line
        )
