from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QPainter
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ...paths import ASSETS_DIR
from ..common.toggle_switch import ToggleSwitch
from .map_overlay_panel import (
    FIRST_COLUMN_WIDTH,
    SECOND_COLUMN_WIDTH,
    THIRD_COLUMN_WIDTH,
    CHILD_INDENT,
    SECTION_LEFT_MARGIN,
    RIGHT_MARGIN,
    COLUMN_SPACING,
)


class SvgIcon(QWidget):
    """Displays an SVG while preserving its original aspect ratio."""

    def __init__(
        self,
        svg_path,
        width=22,
        height=17,
        parent=None,
    ):
        super().__init__(parent)

        self.renderer = QSvgRenderer(str(svg_path))
        self.setFixedSize(width, height)

    def paintEvent(self, event):
        if not self.renderer.isValid():
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        svg_size = self.renderer.defaultSize()

        if (
            svg_size.width() <= 0
            or svg_size.height() <= 0
        ):
            return

        scale = min(
            self.width() / svg_size.width(),
            self.height() / svg_size.height(),
        )

        draw_width = svg_size.width() * scale
        draw_height = svg_size.height() * scale

        x = (self.width() - draw_width) / 2
        y = (self.height() - draw_height) / 2

        self.renderer.render(
            painter,
            QRectF(
                x,
                y,
                draw_width,
                draw_height,
            ),
        )


class LegendPanel(QWidget):
    """
    Legend page only.

    The Legend / Waypoints tabs are owned by MapOverlayPanel.
    """

    def __init__(
        self,
        status_colours,
        layer_callback=None,
        parent=None,
    ):
        super().__init__(parent)

        self.status_colours = status_colours
        self.layer_callback = layer_callback

        self.assets_dir = ASSETS_DIR
        print(f"[LegendPanel] Assets directory: {self.assets_dir}")

        # =========================================================
        # PAGE LAYOUT
        # =========================================================

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(3)

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
        header.setHorizontalSpacing(COLUMN_SPACING)

        item_header = QLabel("ITEM")
        symbol_header = QLabel("SYMBOL")
        view_header = QLabel("VIEW")

        for label in (
            item_header,
            symbol_header,
            view_header,
        ):
            label.setObjectName("legendColumnHeader")

        item_header.setAlignment(
            Qt.AlignLeft | Qt.AlignVCenter
        )
        symbol_header.setAlignment(Qt.AlignCenter)
        view_header.setAlignment(Qt.AlignCenter)

        view_header.setContentsMargins(
            -6, 0, 6, 0
        )

        header.addWidget(item_header, 0, 0)
        header.addWidget(symbol_header, 0, 1)
        header.addWidget(view_header, 0, 2)

        self._set_column_widths(header)

        outer.addLayout(header)

        self._add_separator(outer)

        # =========================================================
        # SCROLLABLE CONTENT
        # =========================================================

        self.legend_scroll_area = QScrollArea()
        self.legend_scroll_area.setObjectName(
            "legendScrollArea"
        )

        self.legend_scroll_area.setWidgetResizable(True)
        self.legend_scroll_area.setFrameShape(
            QFrame.NoFrame
        )

        self.legend_scroll_area.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )

        self.legend_scroll_area.setVerticalScrollBarPolicy(
            Qt.ScrollBarAsNeeded
        )

        self.legend_scroll_content = QWidget()
        self.legend_scroll_content.setObjectName(
            "legendScrollContent"
        )

        self.legend_scroll_area.setWidget(
            self.legend_scroll_content
        )

        outer.addWidget(self.legend_scroll_area)

        legend_layout = QVBoxLayout(
            self.legend_scroll_content
        )

        legend_layout.setContentsMargins(
            0, 0, 0, 0
        )
        legend_layout.setSpacing(3)

        # =========================================================
        # DRONE
        # =========================================================

        self.drone_layer_check = self._add_section(
            legend_layout,
            "Drone",
        )

        self.drone_location_check = self._add_item(
            legend_layout,
            "Drone Location",
            image="drone_icons/drone_location.svg",
        )

        self.drone_path_check = self._add_item(
            legend_layout,
            "Drone Pathing",
            image="drone_icons/drone_path.svg",
        )

        self.drone_waypoint_check = self._add_item(
            legend_layout,
            "Drone Waypoint",
            image="drone_icons/drone_waypoint.svg",
        )

        self.drone_home_check = self._add_item(
            legend_layout,
            "Drone Home",
            image="drone_icons/drone_home.svg",
        )

        self._add_separator(legend_layout)

        # =========================================================
        # HUSKY
        # =========================================================

        self.husky_layer_check = self._add_section(
            legend_layout,
            "Husky",
        )

        self.husky_location_check = self._add_item(
            legend_layout,
            "Husky Location",
            image="husky_icons/husky_location.svg",
        )

        self.husky_path_check = self._add_item(
            legend_layout,
            "Husky Pathing",
            image="husky_icons/husky_path.svg",
        )

        self.husky_waypoint_check = self._add_item(
            legend_layout,
            "Husky Waypoint",
            image="husky_icons/husky_waypoint.svg",
        )

        self.husky_home_check = self._add_item(
            legend_layout,
            "Husky Home",
            image="husky_icons/husky_home.svg",
        )

        self._add_separator(legend_layout)

        # =========================================================
        # HAZARD STATUS
        # =========================================================

        self.hazard_status_check = self._add_section(
            legend_layout,
            "Hazard Status",
        )

        self.detected_check = self._add_item(
            legend_layout,
            "Detected",
            colour=self.status_colours["Detected"],
        )

        self.husky_required_check = self._add_item(
            legend_layout,
            "Husky Required",
            colour=self.status_colours[
                "Husky Required"
            ],
        )

        self.in_progress_check = self._add_item(
            legend_layout,
            "In Progress",
            colour=self.status_colours["In Progress"],
        )

        self.resolved_check = self._add_item(
            legend_layout,
            "Resolved / Safe",
            colour=self.status_colours[
                "Resolved / Safe"
            ],
        )

        self.confirmed_check = self._add_item(
            legend_layout,
            "Confirmed Hazard",
            colour=self.status_colours[
                "Confirmed Hazard"
            ],
        )

        self._add_separator(legend_layout)

        # =========================================================
        # HAZARD TYPES
        # =========================================================

        self.hazard_layer_check = self._add_section(
            legend_layout,
            "Hazard Types",
        )

        self.line_ice_check = self._add_item(
            legend_layout,
            "Ice on Line",
            image="../hazard_icons/line_ice.svg",
        )

        self.ground_hazard_check = self._add_item(
            legend_layout,
            "Ground Hazard",
            image="../hazard_icons/ground_hazard.svg",
        )

        legend_layout.addStretch()

        # =========================================================
        # CALLBACKS
        # =========================================================

        if self.layer_callback is not None:
            self.drone_layer_check.toggled.connect(
                lambda visible: self.layer_callback(
                    "drone",
                    visible,
                )
            )

            self.husky_layer_check.toggled.connect(
                lambda visible: self.layer_callback(
                    "husky",
                    visible,
                )
            )

            self.hazard_layer_check.toggled.connect(
                lambda visible: self.layer_callback(
                    "hazards",
                    visible,
                )
            )

    # =============================================================
    # COLUMN WIDTHS
    # =============================================================

    def _set_column_widths(self, layout):
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 0)
        layout.setColumnStretch(2, 0)

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
    # SECTION ROW
    # =============================================================

    def _add_section(
        self,
        parent_layout,
        text,
        image=None,
    ):
        container = QWidget()
        container.setObjectName("legendSectionRow")

        layout = QGridLayout(container)

        layout.setContentsMargins(
            SECTION_LEFT_MARGIN,
            2,
            RIGHT_MARGIN,
            2,
        )

        layout.setHorizontalSpacing(COLUMN_SPACING)

        label = QLabel(text)
        label.setObjectName("legendSectionLabel")

        if image:
            symbol = self._asset_icon(image)
        else:
            symbol = self._asset_placeholder()

        switch = ToggleSwitch()
        switch.setChecked(True)

        layout.addWidget(
            label,
            0,
            0,
        )

        layout.addWidget(
            symbol,
            0,
            1,
            alignment=Qt.AlignCenter,
        )

        layout.addWidget(
            switch,
            0,
            2,
            alignment=Qt.AlignCenter,
        )

        self._set_column_widths(layout)

        parent_layout.addWidget(container)

        return switch

    # =============================================================
    # CHILD ROW
    # =============================================================

    def _add_item(
        self,
        parent_layout,
        text,
        colour=None,
        image=None,
    ):
        container = QWidget()
        container.setObjectName("legendItemRow")

        layout = QGridLayout(container)

        layout.setContentsMargins(
            SECTION_LEFT_MARGIN,
            1,
            RIGHT_MARGIN,
            1,
        )

        layout.setHorizontalSpacing(COLUMN_SPACING)

        label = QLabel(text)
        label.setObjectName("legendItemLabel")

        label.setContentsMargins(
            CHILD_INDENT - SECTION_LEFT_MARGIN,
            0,
            0,
            0,
        )

        if image:
            symbol = self._asset_icon(image)

        elif colour is not None:
            symbol = self._colour_placeholder(
                colour
            )

        else:
            symbol = self._asset_placeholder()

        switch = ToggleSwitch()
        switch.setChecked(True)

        layout.addWidget(
            label,
            0,
            0,
        )

        layout.addWidget(
            symbol,
            0,
            1,
            alignment=Qt.AlignCenter,
        )

        layout.addWidget(
            switch,
            0,
            2,
            alignment=Qt.AlignCenter,
        )

        self._set_column_widths(layout)

        parent_layout.addWidget(container)

        return switch

    # =============================================================
    # ASSETS
    # =============================================================

    def _asset_icon(
        self,
        relative_path,
        width=22,
        height=17,
    ):
        path = self.assets_dir / relative_path

        # File does not exist
        if not path.exists():
            print(
                f"[LegendPanel] Missing asset: {path}"
            )
            return self._asset_placeholder()

        # File exists — try loading the SVG
        icon = SvgIcon(
            path,
            width,
            height,
        )

        # File exists but Qt could not load it
        if not icon.renderer.isValid():
            print(
                f"[LegendPanel] Failed to load asset: {path}"
            )
            return self._asset_placeholder()

        return icon

    def _asset_placeholder(self):
        placeholder = QLabel("?")
        placeholder.setObjectName("legendAssetPlaceholder")

        placeholder.setFixedSize(
            22,
            17,
        )

        placeholder.setAlignment(
            Qt.AlignCenter
        )

        return placeholder

    def _colour_placeholder(
        self,
        colour,
    ):
        dot = QLabel("●")
        dot.setAlignment(Qt.AlignCenter)
        dot.setFixedSize(22, 17)

        dot.setStyleSheet(
            f"""
            QLabel {{
                color: {colour};
                font-size: 14px;
                background: transparent;
            }}
            """
        )

        return dot

    # =============================================================
    # SEPARATOR
    # =============================================================

    def _add_separator(
        self,
        parent_layout,
    ):
        line = QFrame()
        line.setObjectName("legendSeparator")
        line.setFrameShape(QFrame.HLine)
        line.setFixedHeight(1)

        parent_layout.addWidget(line)