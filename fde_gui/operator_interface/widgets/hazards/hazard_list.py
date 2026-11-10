from datetime import datetime

from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QPainter
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ...paths import ASSETS_DIR


# =============================================================
# SVG ICON
# =============================================================

class SvgIcon(QWidget):
    """
    Displays an SVG while preserving its aspect ratio.
    """

    def __init__(
        self,
        svg_path,
        width=30,
        height=30,
        parent=None,
    ):
        super().__init__(parent)

        self.renderer = QSvgRenderer(
            str(svg_path)
        )

        self.setFixedSize(
            width,
            height,
        )

    def paintEvent(self, event):

        if not self.renderer.isValid():
            return

        painter = QPainter(self)

        painter.setRenderHint(
            QPainter.Antialiasing
        )

        svg_size = (
            self.renderer.defaultSize()
        )

        if (
            svg_size.width() <= 0
            or svg_size.height() <= 0
        ):
            return

        scale = min(
            self.width()
            / svg_size.width(),

            self.height()
            / svg_size.height(),
        )

        draw_width = (
            svg_size.width()
            * scale
        )

        draw_height = (
            svg_size.height()
            * scale
        )

        x = (
            self.width()
            - draw_width
        ) / 2

        y = (
            self.height()
            - draw_height
        ) / 2

        self.renderer.render(
            painter,
            QRectF(
                x,
                y,
                draw_width,
                draw_height,
            ),
        )


# =============================================================
# HAZARD LIST
# =============================================================

class HazardList(QFrame):

    # ---------------------------------------------------------
    # STATUS COLOURS
    # ---------------------------------------------------------

    STATUS_COLOURS = {
        "Inconclusive": "#ffbd16",
        "Husky Required": "#ff9966",
        "In Progress": "#4a9cdc",
        "Resolved / Safe": "#49ad2f",
        "Confirmed Hazard": "#ff3333",
    }

    # ---------------------------------------------------------
    # HAZARD TYPE ICONS
    #
    # Same assets used by LegendPanel.
    # ---------------------------------------------------------

    HAZARD_ICONS = {
        # ROS detection records use "Ice"; the GUI calls it "Ice on Line".
        "ice": "hazard_icons/line_ice.svg",
        "ice on line": "hazard_icons/line_ice.svg",
        # Retain legacy icon support without showing obsolete type filters.
        "slippery ice": "hazard_icons/slippery_ice.svg",
        "no / low snow": "hazard_icons/low_snow.svg",
        "low snow": "hazard_icons/low_snow.svg",
        "wood debris": "hazard_icons/wood_debris.svg",
        "stone debris": "hazard_icons/stone_debris.svg",
    }

    @staticmethod
    def _display_hazard_type(hazard_type):
        """Use a consistent UI name for ROS ice detections."""
        value = str(hazard_type or "").strip()
        if value.casefold() in ("ice", "ice on line"):
            return "Ice on Line"
        return value or "Unknown Hazard"

    def __init__(
        self,
        hazards,
        selection_callback,
        parent=None,
    ):
        super().__init__(parent)

        self.hazards = hazards
        self.selection_callback = (
            selection_callback
        )

        self.selected_hazard_id = None
        self.hazard_cards = {}

        self.assets_dir = ASSETS_DIR

        self.setObjectName(
            "hazardListPanel"
        )

        self._build_ui()

        self.refresh()

    # =============================================================
    # BUILD UI
    # =============================================================

    def _build_ui(self):

        layout = QVBoxLayout(self)

        layout.setContentsMargins(
            10,
            8,
            10,
            8,
        )

        layout.setSpacing(
            4
        )

        # ---------------------------------------------------------
        # TITLE
        # ---------------------------------------------------------

        list_title = QLabel(
            "Hazard List"
        )

        list_title.setObjectName(
            "sectionTitle"
        )

        layout.addWidget(
            list_title
        )

        # ---------------------------------------------------------
        # STATUS FILTER
        # ---------------------------------------------------------

        status_label = QLabel(
            "Filter by Status"
        )

        status_label.setObjectName(
            "hazardFilterLabel"
        )

        layout.addWidget(
            status_label
        )

        self.status_filter = QComboBox()

        self.status_filter.setObjectName(
            "hazardFilter"
        )

        self.status_filter.addItems([
            "All Statuses",
            "Inconclusive",
            "Husky Required",
            "In Progress",
            "Resolved / Safe",
            "Confirmed Hazard",
        ])

        self.status_filter.currentTextChanged.connect(
            self.refresh
        )

        layout.addWidget(
            self.status_filter
        )

        # ---------------------------------------------------------
        # TYPE FILTER
        # ---------------------------------------------------------

        type_label = QLabel(
            "Filter by Type"
        )

        type_label.setObjectName(
            "hazardFilterLabel"
        )

        layout.addWidget(
            type_label
        )

        self.type_filter = QComboBox()

        self.type_filter.setObjectName(
            "hazardFilter"
        )

        self.type_filter.addItems([
            "All Types",
            "Ice on Line",
        ])

        self.type_filter.currentTextChanged.connect(
            self.refresh
        )

        layout.addWidget(
            self.type_filter
        )

        # ---------------------------------------------------------
        # SORT
        # ---------------------------------------------------------

        sort_label = QLabel(
            "Sort by"
        )

        sort_label.setObjectName(
            "hazardFilterLabel"
        )

        layout.addWidget(
            sort_label
        )

        sort_row = QHBoxLayout()

        sort_row.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        sort_row.setSpacing(
            8
        )

        self.sort_filter = QComboBox()

        self.sort_filter.setObjectName(
            "hazardSortFilter"
        )

        self.sort_filter.addItems([
            "Date (Newest First)",
            "Date (Oldest First)",
            "Hazard ID",
            "Hazard Type",
        ])

        self.sort_filter.currentTextChanged.connect(
            self.refresh
        )

        self.clear_filters_button = QPushButton(
            "Clear Filters"
        )

        self.clear_filters_button.setObjectName(
            "clearHazardFilters"
        )

        self.clear_filters_button.clicked.connect(
            self.clear_filters
        )

        sort_row.addWidget(
            self.sort_filter,
            3,
        )

        sort_row.addWidget(
            self.clear_filters_button,
            2,
        )

        layout.addLayout(
            sort_row
        )

        # ---------------------------------------------------------
        # SCROLL AREA
        # ---------------------------------------------------------

        self.scroll = QScrollArea()

        self.scroll.setObjectName(
            "hazardListScroll"
        )

        self.scroll.setWidgetResizable(
            True
        )

        self.scroll.setFrameShape(
            QFrame.NoFrame
        )

        self.scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )

        self.scroll.setVerticalScrollBarPolicy(
            Qt.ScrollBarAsNeeded
        )

        self.scroll_content = QWidget()

        self.scroll_content.setObjectName(
            "hazardListScrollContent"
        )

        self.cards_layout = QVBoxLayout(
            self.scroll_content
        )

        self.cards_layout.setContentsMargins(
            0,
            6,
            0,
            6,
        )

        self.cards_layout.setSpacing(
            8
        )

        self.cards_layout.addStretch()

        self.scroll.setWidget(
            self.scroll_content
        )

        layout.addWidget(
            self.scroll,
            1,
        )

    # =============================================================
    # REFRESH
    # =============================================================

    def refresh(self):

        # ---------------------------------------------------------
        # REMOVE EXISTING CARDS
        # ---------------------------------------------------------

        while self.cards_layout.count() > 1:

            item = self.cards_layout.takeAt(
                0
            )

            widget = item.widget()

            if widget is not None:
                widget.deleteLater()

        self.hazard_cards.clear()

        # ---------------------------------------------------------
        # CURRENT FILTERS
        # ---------------------------------------------------------

        status_filter = (
            self.status_filter.currentText()
        )

        type_filter = (
            self.type_filter.currentText()
        )

        visible_hazards = []

        # ---------------------------------------------------------
        # FILTER HAZARDS
        # ---------------------------------------------------------

        for (
            hazard_id,
            hazard,
        ) in self.hazards.items():

            if (
                status_filter
                != "All Statuses"
                and hazard["status"]
                != status_filter
            ):
                continue

            if (
                type_filter
                != "All Types"
                and self._display_hazard_type(hazard.get("type"))
                != type_filter
            ):
                continue

            visible_hazards.append(
                (
                    hazard_id,
                    hazard,
                )
            )

        # ---------------------------------------------------------
        # SORT
        # ---------------------------------------------------------

        sort_mode = (
            self.sort_filter.currentText()
        )

        if (
            sort_mode
            == "Date (Newest First)"
        ):

            visible_hazards.sort(
                key=lambda item:
                item[1].get(
                    "time_detected",
                    "",
                ),
                reverse=True,
            )

        elif (
            sort_mode
            == "Date (Oldest First)"
        ):

            visible_hazards.sort(
                key=lambda item:
                item[1].get(
                    "time_detected",
                    "",
                )
            )

        elif sort_mode == "Hazard ID":

            visible_hazards.sort(
                key=lambda item:
                item[0]
            )

        elif sort_mode == "Hazard Type":

            visible_hazards.sort(
                key=lambda item:
                item[1].get(
                    "type",
                    "",
                )
            )

        # ---------------------------------------------------------
        # CREATE CARDS
        # ---------------------------------------------------------

        for (
            hazard_id,
            hazard,
        ) in visible_hazards:

            card = (
                self._create_hazard_card(
                    hazard_id,
                    hazard,
                )
            )

            self.cards_layout.insertWidget(
                self.cards_layout.count()
                - 1,
                card,
            )

            self.hazard_cards[
                hazard_id
            ] = card

    # =============================================================
    # CREATE HAZARD CARD
    # =============================================================

    def _create_hazard_card(
        self,
        hazard_id,
        hazard,
    ):

        card = QFrame()

        card.setObjectName(
            "hazardCard"
        )

        card.setProperty(
            "selected",
            hazard_id
            == self.selected_hazard_id,
        )

        card.setMinimumHeight(
            76
        )

        card.setCursor(
            Qt.PointingHandCursor
        )

        # ---------------------------------------------------------
        # CARD LAYOUT
        # ---------------------------------------------------------

        main = QHBoxLayout(
            card
        )

        main.setContentsMargins(
            10,
            7,
            7,
            7,
        )

        main.setSpacing(
            7
        )

        # ---------------------------------------------------------
        # STATUS DOT
        # ---------------------------------------------------------

        status_colour = (
            self.STATUS_COLOURS.get(
                hazard.get(
                    "status"
                ),
                "#999999",
            )
        )

        # Draw a status circle using Qt styles rather than a Unicode
        # bullet, which can render as a question mark on some systems.
        status_dot = QLabel("")
        status_dot.setObjectName("hazardStatusDot")
        status_dot.setFixedSize(20, 20)
        status_dot.setStyleSheet(
            f"background-color: {status_colour}; "
            "border-radius: 10px; "
            "border: none;"
        )

        main.addWidget(
            status_dot,
            alignment=Qt.AlignVCenter,
        )

        # ---------------------------------------------------------
        # HAZARD TYPE SVG
        # ---------------------------------------------------------

        hazard_icon = (
            self._hazard_icon(
                hazard.get(
                    "type",
                    "",
                )
            )
        )

        main.addWidget(
            hazard_icon,
            alignment=Qt.AlignVCenter,
        )

        # ---------------------------------------------------------
        # TEXT
        # ---------------------------------------------------------

        info_layout = QVBoxLayout()

        info_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        info_layout.setSpacing(
            1
        )

        # Split long ROS hazard IDs across two lines
        display_id = hazard_id

        if len(hazard_id) > 17 and "-" in hazard_id:
            prefix, suffix = hazard_id.rsplit("-", 1)
            display_id = f"{prefix}\n{suffix}"

        hazard_id_label = QLabel(display_id)
        hazard_id_label.setObjectName("hazardCardId")

        hazard_id_label.setWordWrap(True)
        hazard_id_label.setStyleSheet("""
            QLabel {
                font-size: 11px;
                font-weight: bold;
            }
        """)

        hazard_id_label.setToolTip(hazard_id)

        hazard_type_label = QLabel(
            self._display_hazard_type(hazard.get("type"))
        )

        hazard_type_label.setObjectName(
            "hazardCardType"
        )

        date_label = QLabel(
            self._format_time(
                hazard.get(
                    "time_detected"
                )
            )
        )

        date_label.setObjectName(
            "hazardCardDate"
        )

        info_layout.addWidget(
            hazard_id_label
        )

        info_layout.addWidget(
            hazard_type_label
        )

        info_layout.addWidget(
            date_label
        )

        main.addLayout(
            info_layout,
            1,
        )

        # ---------------------------------------------------------
        # CHEVRON
        # ---------------------------------------------------------

        chevron = QLabel(
            ":"
        )

        chevron.setObjectName(
            "hazardCardChevron"
        )

        chevron.setFixedWidth(
            24
        )

        chevron.setAlignment(
            Qt.AlignCenter
        )

        main.addWidget(
            chevron,
            alignment=Qt.AlignVCenter,
        )

        # ---------------------------------------------------------
        # CLICK CARD
        # ---------------------------------------------------------

        card.mousePressEvent = (
            lambda event,
            h_id=hazard_id:
            self.select_hazard(
                h_id
            )
        )

        return card

    # =============================================================
    # SELECT HAZARD
    # =============================================================

    def select_hazard(
        self,
        hazard_id,
    ):

        self.selected_hazard_id = (
            hazard_id
        )

        self.refresh()

        self.selection_callback(
            hazard_id
        )

    # =============================================================
    # HAZARD ICON
    # =============================================================

    def _hazard_icon(
        self,
        hazard_type,
    ):

        relative_path = (
            self.HAZARD_ICONS.get(
                str(hazard_type or "").strip().casefold()
            )
        )

        if relative_path is None:

            return self._icon_placeholder()

        path = (
            self.assets_dir
            / relative_path
        )

        if not path.exists():

            print(
                f"[HazardList] "
                f"Missing asset: {path}"
            )

            return self._icon_placeholder()

        icon = SvgIcon(
            path,
            width=30,
            height=30,
        )

        if not icon.renderer.isValid():

            print(
                f"[HazardList] "
                f"Failed to load asset: "
                f"{path}"
            )

            return self._icon_placeholder()

        return icon

    # =============================================================
    # ICON PLACEHOLDER
    # =============================================================

    def _icon_placeholder(self):

        placeholder = QLabel(
            "?"
        )

        placeholder.setObjectName(
            "hazardIconPlaceholder"
        )

        placeholder.setFixedSize(
            30,
            30,
        )

        placeholder.setAlignment(
            Qt.AlignCenter
        )

        return placeholder

    # =============================================================
    # CLEAR FILTERS
    # =============================================================

    def clear_filters(self):

        self.status_filter.blockSignals(
            True
        )

        self.type_filter.blockSignals(
            True
        )

        self.sort_filter.blockSignals(
            True
        )

        self.status_filter.setCurrentIndex(
            0
        )

        self.type_filter.setCurrentIndex(
            0
        )

        self.sort_filter.setCurrentIndex(
            0
        )

        self.status_filter.blockSignals(
            False
        )

        self.type_filter.blockSignals(
            False
        )

        self.sort_filter.blockSignals(
            False
        )

        self.refresh()

    # =============================================================
    # FORMAT TIME
    # =============================================================

    def _format_time(
        self,
        value,
    ):

        if (
            value is None
            or value == ""
            or value == "_"
        ):
            return "_"

        try:

            timestamp = (
                datetime.fromisoformat(
                    str(value)
                )
            )

            return timestamp.strftime(
                "%d %b %Y %H:%M"
            )

        except (
            ValueError,
            TypeError,
        ):

            return str(
                value
            )