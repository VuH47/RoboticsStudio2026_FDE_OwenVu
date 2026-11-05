from PySide6.QtCore import QEasingCurve, Property, QPropertyAnimation, Qt, QTimer
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QPushButton, QSizePolicy,
    QStackedWidget, QVBoxLayout,
)

# Shared constants used by legend_panel.py and waypoint_panel.py
FIRST_COLUMN_WIDTH = 100
SECOND_COLUMN_WIDTH = 55
THIRD_COLUMN_WIDTH = 65
CHILD_INDENT = 18
SECTION_LEFT_MARGIN = 3
RIGHT_MARGIN = 3
COLUMN_SPACING = 3
WAYPOINT_SCROLL_MIN_HEIGHT = 80
WAYPOINT_SCROLL_MAX_HEIGHT = 115


class MapOverlayPanel(QFrame):
    """Sliding map legend / waypoint panel with an embedded-looking toggle.

    The toggle is a viewport sibling for reliable visibility while collapsed,
    but appears inside the panel header when expanded.
    """

    ANIMATION_MS = 260
    TOGGLE_SIZE = 26
    TOGGLE_INSET = 6

    def __init__(self, status_colours, waypoint_store=None,
                 layer_callback=None, parent=None):
        super().__init__(parent)
        from .legend_panel import LegendPanel
        from .waypoint_panel import WaypointPanel

        self.setObjectName("mapLegend")
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)

        self._expanded = True
        self._open_x = 10
        self._top_y = 10
        self._slide_offset = 0.0

        self._animation = QPropertyAnimation(self, b"slideOffset", self)
        self._animation.setDuration(self.ANIMATION_MS)
        self._animation.setEasingCurve(QEasingCurve.InOutCubic)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(3)

        tabs = QHBoxLayout()
        tabs.setSpacing(2)
        # Reserve room for the floating toggle within the header.
        tabs.setContentsMargins(0, 0, self.TOGGLE_SIZE + 5, 0)
        self.legend_tab_button = QPushButton("Legend")
        self.waypoints_tab_button = QPushButton("Waypoints")
        for button in (self.legend_tab_button, self.waypoints_tab_button):
            button.setObjectName("mapOverlayTab")
            button.setCheckable(True)
        self.legend_tab_button.setChecked(True)
        tabs.addWidget(self.legend_tab_button)
        tabs.addWidget(self.waypoints_tab_button)
        outer.addLayout(tabs)

        self.pages = QStackedWidget()
        self.legend_panel = LegendPanel(
            status_colours=status_colours,
            layer_callback=layer_callback,
        )
        self.waypoint_panel = WaypointPanel(waypoint_store=waypoint_store)
        self.pages.addWidget(self.legend_panel)
        self.pages.addWidget(self.waypoint_panel)
        outer.addWidget(self.pages)

        self.legend_tab_button.clicked.connect(lambda: self.set_panel_page(0))
        self.waypoints_tab_button.clicked.connect(lambda: self.set_panel_page(1))

        # A viewport sibling visually embedded in the panel header when open.
        # It remains visible after the panel slides completely off-screen.
        self.collapse_button = QPushButton("❮", parent)
        self.collapse_button.setObjectName("mapSidebarToggle")
        self.collapse_button.setFixedSize(self.TOGGLE_SIZE, self.TOGGLE_SIZE)
        self.collapse_button.setCursor(Qt.PointingHandCursor)
        self.collapse_button.setToolTip("Collapse legend and waypoints")
        self.collapse_button.clicked.connect(self.toggle_collapsed)
        self._position_toggle()
        QTimer.singleShot(0, self._raise_toggle)

    def _raise_toggle(self):
        if hasattr(self, "collapse_button"):
            self.collapse_button.show()
            self.collapse_button.raise_()

    def raise_(self):
        """Keep the independent toggle above this panel when callers raise it."""
        super().raise_()
        self._raise_toggle()

    def set_panel_page(self, page):
        self.pages.setCurrentIndex(page)
        for button, checked in (
            (self.legend_tab_button, page == 0),
            (self.waypoints_tab_button, page == 1),
        ):
            button.blockSignals(True)
            button.setChecked(checked)
            button.blockSignals(False)

    def _get_slide_offset(self):
        return self._slide_offset

    def _set_slide_offset(self, offset):
        self._slide_offset = float(offset)
        self.move(round(self._open_x + self._slide_offset), self._top_y)
        self._position_toggle()

    slideOffset = Property(float, _get_slide_offset, _set_slide_offset)

    def _position_toggle(self):
        """Keep the toggle aligned with the tabs and visible when collapsed."""

        if not hasattr(self, "collapse_button"):
            return

        # Expanded: top-right, within the tab row.
        embedded_x = (
            self.x()
            + self.width()
            - self.TOGGLE_SIZE
            - self.TOGGLE_INSET
        )

        # Collapsed: small square at the left edge.
        x = max(self._open_x, embedded_x)

        self.collapse_button.move(
            round(x),
            self._top_y + self.TOGGLE_INSET,
        )

        self._raise_toggle()

    def toggle_collapsed(self):
        self.set_collapsed(self._expanded)

    def set_collapsed(self, collapsed, animated=True):
        self._expanded = not bool(collapsed)
        self.collapse_button.setText("❯" if collapsed else "❮")
        self.collapse_button.setToolTip(
            "Expand legend and waypoints" if collapsed
            else "Collapse legend and waypoints"
        )

        # Move the ENTIRE panel past the left viewport boundary.
        target = -float(self._open_x + self.width() + 2) if collapsed else 0.0
        self._animation.stop()
        if animated:
            self._animation.setStartValue(self._slide_offset)
            self._animation.setEndValue(target)
            self._animation.start()
        else:
            self._set_slide_offset(target)
        self.raise_()
        self._raise_toggle()
        QTimer.singleShot(0, self._raise_toggle)

    def position_in_viewport(self, y, width, height, left=10):
        """Call from operator_gui.position_map_overlays() instead of setGeometry."""
        self._animation.stop()
        self._open_x = int(left)
        self._top_y = int(y)
        self.resize(int(width), int(height))
        target = 0.0 if self._expanded else -float(self._open_x + self.width() + 2)
        self._set_slide_offset(target)
        self.raise_()
        self._raise_toggle()
        QTimer.singleShot(0, self._raise_toggle)

    @property
    def is_collapsed(self):
        return not self._expanded

    @property
    def drone_layer_check(self): return self.legend_panel.drone_layer_check
    @property
    def drone_location_check(self): return self.legend_panel.drone_location_check
    @property
    def drone_waypoint_check(self): return self.legend_panel.drone_waypoint_check
    @property
    def drone_home_check(self): return self.legend_panel.drone_home_check
    @property
    def husky_layer_check(self): return self.legend_panel.husky_layer_check
    @property
    def husky_location_check(self): return self.legend_panel.husky_location_check
    @property
    def husky_waypoint_check(self): return self.legend_panel.husky_waypoint_check
    @property
    def husky_home_check(self): return self.legend_panel.husky_home_check
    @property
    def hazard_status_check(self): return self.legend_panel.hazard_status_check
    @property
    def detected_check(self): return self.legend_panel.detected_check
    @property
    def husky_required_check(self): return self.legend_panel.husky_required_check
    @property
    def in_progress_check(self): return self.legend_panel.in_progress_check
    @property
    def resolved_check(self): return self.legend_panel.resolved_check
    @property
    def confirmed_check(self): return self.legend_panel.confirmed_check
    @property
    def hazard_layer_check(self): return self.legend_panel.hazard_layer_check
    @property
    def slippery_ice_check(self): return self.legend_panel.slippery_ice_check
    @property
    def low_snow_check(self): return self.legend_panel.low_snow_check
    @property
    def wood_debris_check(self): return self.legend_panel.wood_debris_check
    @property
    def stone_debris_check(self): return self.legend_panel.stone_debris_check
