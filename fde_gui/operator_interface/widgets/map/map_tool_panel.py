from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

from ...paths import ASSETS_DIR

class MapToolPanel(QFrame):
    """
    Floating map editing toolbar.

    Modes:
        select
        add_drone
        add_husky
        remove
    """

    mode_changed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setObjectName("mapToolPanel")

        self.current_mode = "select"

        # fde_gui/assets
        self.assets_dir = ASSETS_DIR

        # =========================================================
        # MAIN LAYOUT
        # =========================================================

        layout = QVBoxLayout(self)

        layout.setContentsMargins(
            5,
            5,
            5,
            5,
        )

        layout.setSpacing(3)

        # =========================================================
        # BUTTON GROUP
        # =========================================================

        self.button_group = QButtonGroup(self)
        self.button_group.setExclusive(True)

        # =========================================================
        # SELECT
        # =========================================================

        self.select_button = self._create_tool_button(
            icon_path="map_tool_icons/select.svg",
            label_text="Select",
            mode="select",
        )

        layout.addWidget(self.select_button)

        # =========================================================
        # ADD DRONE WAYPOINT
        # =========================================================

        self.drone_button = self._create_tool_button(
            icon_path="map_tool_icons/drone_waypoint.svg",
            label_text="Add Drone WP",
            mode="add_drone",
        )

        layout.addWidget(self.drone_button)

        # =========================================================
        # ADD HUSKY WAYPOINT
        # =========================================================

        self.husky_button = self._create_tool_button(
            icon_path="map_tool_icons/husky_waypoint.svg",
            label_text="Add Husky WP",
            mode="add_husky",
        )

        layout.addWidget(self.husky_button)

        # =========================================================
        # DELETE WAYPOINT
        # =========================================================

        self.remove_button = self._create_tool_button(
            icon_path="map_tool_icons/delete_waypoint.svg",
            label_text="Remove WP",
            mode="remove",
        )

        layout.addWidget(self.remove_button)

        # Default mode
        self.select_button.setChecked(True)

    # =============================================================
    # CREATE TOOL BUTTON
    # =============================================================

    def _create_tool_button(
        self,
        icon_path,
        label_text,
        mode,
    ):
        button = QPushButton()

        button.setObjectName("mapToolButton")
        button.setCheckable(True)

        button.setProperty(
            "toolMode",
            mode,
        )

        button.setToolTip(
            self._tool_tip(mode)
        )

        # Make each row fill the available toolbar width.
        button.setMinimumHeight(25)

        # =========================================================
        # BUTTON LAYOUT
        # =========================================================

        button_layout = QHBoxLayout(button)

        button_layout.setContentsMargins(
            7,
            1,
            7,
            1,
        )

        button_layout.setSpacing(7)

        # =========================================================
        # ICON
        # =========================================================

        icon = QLabel()
        icon.setObjectName("mapToolImage")

        icon.setFixedSize(
            20,
            20,
        )

        icon.setAlignment(
            Qt.AlignCenter
        )

        full_icon_path = self.assets_dir / icon_path

        # File does not exist
        if not full_icon_path.exists():
            print(
                f"[MapToolPanel] Missing asset: {full_icon_path}"
            )
            icon.setText("?")

        else:
            pixmap = QPixmap(str(full_icon_path))

            # File exists but Qt could not load it
            if pixmap.isNull():
                print(
                    f"[MapToolPanel] Failed to load asset: {full_icon_path}"
                )
                icon.setText("?")

            else:
                pixmap = pixmap.scaled(
                    16,
                    16,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )

                icon.setPixmap(pixmap)

        icon.setAttribute(
            Qt.WA_TransparentForMouseEvents,
            True,
        )

        # =========================================================
        # LABEL
        # =========================================================

        label = QLabel(label_text)

        label.setObjectName(
            "mapToolLabel"
        )

        label.setAlignment(
            Qt.AlignLeft | Qt.AlignVCenter
        )

        label.setAttribute(
            Qt.WA_TransparentForMouseEvents,
            True,
        )

        # =========================================================
        # ADD CONTENT
        # =========================================================

        button_layout.addWidget(icon)

        button_layout.addWidget(
            label,
            1,
        )

        # =========================================================
        # BUTTON GROUP
        # =========================================================

        self.button_group.addButton(
            button
        )

        button.clicked.connect(
            lambda checked,
            selected_mode=mode:
                self._set_mode(
                    selected_mode,
                    checked,
                )
        )

        return button

    # =============================================================
    # MODE CHANGE
    # =============================================================

    def _set_mode(
        self,
        mode,
        checked,
    ):
        if not checked:
            return

        self.current_mode = mode

        self.mode_changed.emit(
            mode
        )

    # =============================================================
    # PROGRAMMATIC MODE
    # =============================================================

    def set_mode(
        self,
        mode,
    ):
        buttons = {
            "select": self.select_button,
            "add_drone": self.drone_button,
            "add_husky": self.husky_button,
            "remove": self.remove_button,
        }

        button = buttons.get(mode)

        if button is None:
            return

        button.setChecked(True)

        self.current_mode = mode

        self.mode_changed.emit(
            mode
        )

    def get_mode(self):
        return self.current_mode

    def reset_to_select(self):
        self.set_mode(
            "select"
        )

    # =============================================================
    # TOOLTIPS
    # =============================================================

    def _tool_tip(
        self,
        mode,
    ):
        tooltips = {
            "select":
                "Select hazards and map objects.",

            "add_drone":
                "Click the map to add a Drone waypoint.",

            "add_husky":
                "Click the map to add a Husky waypoint.",

            "remove":
                "Click a manual waypoint to remove it.",
        }

        return tooltips.get(
            mode,
            "",
        )