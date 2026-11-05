from PySide6.QtCore import QObject, QEvent, Qt
from PySide6.QtWidgets import QWidget

from .....paths import ASSETS_DIR
from .numbered_waypoint import NumberedWaypoint


# =============================================================
# WAYPOINT CLICK FILTER
# =============================================================

class WaypointClickFilter(QObject):
    """
    Detects left-clicks on hazard-linked waypoint markers.

    The callback determines whether the click should be handled.
    OperatorGUI only accepts hazard selection in Select mode.
    """

    def __init__(self, callback, hazard_id, parent=None):
        super().__init__(parent)

        self.callback = callback
        self.hazard_id = hazard_id

    def eventFilter(self, watched, event):
        if (
            event.type() == QEvent.MouseButtonPress
            and event.button() == Qt.LeftButton
        ):
            print(
                f"[WAYPOINT FILTER] Click detected: "
                f"hazard_id={self.hazard_id}",
                flush=True,
            )

            if self.callback is not None:
                handled = self.callback(self.hazard_id)

                if handled:
                    event.accept()
                    return True

        return False


# =============================================================
# WAYPOINT MARKERS
# =============================================================

class WaypointMarkers:
    """
    Displays Drone and Husky waypoints on the 3D map.

    Features:
    - Numbers correspond to waypoint queue positions.
    - Drone and Husky numbering starts independently at 1.
    - Manual and hazard-generated waypoints use the same numbering.
    - Hazard-linked waypoints can open the Selected Hazard panel.
    - Manual waypoints do not trigger hazard selection.
    - MarkerOverlay handles world-to-screen positioning.
    - WaypointStore determines waypoint order.
    """

    def __init__(self, marker_overlay):

        self.overlay = marker_overlay

        # Active SVG marker IDs
        self.marker_ids = set()

        # Number labels associated with SVG markers
        self.number_labels = {}

        # Active click filters and the widgets they monitor
        self.click_filters = []

        # Assigned by OperatorGUI
        self.on_hazard_waypoint_clicked = None

        # ---------------------------------------------------------
        # SVG ASSETS
        # ---------------------------------------------------------

        self.svg_paths = {
            "drone": (
                ASSETS_DIR
                / "drone_icons"
                / "drone_waypoint.svg"
            ),
            "husky": (
                ASSETS_DIR
                / "husky_icons"
                / "husky_waypoint.svg"
            ),
        }

        # ---------------------------------------------------------
        # LAYER VISIBILITY
        # ---------------------------------------------------------

        self.layer_visibility = {
            "drone": True,
            "husky": True,
        }

    # =============================================================
    # REFRESH WAYPOINTS
    # =============================================================

    def refresh(self, drone_waypoints, husky_waypoints):
        """
        Rebuild map markers using the current waypoint queues.

        Reordering, adding or deleting waypoints automatically
        updates their displayed numbers.
        """

        self.clear()

        self._add_waypoints(
            robot="drone",
            waypoints=drone_waypoints,
        )

        self._add_waypoints(
            robot="husky",
            waypoints=husky_waypoints,
        )

        self.overlay.refresh()
        self.update_label_positions()

    # =============================================================
    # HANDLE HAZARD WAYPOINT CLICKS
    # =============================================================


    def _handle_waypoint_click(self, hazard_id):
        print(f"[WAYPOINT CLICK] Hazard ID: {hazard_id}", flush=True)

        if not hazard_id:
            return False

        if self.on_hazard_waypoint_clicked is None:
            print("[WAYPOINT CLICK] No callback connected", flush=True)
            return False

        result = self.on_hazard_waypoint_clicked(hazard_id)
        print(f"[WAYPOINT CLICK] Callback result: {result}", flush=True)

        return bool(result)


    # =============================================================
    # ADD WAYPOINTS
    # =============================================================

    def _add_waypoints(self, robot, waypoints):
        """
        Create SVG markers and numbered labels.

        Waypoints are numbered according to their current
        position in their respective robot's waypoint list.
        """

        for number, waypoint in enumerate(waypoints, start=1):

            marker_id = f"{robot}_waypoint_{number}"

            # -----------------------------------------------------
            # CREATE SVG ICON
            # -----------------------------------------------------

            icon = self.overlay.add_marker(
                marker_id=marker_id,
                svg_path=self.svg_paths[robot],
                size=30,
                visible=self.layer_visibility[robot],
            )

            # -----------------------------------------------------
            # SET WORLD POSITION
            # -----------------------------------------------------

            self.overlay.set_position(
                marker_id,
                waypoint["x"],
                waypoint["y"],
                waypoint.get("z", 0.0) + 1.0,
            )

            self.marker_ids.add(marker_id)

            # -----------------------------------------------------
            # CREATE NUMBER LABEL
            # -----------------------------------------------------

            label = NumberedWaypoint(
                number=number,
                robot=robot,
                parent=self.overlay.viewport,
            )

            self.number_labels[marker_id] = label

            # -----------------------------------------------------
            # LINK HAZARD WAYPOINTS TO HAZARD RECORDS
            # -----------------------------------------------------

            hazard_id = waypoint.get("hazard_id")
            print(
                f"[WAYPOINT CREATED] robot={robot}, "
                f"number={number}, "
                f"hazard_id={hazard_id}",
                flush=True,
            )

            # Manual waypoints have no associated hazard.
            if not hazard_id:
                continue

            click_filter = WaypointClickFilter(
                callback=self._handle_waypoint_click,
                hazard_id=hazard_id,
                parent=self.overlay.viewport,
            )

            # Install on SVG icon and number label.
            widgets = [icon, label]

            # Include child widgets, such as QLabel.
            widgets.extend(
                label.findChildren(QWidget)
            )

            # Remove duplicates while preserving order.
            widgets = list(dict.fromkeys(widgets))

            for widget in widgets:
                widget.setAttribute(
                    Qt.WA_TransparentForMouseEvents,
                    False,
                )

                widget.installEventFilter(click_filter)
                widget.setCursor(Qt.PointingHandCursor)

                print(
                    f"[CLICK FILTER INSTALLED] "
                    f"hazard={hazard_id}, "
                    f"widget={type(widget).__name__}, "
                    f"visible={widget.isVisible()}, "
                    f"transparent={widget.testAttribute(Qt.WA_TransparentForMouseEvents)}",
                    flush=True,
                )

            # Keep the filter and monitored widgets alive.
            self.click_filters.append(
                (click_filter, widgets)
            )

    # =============================================================
    # UPDATE LABEL POSITIONS
    # =============================================================

    def update_label_positions(self):
        """
        Keep waypoint numbers aligned with their SVG icons
        when the camera moves, zooms or the viewport resizes.
        """

        for marker_id, label in self.number_labels.items():

            marker = self.overlay.markers.get(marker_id)

            if marker is None:
                label.hide()
                continue

            icon = marker["widget"]

            if not icon.isVisible():
                label.hide()
                continue

            label.move(
                icon.x() + icon.width() - 5,
                icon.y() - 8,
            )

            label.show()

    # =============================================================
    # WAYPOINT LAYER VISIBILITY
    # =============================================================

    def set_layer_visible(self, robot, visible):
        """
        Show or hide waypoint SVG icons and their number labels.
        """

        if robot not in self.layer_visibility:
            return

        self.layer_visibility[robot] = bool(visible)

        for marker_id in self.marker_ids:

            if not marker_id.startswith(
                f"{robot}_waypoint_"
            ):
                continue

            self.overlay.set_visible(
                marker_id,
                visible,
            )

        self.update_label_positions()

    # =============================================================
    # CLEAR WAYPOINTS
    # =============================================================

    def clear(self):
        """
        Remove existing waypoint icons, number labels
        and their associated click filters.
        """

        # ---------------------------------------------------------
        # REMOVE CLICK FILTERS
        # ---------------------------------------------------------

        for click_filter, widgets in self.click_filters:

            for widget in widgets:
                try:
                    widget.removeEventFilter(click_filter)
                except RuntimeError:
                    pass

            click_filter.deleteLater()

        self.click_filters.clear()

        # ---------------------------------------------------------
        # REMOVE NUMBER LABELS
        # ---------------------------------------------------------

        for label in self.number_labels.values():
            label.hide()
            label.deleteLater()

        self.number_labels.clear()

        # ---------------------------------------------------------
        # REMOVE SVG MARKERS
        # ---------------------------------------------------------

        for marker_id in self.marker_ids:
            self.overlay.remove_marker(marker_id)

        self.marker_ids.clear()