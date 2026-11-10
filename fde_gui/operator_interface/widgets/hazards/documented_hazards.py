from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QHBoxLayout,
    QWidget,
)

from operator_interface.widgets.hazards.hazard_list import (
    HazardList,
)

from operator_interface.widgets.hazards.hazard_details import (
    HazardDetails,
)


class DocumentedHazards(QWidget):
    """
    Complete Documented Hazards interface.

    HazardStore owns the data.
    This widget owns the presentation and user interaction.
    """

    hazard_selected = Signal(str)

    def __init__(
        self,
        hazard_store,
        status_colours,
        waypoint_store=None,
        parent=None,
    ):
        super().__init__(parent)

        self.hazard_store = hazard_store
        self.status_colours = status_colours

        self.waypoint_store = waypoint_store

        self.hazards = (
            self.hazard_store.get_all()
        )

        self._sync_husky_missions()

        self.selected_hazard_id = None

        self._build_ui()

        self.hazard_store.hazards_changed.connect(
            self.refresh
        )

    # =============================================================
    # BUILD UI
    # =============================================================

    def _build_ui(self):

        main = QHBoxLayout(self)

        main.setContentsMargins(
            3,
            3,
            3,
            3,
        )

        main.setSpacing(8)

        # ---------------------------------------------------------
        # HAZARD LIST
        # ---------------------------------------------------------

        self.hazard_list = HazardList(
            hazards=self.hazards,
            selection_callback=self.select_hazard,
        )

        # ---------------------------------------------------------
        # HAZARD DETAILS
        # ---------------------------------------------------------

        self.hazard_details = HazardDetails(
            status_colours=self.status_colours,
            confirm_safe_callback=self.confirm_safe,
            confirm_hazard_callback=self.confirm_hazard,
            primary_action_callback=self.handle_primary_action,
            secondary_action_callback=self.dismiss_hazard,
        )

        main.addWidget(
            self.hazard_list,
            2,
        )

        main.addWidget(
            self.hazard_details,
            4,
        )

    # =============================================================
    # SELECT
    # =============================================================

    def select_hazard(
        self,
        hazard_id,
    ):

        hazard = self.hazard_store.get(
            hazard_id
        )

        if hazard is None:
            return

        self.selected_hazard_id = (
            hazard_id
        )

        self.hazard_details.show_hazard(
            hazard_id,
            hazard,
        )

        self.hazard_selected.emit(
            hazard_id
        )

    # =============================================================
    # SHOW HAZARD
    # =============================================================

    def show_hazard(
        self,
        hazard_id,
    ):

        self.select_hazard(
            hazard_id
        )

    # =============================================================
    # PRIMARY ACTION
    # =============================================================

    def handle_primary_action(self):

        hazard_id = (
            self.selected_hazard_id
        )

        if hazard_id is None:
            return

        hazard = self.hazard_store.get(
            hazard_id
        )

        if hazard is None:
            return

        status = hazard["status"]

        # ---------------------------------------------------------
        # SEND TO HUSKY
        # ---------------------------------------------------------

        if status == "Inconclusive":

            # Update hazard state first.
            self.hazard_store.send_husky(
                hazard_id
            )

            # Add the hazard to the Husky waypoint queue.
            if self.waypoint_store is not None:

                x, y, z = hazard.get(
                    "coords",
                    (0.0, 0.0, 0.0),
                )

                self.waypoint_store.add_hazard_mission(
                    hazard_id=hazard_id,
                    hazard_type=hazard.get(
                        "type",
                        "Hazard",
                    ),
                    x=x,
                    y=y,
                    z=z,
                )

        # ---------------------------------------------------------
        # CANCEL HUSKY
        # ---------------------------------------------------------

        elif status == "Husky Required":

            self.hazard_store.cancel_husky(
                hazard_id
            )

            if self.waypoint_store is not None:
                self.waypoint_store.remove_hazard_mission(
                    hazard_id
                )

        # ---------------------------------------------------------
        # IN PROGRESS
        #
        # Don't alter it here. OperatorGUI can later use another
        # signal if we want "View Live Husky" to change tabs.
        # ---------------------------------------------------------

        elif status == "In Progress":

            return

        # ---------------------------------------------------------
        # REOPEN
        # ---------------------------------------------------------

        else:

            self.hazard_store.reopen(
                hazard_id
            )

        self.refresh()

        self.select_hazard(
            hazard_id
        )

    # =============================================================
    # DISMISS
    # =============================================================

    def dismiss_hazard(self):

        # At the moment dismissal does not delete a documented
        # hazard. It simply clears the selection.
        self.selected_hazard_id = None

    # =============================================================
    # REFRESH
    # =============================================================

    def refresh(self):

        self.hazards = (
            self.hazard_store.get_all()
        )

        self._sync_husky_missions()

        # HazardList holds the same dictionary object currently,
        # but explicitly update it to keep this robust.
        self.hazard_list.hazards = (
            self.hazards
        )

        self.hazard_list.refresh()

        if (
            self.selected_hazard_id
            and self.hazard_store.exists(
                self.selected_hazard_id
            )
        ):

            hazard = self.hazard_store.get(
                self.selected_hazard_id
            )

            self.hazard_details.show_hazard(
                self.selected_hazard_id,
                hazard,
            )

    # =============================================================
    # CONFIRM SAFE
    # =============================================================

    def confirm_safe(self):

        hazard_id = self.selected_hazard_id

        if hazard_id is None:
            return

        self.hazard_store.confirm_safe(
            hazard_id
        )

        self.refresh()

        self.select_hazard(
            hazard_id
        )


    # =============================================================
    # CONFIRM HAZARD
    # =============================================================

    def confirm_hazard(self):

        hazard_id = self.selected_hazard_id

        if hazard_id is None:
            return

        self.hazard_store.confirm_hazard(
            hazard_id
        )

        self.refresh()

        self.select_hazard(
            hazard_id
        )

    def _sync_husky_missions(self):

        if self.waypoint_store is None:
            return

        for hazard_id, hazard in self.hazards.items():

            if hazard.get("status") != "Husky Required":
                continue

            coords = hazard.get("coords")

            if not coords:
                continue

            x, y, z = coords

            self.waypoint_store.add_hazard_mission(
                hazard_id=hazard_id,
                hazard_type=hazard.get(
                    "type",
                    "Hazard",
                ),
                x=x,
                y=y,
                z=z,
            )