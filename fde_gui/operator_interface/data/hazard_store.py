from PySide6.QtCore import QObject, Signal


class HazardStore(QObject):
    """
    Central hazard state shared by the operator interface.

    MissionDatabase handles persistent SQLite storage.
    HazardStore provides the GUI-friendly hazard model.
    """

    hazards_changed = Signal()
    hazard_changed = Signal(str)

    def __init__(
        self,
        database,
        parent=None,
    ):
        super().__init__(parent)

        self.database = database
        self.hazards = {}

        self.reload()

    # =============================================================
    # LOAD DATABASE
    # =============================================================

    def reload(self):

        self.hazards.clear()

        rows = self.database.get_hazards()

        for row in rows:

            hazard_id = row["hazard_id"]

            husky_mission = (
                self.database.get_husky_mission(
                    hazard_id
                )
            )

            evidence_rows = (
                self.database.get_evidence(
                    hazard_id
                )
            )

            # -----------------------------------------------------
            # HUSKY STATE
            # -----------------------------------------------------

            if husky_mission is None:

                husky_assignment = "Not Assigned"
                assigned_time = "—"
                resolved_by = "—"
                resolved_time = "—"

            else:

                husky_assignment = (
                    husky_mission["status"]
                )

                assigned_time = (
                    husky_mission["assigned_at"]
                    or "—"
                )

                if husky_mission["completed_at"]:

                    resolved_by = "Husky"

                    resolved_time = (
                        husky_mission["completed_at"]
                    )

                else:

                    resolved_by = "—"
                    resolved_time = "—"

            # -----------------------------------------------------
            # EVIDENCE
            # -----------------------------------------------------

            evidence = []

            for evidence_row in evidence_rows:

                evidence.append(
                    {
                        "type":
                            evidence_row[
                                "evidence_type"
                            ],

                        "file_path":
                            evidence_row[
                                "file_path"
                            ],

                        "captured_at":
                            evidence_row[
                                "captured_at"
                            ],
                    }
                )

            # -----------------------------------------------------
            # GUI MODEL
            # -----------------------------------------------------

            self.hazards[hazard_id] = {

                "type":
                    row["type"],

                "status":
                    row["status"],

                "coords": (
                    row["x"],
                    row["y"],
                    row["z"],
                ),

                "detected_by":
                    row["detected_by"] or "—",

                "time_detected":
                    row["time_detected"] or "—",

                "detection_confidence":
                    row["detection_confidence"],

                "husky_solvable":
                    row["husky_solvable"],

                # Temporary compatibility if another widget
                # still expects hazard["confidence"].
                "confidence":
                    row["detection_confidence"],

                "husky_assignment":
                    husky_assignment,

                "assigned_time":
                    assigned_time,

                "resolved_by":
                    resolved_by,

                "resolved_time":
                    resolved_time,

                "notes":
                    row["notes"] or "",

                "evidence":
                    evidence,
            }

        self.hazards_changed.emit()

    # =============================================================
    # GETTERS
    # =============================================================

    def get_all(self):
        return self.hazards

    def get(self, hazard_id):
        return self.hazards.get(hazard_id)

    def exists(self, hazard_id):
        return hazard_id in self.hazards

    # =============================================================
    # SEND HUSKY
    # =============================================================

    def send_husky(
        self,
        hazard_id,
    ):

        hazard = self.get(hazard_id)

        if hazard is None:
            return

        existing_mission = (
            self.database.get_husky_mission(
                hazard_id
            )
        )

        if existing_mission is None:

            self.database.create_husky_mission(
                hazard_id
            )

        self.database.update_hazard_status(
            hazard_id,
            "Husky Required",
        )

        self.reload()

        self.hazard_changed.emit(
            hazard_id
        )

    # =============================================================
    # CANCEL HUSKY
    # =============================================================

    def cancel_husky(
        self,
        hazard_id,
    ):

        if not self.exists(hazard_id):
            return

        self.database.cancel_husky_mission(
            hazard_id
        )

        self.database.update_hazard_status(
            hazard_id,
            "Inconclusive",
        )

        self.reload()

        self.hazard_changed.emit(
            hazard_id
        )

    # =============================================================
    # REOPEN HAZARD
    # =============================================================

    def reopen(
        self,
        hazard_id,
    ):

        if not self.exists(hazard_id):
            return

        self.database.cancel_husky_mission(
            hazard_id
        )

        self.database.update_hazard_status(
            hazard_id,
            "Inconclusive",
        )

        self.reload()

        self.hazard_changed.emit(
            hazard_id
        )

    # =============================================================
    # UPDATE NOTES
    # =============================================================

    def update_notes(
        self,
        hazard_id,
        notes,
    ):

        if not self.exists(hazard_id):
            return

        self.database.update_hazard_notes(
            hazard_id,
            notes,
        )

        self.hazards[
            hazard_id
        ]["notes"] = notes

        self.hazard_changed.emit(
            hazard_id
        )

    # =============================================================
    # CONFIRM SAFE
    # =============================================================

    def confirm_safe(
        self,
        hazard_id,
    ):

        if not self.exists(hazard_id):
            return

        self.database.update_hazard_status(
            hazard_id,
            "Resolved / Safe",
        )

        self.reload()

        self.hazard_changed.emit(
            hazard_id
        )


    # =============================================================
    # CONFIRM HAZARD
    # =============================================================

    def confirm_hazard(
        self,
        hazard_id,
    ):

        if not self.exists(hazard_id):
            return

        self.database.update_hazard_status(
            hazard_id,
            "Confirmed Hazard",
        )

        self.reload()

        self.hazard_changed.emit(
            hazard_id
        )

    # =============================================================
    # CLEAR ALL
    # =============================================================

    def clear_all(self):
        """Clear all hazards in database and local cache."""
        self.database.clear_all()
        self.hazards.clear()
        self.hazards_changed.emit()