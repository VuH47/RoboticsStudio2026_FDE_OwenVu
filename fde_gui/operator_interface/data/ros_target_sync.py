import math


STATE_TO_STATUS = {
    "tracking": "Inconclusive",
    "ready": "Husky Required",
    "active": "In Progress",
    "confirmed": "Confirmed Hazard",
    "not_confirmed": "Resolved / Safe",
    "inconclusive": "Inconclusive",
    "missed": "Inconclusive",
}


class RosTargetSync:
    """Synchronize ROS ice targets with the GUI database."""

    def __init__(self, hazard_store, run_id):
        self.store = hazard_store
        self.database = hazard_store.database
        self.run_id = run_id

    def process(self, data):
        if not isinstance(data, dict):
            return

        targets = data.get("targets", [])

        if not isinstance(targets, list):
            return

        changed = False

        for target in targets:
            if not isinstance(target, dict):
                continue

            target_id = str(target.get("id", "")).strip()

            if not (
                target_id.startswith("T")
                and target_id[1:].isdigit()
            ):
                continue

            try:
                x = float(target["x"])
                y = float(target["y"])
                z = float(target["z"])
            except (KeyError, TypeError, ValueError):
                continue

            if not all(math.isfinite(v) for v in (x, y, z)):
                continue

            state = str(
                target.get("state", "tracking")
            ).strip().lower()

            if state not in STATE_TO_STATUS:
                continue

            hazard_id = f"H-ROS-{self.run_id}-{target_id}"
            status = STATE_TO_STATUS[state]

            existing = self.database.get_hazard(hazard_id)

            if existing is None:
                self.database.create_hazard(
                    hazard_id=hazard_id,
                    hazard_type="Ice",
                    status=status,
                    x=x,
                    y=y,
                    z=z,
                    detection_confidence=None,
                    husky_solvable=None,
                    detected_by="Drone",
                    notes=f"ROS target: {target_id}",
                )
                changed = True

            elif (
                existing["status"] != status
                or (
                    existing["x"],
                    existing["y"],
                    existing["z"],
                ) != (x, y, z)
            ):
                self.database.update_hazard_detection(
                    hazard_id, status, x, y, z
                )
                changed = True

        if changed:
            self.store.reload()