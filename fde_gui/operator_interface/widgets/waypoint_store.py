from PySide6.QtCore import QObject, Signal


class WaypointStore(QObject):
    """
    Single source of truth for Drone and Husky waypoints.

    Waypoints may originate from:
        - Manual operator input
        - Hazard / mission assignment

    The map, waypoint panel and waypoint manager should all
    read/write waypoint data through this object.
    """

    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.drone_waypoints = []
        self.husky_waypoints = []

        self._next_drone_manual_id = 1
        self._next_husky_manual_id = 1

    # =========================================================
    # GET WAYPOINTS
    # =========================================================

    def get_waypoints(self, robot):
        if robot.lower() == "drone":
            return list(self.drone_waypoints)

        if robot.lower() == "husky":
            return list(self.husky_waypoints)

        return []

    def get_drone_waypoints(self):
        return list(self.drone_waypoints)

    def get_husky_waypoints(self):
        return list(self.husky_waypoints)

    # =========================================================
    # ADD MANUAL WAYPOINT
    # =========================================================

    def add_manual_waypoint(
        self,
        robot,
        x,
        y,
        z=0.0,
        description="Operator Waypoint",
    ):
        robot = robot.lower()

        if robot == "drone":
            waypoint_id = (
                f"WP-{self._next_drone_manual_id:02d}"
            )

            self._next_drone_manual_id += 1

        elif robot == "husky":
            waypoint_id = (
                f"WP-{self._next_husky_manual_id:02d}"
            )

            self._next_husky_manual_id += 1

        else:
            return None

        waypoint = {
            "id": waypoint_id,
            "type": "Manual",
            "source": "manual",
            "robot": robot,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "description": description,
            "can_delete": True,
            "hazard_id": None,
        }

        self._queue(robot).append(waypoint)

        self.changed.emit()

        return waypoint

    # =========================================================
    # ADD HAZARD MISSION
    # =========================================================

    def add_hazard_mission(
        self,
        hazard_id,
        hazard_type,
        x,
        y,
        z=0.0,
    ):
        # Prevent the same hazard from being queued twice.
        for waypoint in self.husky_waypoints:
            if waypoint.get("hazard_id") == hazard_id:
                return waypoint

        waypoint = {
            "id": hazard_id,
            "type": "Mission",
            "source": "hazard",
            "robot": "husky",
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "description": f"Inspect {hazard_type}",
            "can_delete": False,
            "hazard_id": hazard_id,
        }

        self.husky_waypoints.append(waypoint)

        self.changed.emit()

        return waypoint

    # =========================================================
    # DELETE
    # =========================================================

    def delete_waypoint(
        self,
        robot,
        index,
    ):
        queue = self._queue(robot)

        if not 0 <= index < len(queue):
            return False

        waypoint = queue[index]

        # Hazard-generated missions cannot be manually deleted.
        if not waypoint.get(
            "can_delete",
            True,
        ):
            return False

        del queue[index]

        self.changed.emit()

        return True

    # =========================================================
    # MOVE UP
    # =========================================================

    def move_up(
        self,
        robot,
        index,
    ):
        queue = self._queue(robot)

        if not 0 < index < len(queue):
            return False

        queue[index - 1], queue[index] = (
            queue[index],
            queue[index - 1],
        )

        self.changed.emit()

        return True

    # =========================================================
    # MOVE DOWN
    # =========================================================

    def move_down(
        self,
        robot,
        index,
    ):
        queue = self._queue(robot)

        if not 0 <= index < len(queue) - 1:
            return False

        queue[index], queue[index + 1] = (
            queue[index + 1],
            queue[index],
        )

        self.changed.emit()

        return True

    # =========================================================
    # CLEAR MANUAL
    # =========================================================

    def clear_manual(self):
        self.drone_waypoints = [
            waypoint
            for waypoint in self.drone_waypoints
            if waypoint["type"] != "Manual"
        ]

        self.husky_waypoints = [
            waypoint
            for waypoint in self.husky_waypoints
            if waypoint["type"] != "Manual"
        ]

        self.changed.emit()

    # =========================================================
    # REMOVE HAZARD MISSION
    # =========================================================

    def remove_hazard_mission(
        self,
        hazard_id,
    ):
        original_length = len(
            self.husky_waypoints
        )

        self.husky_waypoints = [
            waypoint
            for waypoint in self.husky_waypoints
            if waypoint.get("hazard_id") != hazard_id
        ]

        if (
            len(self.husky_waypoints)
            != original_length
        ):
            self.changed.emit()
            return True

        return False

    # =========================================================
    # INTERNAL
    # =========================================================

    def _queue(self, robot):
        robot = robot.lower()

        if robot == "drone":
            return self.drone_waypoints

        if robot == "husky":
            return self.husky_waypoints

        raise ValueError(
            f"Unknown robot: {robot}"
        )