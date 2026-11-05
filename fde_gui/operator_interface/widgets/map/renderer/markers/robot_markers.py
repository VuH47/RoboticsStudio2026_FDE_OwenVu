
import math

from .....paths import ASSETS_DIR


class RobotMarkers:
    """
    Manages Drone and Husky SVG markers.

    MarkerOverlay handles:
    - SVG rendering
    - World-to-screen projection
    - Fixed pixel sizing
    - Visibility and positioning
    """

    def __init__(self, marker_overlay):
        self.overlay = marker_overlay

        # =====================================================
        # DRONE MARKERS
        # =====================================================

        self.overlay.add_marker(
            marker_id="drone_location",
            svg_path=(
                ASSETS_DIR
                / "drone_icons"
                / "drone_location.svg"
            ),
            size=34,
        )

        self.overlay.add_marker(
            marker_id="drone_home",
            svg_path=(
                ASSETS_DIR
                / "drone_icons"
                / "drone_home.svg"
            ),
            size=34,
        )

        # =====================================================
        # HUSKY MARKERS
        # =====================================================

        self.overlay.add_marker(
            marker_id="husky_location",
            svg_path=(
                ASSETS_DIR
                / "husky_icons"
                / "husky_location.svg"
            ),
            size=34,
        )

        self.overlay.add_marker(
            marker_id="husky_home",
            svg_path=(
                ASSETS_DIR
                / "husky_icons"
                / "husky_home.svg"
            ),
            size=34,
        )

    # =========================================================
    # QUATERNION TO YAW
    # =====================================================

    @staticmethod
    def quaternion_to_yaw(qx, qy, qz, qw):
        """Convert ROS quaternion to yaw in degrees."""

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        return math.degrees(yaw)

    # =========================================================
    # DRONE LOCATION
    # =====================================================

    def update_drone_location(
        self, x, y, z, qx, qy, qz, qw
    ):
        """Update Drone Location using ROS odometry."""

        self.overlay.set_position(
            "drone_location",
            x,
            y,
            z + 1.0,
        )

        self.overlay.set_heading(
            "drone_location",
            -self.quaternion_to_yaw(
                qx, qy, qz, qw
            ),
        )

    # =========================================================
    # DRONE HOME
    # =====================================================

    def set_drone_home(self, x, y, z=0.0):
        """Set the Drone Home world coordinates."""

        self.overlay.set_position(
            "drone_home",
            x,
            y,
            z + 1.0,
        )

    # =========================================================
    # HUSKY LOCATION
    # =====================================================

    def update_husky_location(
        self, x, y, z, qx, qy, qz, qw
    ):
        """Update Husky Location using ROS odometry."""

        self.overlay.set_position(
            "husky_location",
            x,
            y,
            z + 1.0,
        )

        self.overlay.set_heading(
            "husky_location",
            -self.quaternion_to_yaw(
                qx, qy, qz, qw
            ),
        )

    # =========================================================
    # HUSKY HOME
    # =====================================================

    def set_husky_home(self, x, y, z=0.0):
        """Set the Husky Home world coordinates."""

        self.overlay.set_position(
            "husky_home",
            x,
            y,
            z + 1.0,
        )

    # =========================================================
    # LAYER VISIBILITY
    # =====================================================

    def set_layer_visible(self, layer_name, visible):
        """Control individual robot marker visibility."""

        if layer_name in (
            "drone_location",
            "drone_home",
            "husky_location",
            "husky_home",
        ):
            self.overlay.set_visible(
                layer_name,
                visible,
            )
