import math
import pyvista as pv


class RobotMapLayers:
    """Dynamic robot markers displayed over the static Gazebo world."""

    def __init__(self, plotter):
        self.plotter = plotter
        self.actors = {}

        self.add_robot("husky1", "#ff9800", 1.4)
        self.add_robot("parrot1", "#00bcd4", 1.8)

    def add_robot(self, name, color, size):
        # Arrow points along the robot's positive X axis.
        mesh = pv.Arrow(
            start=(0, 0, 0),
            direction=(1, 0, 0),
            tip_length=0.35,
            tip_radius=0.18,
            shaft_radius=0.10,
            scale=size,
        )

        actor = self.plotter.add_mesh(
            mesh,
            color=color,
            lighting=False,
            name=f"robot_{name}",
            reset_camera=False,
        )

        actor.SetVisibility(False)
        self.actors[name] = actor

    def update_robot(self, name, x, y, z, qx, qy, qz, qw):
        actor = self.actors.get(name)

        if actor is None:
            return

        # Extract heading (yaw) from the orientation quaternion.
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        actor.SetOrientation(0, 0, math.degrees(yaw))

        # Raise marker slightly above the terrain for visibility.
        actor.SetPosition(x, y, z + 1.0)
        actor.SetVisibility(True)