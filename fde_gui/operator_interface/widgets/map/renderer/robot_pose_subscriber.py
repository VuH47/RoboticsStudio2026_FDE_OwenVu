import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class RobotPoseSubscriber(Node):
    """Receive live robot odometry without touching the Qt renderer."""

    def __init__(self):
        super().__init__("mission_map_robot_tracking")

        self.latest_poses = {}

        self.create_subscription(
            Odometry,
            "/husky1/odometry",
            lambda msg: self.on_odometry("husky1", msg),
            10,
        )

        self.create_subscription(
            Odometry,
            "/parrot1/odometry",
            lambda msg: self.on_odometry("parrot1", msg),
            10,
        )

    def on_odometry(self, robot_name, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.latest_poses[robot_name] = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        