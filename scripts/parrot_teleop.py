#!/usr/bin/env python3
"""Interactive 3D drone teleoperation for Parrot drone.

Controls:
    w / s : Forward / Backward (linear.x)
    a / d : Strafe Left / Right (linear.y)
    q / e : Yaw Left / Yaw Right (angular.z)
    Space : Ascend / Climb (+linear.z)
    c     : Descend (-linear.z)
    x / k : Stop / Hover (zero velocities)
    + / - : Increase / Decrease linear speed
    r     : Reset target height to default (1.5m)
    Ctrl+C: Quit
"""

from __future__ import annotations

import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

USAGE = """
------------------------------------------------------
 Parrot 3D Drone Teleop
------------------------------------------------------
 Planar Movement:             Altitude & Heading:
      [w]                          [Space] : Ascend (Up)
  [a] [s] [d]                      [c]     : Descend (Down)
                                   [q]     : Yaw Left
  w / s : Forward / Back           [e]     : Yaw Right
  a / d : Strafe Left / Right
  
  [x] or [k] : Stop / Hover in place
  [r]        : Reset hover height
  [+] / [-]  : Increase / decrease speeds
  [Ctrl+C]   : Quit
------------------------------------------------------
"""


class ParrotTeleop(Node):
    def __init__(self, topic: str):
        super().__init__('parrot_teleop')
        self.pub = self.create_publisher(Twist, topic, 10)
        self.topic = topic

        self.speed_xy = 0.8
        self.speed_z = 0.5
        self.speed_yaw = 0.8

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0

        self.get_logger().info(f"Parrot Teleop publishing to: '{self.topic}'")

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.vx)
        msg.linear.y = float(self.vy)
        msg.linear.z = float(self.vz)
        msg.angular.z = float(self.wz)
        self.pub.publish(msg)

    def print_status(self):
        print(
            f"\rSpeed XY: {self.speed_xy:.1f} m/s | Z: {self.speed_z:.1f} m/s | "
            f"Active: vx={self.vx:+.1f} vy={self.vy:+.1f} vz={self.vz:+.1f} wz={self.wz:+.1f}    ",
            end='',
            flush=True,
        )


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # escape sequence
            extra = sys.stdin.read(2)
            if extra == '[A':
                key = 'w'
            elif extra == '[B':
                key = 's'
            elif extra == '[C':
                key = 'd'
            elif extra == '[D':
                key = 'a'
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    topic = '/parrot1/cmd_vel_nav'
    if len(sys.argv) > 1 and sys.argv[1].startswith('/'):
        topic = sys.argv[1]

    rclpy.init(args=args)
    teleop = ParrotTeleop(topic)

    print(USAGE)
    print(f"Publishing to topic: {topic}")
    print("Click inside this terminal and press keys to control the drone.")

    settings = termios.tcgetattr(sys.stdin)

    try:
        teleop.print_status()
        while rclpy.ok():
            key = get_key(settings, timeout=0.05)

            if not key:
                # Key released or idle: zero out impulse velocities (except altitude follow holds z)
                if teleop.vx != 0.0 or teleop.vy != 0.0 or teleop.vz != 0.0 or teleop.wz != 0.0:
                    teleop.vx = 0.0
                    teleop.vy = 0.0
                    teleop.vz = 0.0
                    teleop.wz = 0.0
                    teleop.publish_cmd()
                    teleop.print_status()
                continue

            if key == '\x03':  # Ctrl-C
                break

            if key in ('w', 'W'):
                teleop.vx = teleop.speed_xy
            elif key in ('s', 'S'):
                teleop.vx = -teleop.speed_xy
            elif key in ('a', 'A'):
                teleop.vy = teleop.speed_xy
            elif key in ('d', 'D'):
                teleop.vy = -teleop.speed_xy
            elif key in ('q', 'Q'):
                teleop.wz = teleop.speed_yaw
            elif key in ('e', 'E'):
                teleop.wz = -teleop.speed_yaw
            elif key == ' ':  # Ascend
                teleop.vz = teleop.speed_z
            elif key in ('c', 'C'):  # Descend
                teleop.vz = -teleop.speed_z
            elif key in ('x', 'k', 'K', 'X'):
                teleop.vx = 0.0
                teleop.vy = 0.0
                teleop.vz = 0.0
                teleop.wz = 0.0
            elif key in ('+', '='):
                teleop.speed_xy = min(3.0, teleop.speed_xy + 0.2)
                teleop.speed_z = min(2.0, teleop.speed_z + 0.1)
            elif key in ('-', '_'):
                teleop.speed_xy = max(0.2, teleop.speed_xy - 0.2)
                teleop.speed_z = max(0.1, teleop.speed_z - 0.1)

            teleop.publish_cmd()
            teleop.print_status()

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        stop_twist = Twist()
        teleop.pub.publish(stop_twist)
        print("\nDrone stopped. Teleop exited.")
        teleop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
