#!/usr/bin/env python3
"""Keyboard input hardware that publishes generic teleoperation commands."""

from __future__ import annotations

import sys
import termios
import tty

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

from teleop_msgs.msg import TeleopCommand


HELP = """
Keyboard Servo Hardware

w/s : +X / -X
a/d : +Y / -Y
q/e : +Z / -Z

i/k : +Rx / -Rx
j/l : +Ry / -Ry
u/o : +Rz / -Rz

space : stop
x     : stop and exit
"""


class KeyboardServoHardware(Node):
    """Convert keyboard key presses into TeleopCommand messages."""

    def __init__(self) -> None:
        super().__init__("keyboard_servo")

        self.declare_parameter("command_topic", "/teleop/command")
        self.declare_parameter("robot_name", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("linear_step", 0.5)
        self.declare_parameter("angular_step", 0.2)
        self.declare_parameter("enable", True)

        self.command_topic = self.get_parameter("command_topic").value
        self.robot_name = self.get_parameter("robot_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        self.enable = bool(self.get_parameter("enable").value)
        self.sequence_id = 0

        self.pub = self.create_publisher(TeleopCommand, self.command_topic, 10)
        self.get_logger().info(HELP)
        self.get_logger().info(f"publishing TeleopCommand on {self.command_topic}")

    def publish_command(
        self,
        linear: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
        *,
        mode: int = TeleopCommand.CARTESIAN_TWIST,
    ) -> None:
        now = self.get_clock().now().to_msg()

        twist = TwistStamped()
        twist.header.stamp = now
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = linear[0]
        twist.twist.linear.y = linear[1]
        twist.twist.linear.z = linear[2]
        twist.twist.angular.x = angular[0]
        twist.twist.angular.y = angular[1]
        twist.twist.angular.z = angular[2]

        msg = TeleopCommand()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.robot_name = self.robot_name
        msg.sequence_id = self.sequence_id
        msg.control_mode = mode
        msg.enable = self.enable
        msg.emergency_stop = False
        msg.command_frame = self.frame_id
        msg.target_twist = twist

        self.sequence_id += 1
        self.pub.publish(msg)

        self.get_logger().info(
            "linear=(%.3f, %.3f, %.3f) angular=(%.3f, %.3f, %.3f)"
            % (*linear, *angular)
        )

    def get_key(self) -> str:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def spin_keyboard(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().error("keyboard_servo requires an interactive terminal")
            return

        while rclpy.ok():
            key = self.get_key()

            if key == "w":
                self.publish_command(linear=(self.linear_step, 0.0, 0.0))
            elif key == "s":
                self.publish_command(linear=(-self.linear_step, 0.0, 0.0))
            elif key == "a":
                self.publish_command(linear=(0.0, self.linear_step, 0.0))
            elif key == "d":
                self.publish_command(linear=(0.0, -self.linear_step, 0.0))
            elif key == "q":
                self.publish_command(linear=(0.0, 0.0, self.linear_step))
            elif key == "e":
                self.publish_command(linear=(0.0, 0.0, -self.linear_step))
            elif key == "i":
                self.publish_command(angular=(self.angular_step, 0.0, 0.0))
            elif key == "k":
                self.publish_command(angular=(-self.angular_step, 0.0, 0.0))
            elif key == "j":
                self.publish_command(angular=(0.0, self.angular_step, 0.0))
            elif key == "l":
                self.publish_command(angular=(0.0, -self.angular_step, 0.0))
            elif key == "u":
                self.publish_command(angular=(0.0, 0.0, self.angular_step))
            elif key == "o":
                self.publish_command(angular=(0.0, 0.0, -self.angular_step))
            elif key == " ":
                self.publish_command(mode=TeleopCommand.IDLE)
            elif key == "x":
                self.publish_command(mode=TeleopCommand.IDLE)
                break


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KeyboardServoHardware()

    try:
        node.spin_keyboard()
    except KeyboardInterrupt:
        node.publish_command(mode=TeleopCommand.IDLE)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
