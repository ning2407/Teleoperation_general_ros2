#!/usr/bin/env python3
"""Xbox controller hardware input for generic teleoperation commands."""

from __future__ import annotations

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from teleop_msgs.msg import TeleopCommand


class XboxServoHardware(Node):
    """Convert sensor_msgs/Joy messages into TeleopCommand twist commands."""

    def __init__(self) -> None:
        super().__init__("xbox_servo")

        self.declare_parameter("command_topic", "/teleop/command")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("robot_name", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("enable", True)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("deadband", 0.08)
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("deadman_button", 4)
        self.declare_parameter("stop_button", 1)

        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_angular_x", 4)
        self.declare_parameter("axis_angular_y", 3)
        self.declare_parameter("axis_linear_z_positive", 5)
        self.declare_parameter("axis_linear_z_negative", 2)
        self.declare_parameter("button_yaw_positive", 3)
        self.declare_parameter("button_yaw_negative", 0)

        self.declare_parameter("linear_scale", 0.10)
        self.declare_parameter("angular_scale", 0.30)
        self.declare_parameter("trigger_is_released_at_one", True)

        self.command_topic = self.get_parameter("command_topic").value
        self.joy_topic = self.get_parameter("joy_topic").value
        self.robot_name = self.get_parameter("robot_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.enable = bool(self.get_parameter("enable").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.require_deadman = bool(self.get_parameter("require_deadman").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.stop_button = int(self.get_parameter("stop_button").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.trigger_is_released_at_one = bool(
            self.get_parameter("trigger_is_released_at_one").value
        )

        self.axis_linear_x = int(self.get_parameter("axis_linear_x").value)
        self.axis_linear_y = int(self.get_parameter("axis_linear_y").value)
        self.axis_angular_x = int(self.get_parameter("axis_angular_x").value)
        self.axis_angular_y = int(self.get_parameter("axis_angular_y").value)
        self.axis_linear_z_positive = int(
            self.get_parameter("axis_linear_z_positive").value
        )
        self.axis_linear_z_negative = int(
            self.get_parameter("axis_linear_z_negative").value
        )
        self.button_yaw_positive = int(self.get_parameter("button_yaw_positive").value)
        self.button_yaw_negative = int(self.get_parameter("button_yaw_negative").value)

        self.sequence_id = 0
        self.last_joy: Joy | None = None
        self.last_sent_idle = False

        self.pub = self.create_publisher(TeleopCommand, self.command_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(publish_rate, 1.0), self.publish_from_joy)

        self.get_logger().info(
            f"xbox_servo publishing TeleopCommand on {self.command_topic}, "
            f"reading Joy from {self.joy_topic}"
        )

    def on_joy(self, msg: Joy) -> None:
        self.last_joy = msg

    def publish_from_joy(self) -> None:
        if self.last_joy is None:
            return

        if self.button(self.stop_button):
            self.publish_command(mode=TeleopCommand.IDLE)
            self.last_sent_idle = True
            return

        if self.require_deadman and not self.button(self.deadman_button):
            if not self.last_sent_idle:
                self.publish_command(mode=TeleopCommand.IDLE)
                self.last_sent_idle = True
            return

        linear = (
            self.filtered_axis(self.axis_linear_x) * self.linear_scale,
            self.filtered_axis(self.axis_linear_y) * self.linear_scale,
            self.linear_z_value() * self.linear_scale,
        )
        angular = (
            self.filtered_axis(self.axis_angular_x) * self.angular_scale,
            self.filtered_axis(self.axis_angular_y) * self.angular_scale,
            self.yaw_value() * self.angular_scale,
        )
        self.publish_command(linear=linear, angular=angular)
        self.last_sent_idle = False

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

    def linear_z_value(self) -> float:
        positive = self.trigger_value(self.axis_linear_z_positive)
        negative = self.trigger_value(self.axis_linear_z_negative)
        return self.apply_deadband(positive - negative)

    def yaw_value(self) -> float:
        positive = 1.0 if self.button(self.button_yaw_positive) else 0.0
        negative = 1.0 if self.button(self.button_yaw_negative) else 0.0
        return positive - negative

    def filtered_axis(self, index: int) -> float:
        return self.apply_deadband(self.axis(index))

    def axis(self, index: int) -> float:
        if self.last_joy is None or index < 0 or index >= len(self.last_joy.axes):
            return 0.0
        return float(self.last_joy.axes[index])

    def trigger_value(self, index: int) -> float:
        value = self.axis(index)
        if self.trigger_is_released_at_one:
            value = (1.0 - value) * 0.5
        return max(0.0, min(1.0, value))

    def button(self, index: int) -> bool:
        if self.last_joy is None or index < 0 or index >= len(self.last_joy.buttons):
            return False
        return bool(self.last_joy.buttons[index])

    def apply_deadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = XboxServoHardware()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
