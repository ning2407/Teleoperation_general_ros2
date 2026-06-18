#!/usr/bin/env python3
"""Phone IMU hardware input for generic teleoperation commands."""

from __future__ import annotations

import ast
import json
import socket
import threading
from typing import Any

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from teleop_msgs.msg import TeleopCommand


class PhoneImuServoHardware(Node):
    """Convert phone IMU packets into TeleopCommand twist commands."""

    def __init__(self) -> None:
        super().__init__("phone_imu_servo")

        self.declare_parameter("command_topic", "/teleop/command")
        self.declare_parameter("robot_name", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("enable", True)

        self.declare_parameter("protocol", "tcp")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 9000)
        self.declare_parameter("socket_timeout_s", 0.2)

        self.declare_parameter("angular_scale", 0.8)
        self.declare_parameter("linear_scale", 0.05)
        self.declare_parameter("deadband", 0.02)
        self.declare_parameter("use_orientation_for_angular", True)
        self.declare_parameter("use_tilt_for_linear", False)
        self.declare_parameter("invert_roll", False)
        self.declare_parameter("invert_pitch", False)
        self.declare_parameter("invert_yaw", False)

        self.command_topic = self.get_parameter("command_topic").value
        self.robot_name = self.get_parameter("robot_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.enable = bool(self.get_parameter("enable").value)
        self.protocol = self.get_parameter("protocol").value.lower()
        self.listen_host = self.get_parameter("listen_host").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.socket_timeout_s = float(self.get_parameter("socket_timeout_s").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.use_orientation_for_angular = bool(
            self.get_parameter("use_orientation_for_angular").value
        )
        self.use_tilt_for_linear = bool(self.get_parameter("use_tilt_for_linear").value)
        self.invert_roll = bool(self.get_parameter("invert_roll").value)
        self.invert_pitch = bool(self.get_parameter("invert_pitch").value)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)

        self.sequence_id = 0
        self.stop_event = threading.Event()
        self.server_socket: socket.socket | None = None

        self.pub = self.create_publisher(TeleopCommand, self.command_topic, 10)
        self.worker = threading.Thread(target=self.run_server, daemon=True)
        self.worker.start()

        self.get_logger().info(
            "phone_imu_servo listening on %s://%s:%d, publishing %s"
            % (self.protocol, self.listen_host, self.listen_port, self.command_topic)
        )

    def run_server(self) -> None:
        if self.protocol == "udp":
            self.run_udp_server()
        else:
            self.run_tcp_server()

    def run_tcp_server(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket = server
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(self.socket_timeout_s)
        server.bind((self.listen_host, self.listen_port))
        server.listen(1)

        while not self.stop_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(f"phone IMU connected from {addr}")
            with conn:
                conn.settimeout(self.socket_timeout_s)
                self.read_tcp_connection(conn)

    def read_tcp_connection(self, conn: socket.socket) -> None:
        buffer = ""
        while not self.stop_event.is_set():
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break

            buffer += chunk.decode("utf-8", errors="ignore")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                self.handle_packet_text(line.strip())

            if buffer.strip().endswith("}"):
                if self.handle_packet_text(buffer.strip()):
                    buffer = ""

    def run_udp_server(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket = server
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(self.socket_timeout_s)
        server.bind((self.listen_host, self.listen_port))

        while not self.stop_event.is_set():
            try:
                data, _ = server.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            self.handle_packet_text(data.decode("utf-8", errors="ignore").strip())

    def handle_packet_text(self, text: str) -> bool:
        if not text:
            return False

        packet = self.parse_packet(text)
        if packet is None:
            return False

        self.publish_from_packet(packet)
        return True

    def parse_packet(self, text: str) -> dict[str, Any] | None:
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            try:
                parsed = ast.literal_eval(text)
            except (SyntaxError, ValueError):
                self.get_logger().warning("failed to parse phone IMU packet")
                return None

        if not isinstance(parsed, dict):
            self.get_logger().warning("phone IMU packet is not a dict/object")
            return None
        return parsed

    def publish_from_packet(self, packet: dict[str, Any]) -> None:
        euler = packet.get("euler_relative", {})
        if not isinstance(euler, dict):
            euler = {}

        roll = self.signed_value(float(euler.get("roll", 0.0)), self.invert_roll)
        pitch = self.signed_value(float(euler.get("pitch", 0.0)), self.invert_pitch)
        yaw = self.signed_value(float(euler.get("yaw", 0.0)), self.invert_yaw)

        linear = (0.0, 0.0, 0.0)
        if self.use_tilt_for_linear:
            linear = (
                self.apply_deadband(pitch) * self.linear_scale,
                self.apply_deadband(roll) * self.linear_scale,
                0.0,
            )

        angular = (0.0, 0.0, 0.0)
        if self.use_orientation_for_angular:
            angular = (
                self.apply_deadband(roll) * self.angular_scale,
                self.apply_deadband(pitch) * self.angular_scale,
                self.apply_deadband(yaw) * self.angular_scale,
            )

        self.publish_command(linear=linear, angular=angular)

    def publish_command(
        self,
        linear: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
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
        msg.control_mode = TeleopCommand.CARTESIAN_TWIST
        msg.enable = self.enable
        msg.emergency_stop = False
        msg.command_frame = self.frame_id
        msg.target_twist = twist
        self.sequence_id += 1

        self.pub.publish(msg)

    def apply_deadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value

    @staticmethod
    def signed_value(value: float, invert: bool) -> float:
        return -value if invert else value

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PhoneImuServoHardware()
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
