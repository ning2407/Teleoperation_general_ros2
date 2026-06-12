"""Frame conversion helpers for Cartesian teleoperation commands."""

from __future__ import annotations

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import tf2_geometry_msgs  # noqa: F401
except ImportError:
    tf2_geometry_msgs = None


class FrameTransformer:
    """Transforms pose and twist commands into the robot command frame."""

    def __init__(self, node: Node, target_frame: str, timeout_s: float = 0.05) -> None:
        self._node = node
        self._target_frame = target_frame
        self._timeout = Duration(seconds=timeout_s)
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)

    @property
    def target_frame(self) -> str:
        return self._target_frame

    def transform_pose(self, command: PoseStamped) -> PoseStamped:
        source_frame = command.header.frame_id
        if not source_frame or source_frame == self._target_frame:
            output = deepcopy(command)
            output.header.frame_id = self._target_frame
            return output

        return self._buffer.transform(command, self._target_frame, timeout=self._timeout)

    def transform_twist(self, command: TwistStamped) -> TwistStamped:
        source_frame = command.header.frame_id
        if not source_frame or source_frame == self._target_frame:
            output = deepcopy(command)
            output.header.frame_id = self._target_frame
            return output

        transform = self._buffer.lookup_transform(
            self._target_frame,
            source_frame,
            Time.from_msg(command.header.stamp),
            timeout=self._timeout,
        )
        return self._apply_rotation(command, transform)

    def can_transform(self, source_frame: str) -> bool:
        if not source_frame or source_frame == self._target_frame:
            return True
        try:
            return self._buffer.can_transform(
                self._target_frame,
                source_frame,
                self._node.get_clock().now(),
            )
        except TransformException:
            return False

    def _apply_rotation(self, command: TwistStamped, transform: TransformStamped) -> TwistStamped:
        output = deepcopy(command)
        output.header.frame_id = self._target_frame

        q = transform.transform.rotation
        linear = self._rotate(
            [command.twist.linear.x, command.twist.linear.y, command.twist.linear.z],
            [q.x, q.y, q.z, q.w],
        )
        angular = self._rotate(
            [command.twist.angular.x, command.twist.angular.y, command.twist.angular.z],
            [q.x, q.y, q.z, q.w],
        )
        output.twist.linear = Vector3(x=linear[0], y=linear[1], z=linear[2])
        output.twist.angular = Vector3(x=angular[0], y=angular[1], z=angular[2])
        return output

    @staticmethod
    def _rotate(vector: list[float], quat: list[float]) -> list[float]:
        x, y, z, w = FrameTransformer._normalize_quat(quat)
        vx, vy, vz = vector

        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)

        return [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ]

    @staticmethod
    def _normalize_quat(quat: list[float]) -> list[float]:
        norm = math.sqrt(sum(value * value for value in quat))
        if norm < 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        return [value / norm for value in quat]
