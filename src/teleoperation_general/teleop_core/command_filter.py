"""Deadband, saturation and smoothing filters for teleoperation commands."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import math
import time
from typing import Iterable

from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState


@dataclass
class FilterConfig:
    deadband: float = 1e-4
    smoothing_alpha: float = 0.35
    max_linear_velocity: float = 0.15
    max_angular_velocity: float = 0.4
    max_joint_velocity: list[float] = field(default_factory=list)
    min_joint_position: list[float] = field(default_factory=list)
    max_joint_position: list[float] = field(default_factory=list)


class CommandFilter:
    """Applies predictable command shaping before commands reach adapters."""

    def __init__(self, config: FilterConfig) -> None:
        self._config = config
        self._last_joint_velocity: list[float] | None = None
        self._last_linear: list[float] | None = None
        self._last_angular: list[float] | None = None
        self._last_update_s = time.monotonic()

    def reset(self) -> None:
        self._last_joint_velocity = None
        self._last_linear = None
        self._last_angular = None
        self._last_update_s = time.monotonic()

    def filter_joint_velocity(self, command: JointState) -> JointState:
        output = deepcopy(command)
        velocities = self._apply_deadband(output.velocity)
        velocities = self._limit_each(velocities, self._config.max_joint_velocity)
        velocities = self._smooth(velocities, self._last_joint_velocity)
        self._last_joint_velocity = velocities
        output.velocity = velocities
        return output

    def filter_joint_position(self, command: JointState) -> JointState:
        output = deepcopy(command)
        if self._config.min_joint_position and self._config.max_joint_position:
            count = min(
                len(output.position),
                len(self._config.min_joint_position),
                len(self._config.max_joint_position),
            )
            output.position = [
                self._clamp(
                    output.position[index],
                    self._config.min_joint_position[index],
                    self._config.max_joint_position[index],
                )
                for index in range(count)
            ] + list(output.position[count:])
        return output

    def filter_cartesian_twist(self, command: TwistStamped) -> TwistStamped:
        output = deepcopy(command)

        linear = self._apply_deadband([
            output.twist.linear.x,
            output.twist.linear.y,
            output.twist.linear.z,
        ])
        angular = self._apply_deadband([
            output.twist.angular.x,
            output.twist.angular.y,
            output.twist.angular.z,
        ])

        linear = self._limit_norm(linear, self._config.max_linear_velocity)
        angular = self._limit_norm(angular, self._config.max_angular_velocity)
        linear = self._smooth(linear, self._last_linear)
        angular = self._smooth(angular, self._last_angular)

        self._last_linear = linear
        self._last_angular = angular
        output.twist.linear = Vector3(x=linear[0], y=linear[1], z=linear[2])
        output.twist.angular = Vector3(x=angular[0], y=angular[1], z=angular[2])
        return output

    def make_zero_joint_velocity(self, joint_names: Iterable[str]) -> JointState:
        msg = JointState()
        msg.name = list(joint_names)
        msg.velocity = [0.0] * len(msg.name)
        return msg

    def make_zero_twist(self, frame_id: str) -> TwistStamped:
        msg = TwistStamped()
        msg.header.frame_id = frame_id
        return msg

    def _smooth(self, current: list[float], previous: list[float] | None) -> list[float]:
        if previous is None or len(previous) != len(current):
            return current

        alpha = self._clamp(self._config.smoothing_alpha, 0.0, 1.0)
        return [
            alpha * value + (1.0 - alpha) * previous[index]
            for index, value in enumerate(current)
        ]

    def _apply_deadband(self, values: Iterable[float]) -> list[float]:
        deadband = abs(self._config.deadband)
        return [0.0 if abs(value) < deadband else float(value) for value in values]

    @staticmethod
    def _limit_each(values: list[float], limits: list[float]) -> list[float]:
        if not limits:
            return values
        return [
            CommandFilter._clamp(value, -abs(limits[index]), abs(limits[index]))
            for index, value in enumerate(values)
        ]

    @staticmethod
    def _limit_norm(values: list[float], limit: float) -> list[float]:
        limit = abs(limit)
        norm = math.sqrt(sum(value * value for value in values))
        if norm <= limit or norm < 1e-12:
            return values
        scale = limit / norm
        return [value * scale for value in values]

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)
