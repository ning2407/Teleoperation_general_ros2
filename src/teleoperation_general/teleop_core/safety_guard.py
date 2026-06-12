"""Safety checks for joint and Cartesian teleoperation commands."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState


@dataclass(frozen=True)
class SafetyResult:
    ok: bool
    reason: str = ""

    @classmethod
    def pass_(cls) -> "SafetyResult":
        return cls(True, "")

    @classmethod
    def fail(cls, reason: str) -> "SafetyResult":
        return cls(False, reason)


@dataclass
class SafetyLimits:
    command_timeout_s: float = 0.2
    require_enable: bool = True
    max_linear_velocity: float = 0.15
    max_angular_velocity: float = 0.4
    workspace_min: list[float] = field(default_factory=lambda: [-1.0, -1.0, 0.0])
    workspace_max: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.2])
    joint_names: list[str] = field(default_factory=list)
    min_joint_position: list[float] = field(default_factory=list)
    max_joint_position: list[float] = field(default_factory=list)
    max_joint_velocity: list[float] = field(default_factory=list)


class SafetyGuard:
    """Stateless command validator plus emergency-stop latch."""

    def __init__(self, limits: SafetyLimits) -> None:
        self._limits = limits
        self._emergency_stop = False

    @property
    def emergency_stop(self) -> bool:
        return self._emergency_stop

    def set_emergency_stop(self, enabled: bool) -> None:
        self._emergency_stop = enabled

    def check_common(self, enabled: bool) -> SafetyResult:
        if self._emergency_stop:
            return SafetyResult.fail("emergency stop is active")
        if self._limits.require_enable and not enabled:
            return SafetyResult.fail("teleoperation is not enabled")
        return SafetyResult.pass_()

    def check_joint_position(self, command: JointState, enabled: bool) -> SafetyResult:
        common = self.check_common(enabled)
        if not common.ok:
            return common

        if not command.position:
            return SafetyResult.fail("joint position command has no positions")

        if self._limits.joint_names:
            name_check = self._check_joint_names(command.name)
            if not name_check.ok:
                return name_check

        if self._limits.min_joint_position and self._limits.max_joint_position:
            if len(self._limits.min_joint_position) != len(self._limits.max_joint_position):
                return SafetyResult.fail("joint position limits have mismatched lengths")
            if len(command.position) > len(self._limits.min_joint_position):
                return SafetyResult.fail("joint position command is longer than configured limits")
            for index, value in enumerate(command.position):
                if not math.isfinite(value):
                    return SafetyResult.fail(f"joint {index} position is not finite")
                lower = self._limits.min_joint_position[index]
                upper = self._limits.max_joint_position[index]
                if value < lower or value > upper:
                    return SafetyResult.fail(
                        f"joint {index} position {value:.3f} outside [{lower:.3f}, {upper:.3f}]"
                    )

        return SafetyResult.pass_()

    def check_joint_velocity(self, command: JointState, enabled: bool) -> SafetyResult:
        common = self.check_common(enabled)
        if not common.ok:
            return common

        if not command.velocity:
            return SafetyResult.fail("joint velocity command has no velocities")

        if self._limits.joint_names:
            name_check = self._check_joint_names(command.name)
            if not name_check.ok:
                return name_check

        if self._limits.max_joint_velocity:
            if len(command.velocity) > len(self._limits.max_joint_velocity):
                return SafetyResult.fail("joint velocity command is longer than configured limits")
            for index, value in enumerate(command.velocity):
                if not math.isfinite(value):
                    return SafetyResult.fail(f"joint {index} velocity is not finite")
                limit = abs(self._limits.max_joint_velocity[index])
                if abs(value) > limit:
                    return SafetyResult.fail(
                        f"joint {index} velocity {value:.3f} exceeds limit {limit:.3f}"
                    )

        return SafetyResult.pass_()

    def check_cartesian_twist(self, command: TwistStamped, enabled: bool) -> SafetyResult:
        common = self.check_common(enabled)
        if not common.ok:
            return common

        linear_norm = self._norm(
            command.twist.linear.x,
            command.twist.linear.y,
            command.twist.linear.z,
        )
        angular_norm = self._norm(
            command.twist.angular.x,
            command.twist.angular.y,
            command.twist.angular.z,
        )

        if not math.isfinite(linear_norm) or not math.isfinite(angular_norm):
            return SafetyResult.fail("cartesian twist contains non-finite values")

        if linear_norm > self._limits.max_linear_velocity:
            return SafetyResult.fail(
                f"linear velocity {linear_norm:.3f} exceeds limit "
                f"{self._limits.max_linear_velocity:.3f}"
            )
        if angular_norm > self._limits.max_angular_velocity:
            return SafetyResult.fail(
                f"angular velocity {angular_norm:.3f} exceeds limit "
                f"{self._limits.max_angular_velocity:.3f}"
            )

        return SafetyResult.pass_()

    def check_cartesian_pose(self, command: PoseStamped, enabled: bool) -> SafetyResult:
        common = self.check_common(enabled)
        if not common.ok:
            return common

        position = command.pose.position
        lower = self._limits.workspace_min
        upper = self._limits.workspace_max
        if len(lower) != 3 or len(upper) != 3:
            return SafetyResult.fail("workspace limits must contain exactly 3 values")
        values = [position.x, position.y, position.z]

        for index, value in enumerate(values):
            if not math.isfinite(value):
                return SafetyResult.fail(f"{'xyz'[index]} position is not finite")
            if value < lower[index] or value > upper[index]:
                axis = "xyz"[index]
                return SafetyResult.fail(
                    f"{axis} position {value:.3f} outside workspace "
                    f"[{lower[index]:.3f}, {upper[index]:.3f}]"
                )

        q = command.pose.orientation
        q_norm = self._norm(q.x, q.y, q.z, q.w)
        if not math.isfinite(q_norm):
            return SafetyResult.fail("pose orientation quaternion contains non-finite values")
        if q_norm < 1e-6:
            return SafetyResult.fail("pose orientation quaternion has near-zero norm")

        return SafetyResult.pass_()

    def _check_joint_names(self, names: Sequence[str]) -> SafetyResult:
        if not names:
            return SafetyResult.fail("joint command has no names")
        if list(names) != self._limits.joint_names:
            return SafetyResult.fail("joint names do not match configured robot joints")
        return SafetyResult.pass_()

    @staticmethod
    def _norm(*values: float) -> float:
        return math.sqrt(sum(value * value for value in values))
