"""Control-mode state machine for teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time
from typing import Iterable


class ControlMode(str, Enum):
    """Supported command spaces for a generic teleoperation bridge."""

    IDLE = "idle"
    JOINT_POSITION = "joint_position"
    JOINT_VELOCITY = "joint_velocity"
    CARTESIAN_POSE = "cartesian_pose"
    CARTESIAN_TWIST = "cartesian_twist"
    GRIPPER = "gripper"

    @classmethod
    def parse(cls, value: str | "ControlMode") -> "ControlMode":
        if isinstance(value, ControlMode):
            return value

        normalized = value.strip().lower()
        aliases = {
            "joint_pos": cls.JOINT_POSITION,
            "jpos": cls.JOINT_POSITION,
            "joint_vel": cls.JOINT_VELOCITY,
            "jvel": cls.JOINT_VELOCITY,
            "pose": cls.CARTESIAN_POSE,
            "cartesian": cls.CARTESIAN_POSE,
            "cartesian_vel": cls.CARTESIAN_TWIST,
            "cartesian_velocity": cls.CARTESIAN_TWIST,
            "twist": cls.CARTESIAN_TWIST,
            "eef_twist": cls.CARTESIAN_TWIST,
        }
        if normalized in aliases:
            return aliases[normalized]
        return cls(normalized)


@dataclass(frozen=True)
class ModeResult:
    accepted: bool
    mode: ControlMode
    reason: str = ""


class ModeManager:
    """Small state machine that owns the active teleoperation mode."""

    def __init__(
        self,
        default_mode: ControlMode | str = ControlMode.IDLE,
        allowed_modes: Iterable[ControlMode | str] | None = None,
        switch_debounce_s: float = 0.25,
    ) -> None:
        self._active_mode = ControlMode.parse(default_mode)
        self._allowed_modes = {
            ControlMode.parse(mode)
            for mode in (allowed_modes or list(ControlMode))
        }
        self._allowed_modes.add(ControlMode.IDLE)
        self._switch_debounce_s = max(0.0, switch_debounce_s)
        self._last_switch_time = 0.0

        if self._active_mode not in self._allowed_modes:
            raise ValueError(f"default mode {self._active_mode.value!r} is not allowed")

    @property
    def active_mode(self) -> ControlMode:
        return self._active_mode

    @property
    def allowed_modes(self) -> set[ControlMode]:
        return set(self._allowed_modes)

    def accepts(self, command_mode: ControlMode | str) -> bool:
        mode = ControlMode.parse(command_mode)
        return self._active_mode == mode and mode in self._allowed_modes

    def request_mode(
        self,
        requested_mode: ControlMode | str,
        *,
        now_s: float | None = None,
        force: bool = False,
    ) -> ModeResult:
        try:
            mode = ControlMode.parse(requested_mode)
        except ValueError:
            return ModeResult(False, self._active_mode, "unknown mode")

        if mode not in self._allowed_modes:
            return ModeResult(False, self._active_mode, f"mode {mode.value} is not allowed")

        now_s = time.monotonic() if now_s is None else now_s
        if (
            not force
            and mode != self._active_mode
            and now_s - self._last_switch_time < self._switch_debounce_s
        ):
            return ModeResult(False, self._active_mode, "mode switch debounced")

        if mode != self._active_mode:
            self._active_mode = mode
            self._last_switch_time = now_s

        return ModeResult(True, self._active_mode)

    def stop(self) -> ModeResult:
        self._active_mode = ControlMode.IDLE
        self._last_switch_time = time.monotonic()
        return ModeResult(True, self._active_mode)
