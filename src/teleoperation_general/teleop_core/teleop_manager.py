"""ROS 2 node that validates and forwards generic teleoperation commands."""

from __future__ import annotations

from typing import Callable

from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from teleop_core.command_filter import CommandFilter, FilterConfig
from teleop_core.frame_transformer import FrameTransformer
from teleop_core.mode_manager import ControlMode, ModeManager
from teleop_core.safety_guard import SafetyGuard, SafetyLimits, SafetyResult
from teleop_msgs.msg import TeleopCommand, TeleopStatus
from teleop_msgs.srv import ResetEmergencyStop, SetControlMode, SetTeleopEnable
from tf2_ros import TransformException


class TeleopManager(Node):
    """Central bridge between input devices and robot adapters."""

    def __init__(self) -> None:
        super().__init__("teleop_manager")

        self._declare_parameters()

        self.base_frame = self.get_parameter("base_frame").value
        self.robot_name = self.get_parameter("robot_name").value
        self.require_enable = bool(self.get_parameter("require_enable").value)
        self.enabled = not self.require_enable
        self.last_joint_names = list(self.get_parameter("joint_names").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        control_frequency = float(self.get_parameter("control_frequency").value)
        self.last_status_message = ""
        self.last_stop_reason = ""
        self.command_timeout = False

        self.mode_manager = ModeManager(
            default_mode=self.get_parameter("default_mode").value,
            allowed_modes=self.get_parameter("allowed_modes").value,
            switch_debounce_s=float(self.get_parameter("mode_switch_debounce_s").value),
        )
        limits = self._make_safety_limits()
        self.safety_guard = SafetyGuard(limits)
        self.command_filter = CommandFilter(self._make_filter_config(limits))
        self.frame_transformer = FrameTransformer(
            self,
            target_frame=self.base_frame,
            timeout_s=float(self.get_parameter("transform_timeout_s").value),
        )

        self.last_command_time = self.get_clock().now()

        self.joint_velocity_pub = self.create_publisher(
            JointState,
            self.get_parameter("validated_joint_velocity_topic").value,
            10,
        )
        self.joint_position_pub = self.create_publisher(
            JointState,
            self.get_parameter("validated_joint_position_topic").value,
            10,
        )
        self.cartesian_twist_pub = self.create_publisher(
            TwistStamped,
            self.get_parameter("validated_cartesian_twist_topic").value,
            10,
        )
        self.cartesian_pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("validated_cartesian_pose_topic").value,
            10,
        )
        self.status_pub = self.create_publisher(
            TeleopStatus,
            self.get_parameter("status_topic").value,
            10,
        )

        self.create_subscription(
            TeleopCommand,
            self.get_parameter("command_topic").value,
            self.on_command,
            10,
        )
        self.create_service(
            SetControlMode,
            self.get_parameter("set_control_mode_service").value,
            self.on_set_control_mode,
        )
        self.create_service(
            SetTeleopEnable,
            self.get_parameter("set_teleop_enable_service").value,
            self.on_set_teleop_enable,
        )
        self.create_service(
            ResetEmergencyStop,
            self.get_parameter("reset_emergency_stop_service").value,
            self.on_reset_emergency_stop,
        )

        timer_period = 1.0 / max(control_frequency, 1.0)
        self.create_timer(timer_period, self.watchdog)

        self.publish_status(f"ready: mode={self.mode_manager.active_mode.value}")

    def on_command(self, msg: TeleopCommand) -> None:
        if msg.robot_name:
            self.robot_name = msg.robot_name
        self.enabled = bool(msg.enable)
        if msg.emergency_stop:
            self.safety_guard.set_emergency_stop(True)
            self.mode_manager.stop()
            self.publish_stop("emergency stop active")
            return

        mode = self._mode_from_teleop_value(msg.control_mode)
        if mode is None:
            self.publish_stop("unknown command mode")
            return

        if mode != self.mode_manager.active_mode:
            result = self.mode_manager.request_mode(mode)
            if not result.accepted:
                self.publish_status(f"mode rejected: {result.reason}", safety_ok=False)
                return
            self.command_filter.reset()

        if mode == ControlMode.JOINT_VELOCITY:
            self.on_joint_velocity(msg.joint_command)
        elif mode == ControlMode.JOINT_POSITION:
            self.on_joint_position(msg.joint_command)
        elif mode == ControlMode.CARTESIAN_TWIST:
            self.on_cartesian_twist(msg.target_twist)
        elif mode == ControlMode.CARTESIAN_POSE:
            self.on_cartesian_pose(msg.target_pose)
        elif mode == ControlMode.IDLE:
            self.publish_stop("idle command", publish_status=False)
        else:
            self.publish_status("gripper command ignored: no gripper adapter configured")

    def on_set_control_mode(
        self,
        request: SetControlMode.Request,
        response: SetControlMode.Response,
    ) -> SetControlMode.Response:
        mode = self._mode_from_teleop_value(request.control_mode)
        if mode is None:
            response.success = False
            response.message = "unknown mode"
            self.publish_status(response.message, safety_ok=False)
            return response

        result = self.mode_manager.request_mode(mode)
        if result.accepted:
            self.command_filter.reset()
            self.publish_stop(f"mode set to {result.mode.value}", publish_status=False)
            self.publish_status(f"mode={result.mode.value}")
            response.success = True
            response.message = f"mode={result.mode.value}"
        else:
            self.publish_status(f"mode rejected: {result.reason}")
            response.success = False
            response.message = result.reason
        return response

    def on_set_teleop_enable(
        self,
        request: SetTeleopEnable.Request,
        response: SetTeleopEnable.Response,
    ) -> SetTeleopEnable.Response:
        self.enabled = bool(request.enable)
        if not self.enabled:
            self.publish_stop("teleoperation disabled")
            response.message = "teleoperation disabled"
        else:
            self.publish_status("teleoperation enabled")
            response.message = "teleoperation enabled"
        response.success = True
        return response

    def on_reset_emergency_stop(
        self,
        request: ResetEmergencyStop.Request,
        response: ResetEmergencyStop.Response,
    ) -> ResetEmergencyStop.Response:
        del request
        self.safety_guard.set_emergency_stop(False)
        self.last_stop_reason = ""
        self.publish_status("emergency stop cleared")
        response.success = True
        response.message = "emergency stop cleared"
        return response

    def on_joint_velocity(self, msg: JointState) -> None:
        self._handle_command(
            ControlMode.JOINT_VELOCITY,
            msg,
            self.safety_guard.check_joint_velocity,
            self.command_filter.filter_joint_velocity,
            self.joint_velocity_pub.publish,
        )

    def on_joint_position(self, msg: JointState) -> None:
        self._handle_command(
            ControlMode.JOINT_POSITION,
            msg,
            self.safety_guard.check_joint_position,
            self.command_filter.filter_joint_position,
            self.joint_position_pub.publish,
        )

    def on_cartesian_twist(self, msg: TwistStamped) -> None:
        try:
            transformed = self.frame_transformer.transform_twist(msg)
        except TransformException as exc:
            self.publish_stop(f"twist transform failed: {exc}")
            return

        self._handle_command(
            ControlMode.CARTESIAN_TWIST,
            transformed,
            self.safety_guard.check_cartesian_twist,
            self.command_filter.filter_cartesian_twist,
            self.cartesian_twist_pub.publish,
        )

    def on_cartesian_pose(self, msg: PoseStamped) -> None:
        try:
            transformed = self.frame_transformer.transform_pose(msg)
        except TransformException as exc:
            self.publish_stop(f"pose transform failed: {exc}")
            return

        self._handle_command(
            ControlMode.CARTESIAN_POSE,
            transformed,
            self.safety_guard.check_cartesian_pose,
            lambda command: command,
            self.cartesian_pose_pub.publish,
        )

    def watchdog(self) -> None:
        age_s = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        self.command_timeout = age_s > self.command_timeout_s
        if age_s > self.command_timeout_s and self.mode_manager.active_mode != ControlMode.IDLE:
            self.publish_stop("command timeout", publish_status=False)

    def publish_stop(self, reason: str, *, publish_status: bool = True) -> None:
        now = self.get_clock().now().to_msg()
        self.last_stop_reason = reason

        zero_joint = self.command_filter.make_zero_joint_velocity(self.last_joint_names)
        zero_joint.header.stamp = now
        self.joint_velocity_pub.publish(zero_joint)

        zero_twist = self.command_filter.make_zero_twist(self.base_frame)
        zero_twist.header.stamp = now
        self.cartesian_twist_pub.publish(zero_twist)

        self.last_command_time = self.get_clock().now()
        if publish_status:
            self.publish_status(f"stop: {reason}", safety_ok=False)

    def publish_status(self, text: str, *, safety_ok: bool = True) -> None:
        msg = TeleopStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_name = self.robot_name
        msg.active_mode = self._teleop_value_from_mode(self.mode_manager.active_mode)
        msg.enabled = self.enabled
        msg.emergency_stop = self.safety_guard.emergency_stop
        msg.safety_ok = safety_ok and not self.safety_guard.emergency_stop
        age_s = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        msg.input_alive = age_s <= self.command_timeout_s
        msg.command_timeout = age_s > self.command_timeout_s
        msg.command_age = age_s
        msg.last_stop_reason = self.last_stop_reason
        msg.message = text
        self.status_pub.publish(msg)
        self.last_status_message = text
        self.get_logger().info(text)

    def _handle_command(
        self,
        mode: ControlMode,
        command,
        safety_check: Callable[[object, bool], SafetyResult],
        filter_fn: Callable[[object], object],
        publish_fn: Callable[[object], None],
    ) -> None:
        if not self.mode_manager.accepts(mode):
            return

        if isinstance(command, JointState) and command.name:
            self.last_joint_names = list(command.name)

        filtered = filter_fn(command)
        result = safety_check(filtered, self.enabled)
        if not result.ok:
            self.publish_stop(result.reason)
            return

        if hasattr(filtered, "header"):
            filtered.header.stamp = self.get_clock().now().to_msg()
        publish_fn(filtered)
        self.command_timeout = False
        self.last_stop_reason = ""
        self.last_command_time = self.get_clock().now()

    def _declare_parameters(self) -> None:
        self.declare_parameter("robot_name", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("command_topic", "/teleop/command")
        self.declare_parameter("status_topic", "/teleop/status")
        self.declare_parameter(
            "validated_joint_velocity_topic",
            "/teleop/validated/joint_velocity",
        )
        self.declare_parameter(
            "validated_joint_position_topic",
            "/teleop/validated/joint_position",
        )
        self.declare_parameter(
            "validated_cartesian_twist_topic",
            "/teleop/validated/cartesian_twist",
        )
        self.declare_parameter(
            "validated_cartesian_pose_topic",
            "/teleop/validated/cartesian_pose",
        )
        self.declare_parameter("set_control_mode_service", "/teleop/set_control_mode")
        self.declare_parameter("set_teleop_enable_service", "/teleop/set_teleop_enable")
        self.declare_parameter("reset_emergency_stop_service", "/teleop/reset_emergency_stop")
        self.declare_parameter("default_mode", ControlMode.CARTESIAN_TWIST.value)
        self.declare_parameter(
            "allowed_modes",
            [
                ControlMode.JOINT_POSITION.value,
                ControlMode.JOINT_VELOCITY.value,
                ControlMode.CARTESIAN_POSE.value,
                ControlMode.CARTESIAN_TWIST.value,
                ControlMode.GRIPPER.value,
            ],
        )
        self.declare_parameter("control_frequency", 50.0)
        self.declare_parameter("command_timeout_s", 0.2)
        self.declare_parameter("mode_switch_debounce_s", 0.25)
        self.declare_parameter("transform_timeout_s", 0.05)
        self.declare_parameter("require_enable", True)
        self.declare_parameter("deadband", 1e-4)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("max_linear_velocity", 0.15)
        self.declare_parameter("max_angular_velocity", 0.4)
        self.declare_parameter("workspace_min", [-1.0, -1.0, 0.0])
        self.declare_parameter("workspace_max", [1.0, 1.0, 1.2])
        self.declare_parameter("joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("min_joint_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("max_joint_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("max_joint_velocity", Parameter.Type.DOUBLE_ARRAY)

    def _make_safety_limits(self) -> SafetyLimits:
        return SafetyLimits(
            command_timeout_s=self.command_timeout_s,
            require_enable=self.require_enable,
            max_linear_velocity=float(self.get_parameter("max_linear_velocity").value),
            max_angular_velocity=float(self.get_parameter("max_angular_velocity").value),
            workspace_min=list(self.get_parameter("workspace_min").value),
            workspace_max=list(self.get_parameter("workspace_max").value),
            joint_names=list(self.get_parameter("joint_names").value),
            min_joint_position=list(self.get_parameter("min_joint_position").value),
            max_joint_position=list(self.get_parameter("max_joint_position").value),
            max_joint_velocity=list(self.get_parameter("max_joint_velocity").value),
        )

    def _make_filter_config(self, limits: SafetyLimits) -> FilterConfig:
        return FilterConfig(
            deadband=float(self.get_parameter("deadband").value),
            smoothing_alpha=float(self.get_parameter("smoothing_alpha").value),
            max_linear_velocity=limits.max_linear_velocity,
            max_angular_velocity=limits.max_angular_velocity,
            max_joint_velocity=limits.max_joint_velocity,
            min_joint_position=limits.min_joint_position,
            max_joint_position=limits.max_joint_position,
        )

    @staticmethod
    def _mode_from_teleop_value(value: int) -> ControlMode | None:
        return {
            TeleopCommand.IDLE: ControlMode.IDLE,
            TeleopCommand.JOINT_POSITION: ControlMode.JOINT_POSITION,
            TeleopCommand.JOINT_VELOCITY: ControlMode.JOINT_VELOCITY,
            TeleopCommand.CARTESIAN_POSE: ControlMode.CARTESIAN_POSE,
            TeleopCommand.CARTESIAN_TWIST: ControlMode.CARTESIAN_TWIST,
            TeleopCommand.GRIPPER: ControlMode.GRIPPER,
        }.get(value)

    @staticmethod
    def _teleop_value_from_mode(mode: ControlMode) -> int:
        return {
            ControlMode.IDLE: TeleopStatus.IDLE,
            ControlMode.JOINT_POSITION: TeleopStatus.JOINT_POSITION,
            ControlMode.JOINT_VELOCITY: TeleopStatus.JOINT_VELOCITY,
            ControlMode.CARTESIAN_POSE: TeleopStatus.CARTESIAN_POSE,
            ControlMode.CARTESIAN_TWIST: TeleopStatus.CARTESIAN_TWIST,
            ControlMode.GRIPPER: TeleopStatus.GRIPPER,
        }[mode]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TeleopManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
