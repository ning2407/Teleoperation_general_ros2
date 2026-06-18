#!/usr/bin/env python3
"""Keyboard input hardware that publishes generic teleoperation commands."""

from __future__ import annotations

import sys
import termios
import tty
from copy import deepcopy
from enum import Enum
from math import cos, sin

from controller_manager_msgs.srv import ListControllers, SwitchController
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from teleop_msgs.msg import TeleopCommand, TeleopObservation


HELP = """
Keyboard Servo Hardware

w/s : +X / -X
a/d : +Y / -Y
q/e : +Z / -Z

i/k : +Rx / -Rx
j/l : +Ry / -Ry
u/o : +Rz / -Rz

space : stop
h     : go to home
c     : toggle trajectory/servo controller
r     : record current pose as home
z     : record current pose as zero/neutral
x     : stop and exit
"""


class ControllerMode(Enum):
    UNKNOWN = "unknown"
    TRAJECTORY = "trajectory"
    SERVO = "servo"


class KeyboardServoHardware(Node):
    """Convert keyboard key presses into TeleopCommand messages."""

    def __init__(self) -> None:
        super().__init__("keyboard_servo")

        self.declare_parameter("command_topic", "/teleop/command")
        self.declare_parameter("robot_name", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("linear_step", 5.0)
        self.declare_parameter("angular_step", 2.0)
        self.declare_parameter("enable", True)
        self.declare_parameter("observation_topic", "/teleop/observation")
        self.declare_parameter("current_pose_timeout_s", 1.0)
        self.declare_parameter("home_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("home_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("home_rpy", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter(
            "home_joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter(
            "home_joint_positions",
            [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0],
        )
        self.declare_parameter("home_duration_s", 3.0)
        self.declare_parameter(
            "home_trajectory_action",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("controller_manager_prefix", "/controller_manager")
        self.declare_parameter("trajectory_controller", "joint_trajectory_controller")
        self.declare_parameter("servo_controller", "forward_position_controller")
        self.declare_parameter("switch_controller_timeout_s", 2.0)
        self.declare_parameter("auto_switch_for_home", True)
        self.declare_parameter("return_to_servo_after_home", False)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "servo_command_topic",
            "/forward_position_controller/commands",
        )
        self.declare_parameter("seed_servo_on_switch", True)
        self.declare_parameter("servo_seed_publish_count", 5)
        self.declare_parameter("servo_seed_publish_period_s", 0.05)
        self.declare_parameter("manage_servo_node_on_switch", True)
        self.declare_parameter("servo_node_prefix", "/servo_node")

        self.command_topic = self.get_parameter("command_topic").value
        self.observation_topic = self.get_parameter("observation_topic").value
        self.robot_name = self.get_parameter("robot_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        self.enable = bool(self.get_parameter("enable").value)
        self.current_pose_timeout_s = float(
            self.get_parameter("current_pose_timeout_s").value
        )
        self.sequence_id = 0
        self.current_pose: PoseStamped | None = None
        self.current_pose_time = None
        self.zero_pose: PoseStamped | None = None
        self.home_pose = self._make_home_pose()
        self.home_joint_names = list(self.get_parameter("home_joint_names").value)
        self.home_joint_positions = self._double_array_parameter("home_joint_positions")
        self.home_duration_s = float(self.get_parameter("home_duration_s").value)
        self.home_trajectory_action = self.get_parameter("home_trajectory_action").value
        self.controller_manager_prefix = self.get_parameter(
            "controller_manager_prefix"
        ).value.rstrip("/")
        self.trajectory_controller = self.get_parameter("trajectory_controller").value
        self.servo_controller = self.get_parameter("servo_controller").value
        self.switch_controller_timeout_s = float(
            self.get_parameter("switch_controller_timeout_s").value
        )
        self.auto_switch_for_home = bool(self.get_parameter("auto_switch_for_home").value)
        self.return_to_servo_after_home = bool(
            self.get_parameter("return_to_servo_after_home").value
        )
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.servo_command_topic = self.get_parameter("servo_command_topic").value
        self.seed_servo_on_switch = bool(
            self.get_parameter("seed_servo_on_switch").value
        )
        self.servo_seed_publish_count = int(
            self.get_parameter("servo_seed_publish_count").value
        )
        self.servo_seed_publish_period_s = float(
            self.get_parameter("servo_seed_publish_period_s").value
        )
        self.manage_servo_node_on_switch = bool(
            self.get_parameter("manage_servo_node_on_switch").value
        )
        self.servo_node_prefix = self.get_parameter("servo_node_prefix").value.rstrip("/")
        self.last_joint_state: JointState | None = None

        self.pub = self.create_publisher(TeleopCommand, self.command_topic, 10)
        self.servo_command_pub = self.create_publisher(
            Float64MultiArray,
            self.servo_command_topic,
            10,
        )
        self.home_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.home_trajectory_action,
        )
        self.list_controllers_client = self.create_client(
            ListControllers,
            f"{self.controller_manager_prefix}/list_controllers",
        )
        self.switch_controller_client = self.create_client(
            SwitchController,
            f"{self.controller_manager_prefix}/switch_controller",
        )
        self.start_servo_client = self.create_client(
            Trigger,
            f"{self.servo_node_prefix}/start_servo",
        )
        self.stop_servo_client = self.create_client(
            Trigger,
            f"{self.servo_node_prefix}/stop_servo",
        )
        self.reset_servo_status_client = self.create_client(
            Empty,
            f"{self.servo_node_prefix}/reset_servo_status",
        )
        self.create_subscription(
            TeleopObservation,
            self.observation_topic,
            self.on_observation,
            10,
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.on_joint_state,
            10,
        )
        self.get_logger().info(HELP)
        self.get_logger().info(f"publishing TeleopCommand on {self.command_topic}")
        self.get_logger().info(f"reading current EE pose from {self.observation_topic}")
        self.get_logger().info(
            f"sending joint home goals to {self.home_trajectory_action}"
        )
        self.get_logger().info(
            "controller toggle: %s <-> %s"
            % (self.trajectory_controller, self.servo_controller)
        )
        self.get_logger().info(
            f"seeding servo commands on {self.servo_command_topic}"
        )

    def publish_command(
        self,
        linear: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
        *,
        mode: int = TeleopCommand.CARTESIAN_TWIST,
        pose: PoseStamped | None = None,
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
        if mode == TeleopCommand.CARTESIAN_POSE and pose is not None:
            target_pose = deepcopy(pose)
            target_pose.header.stamp = now
            if not target_pose.header.frame_id:
                target_pose.header.frame_id = self.frame_id
            msg.command_frame = target_pose.header.frame_id
            msg.target_pose = target_pose
        else:
            msg.target_twist = twist

        self.sequence_id += 1
        self.pub.publish(msg)

        if mode == TeleopCommand.CARTESIAN_POSE and pose is not None:
            position = pose.pose.position
            orientation = pose.pose.orientation
            self.get_logger().info(
                "pose xyz=(%.3f, %.3f, %.3f) qxyzw=(%.3f, %.3f, %.3f, %.3f)"
                % (
                    position.x,
                    position.y,
                    position.z,
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            )
        else:
            self.get_logger().info(
                "linear=(%.3f, %.3f, %.3f) angular=(%.3f, %.3f, %.3f)"
                % (*linear, *angular)
            )

    def on_observation(self, msg: TeleopObservation) -> None:
        if not msg.end_effector_state.header.frame_id:
            return

        pose = PoseStamped()
        pose.header = msg.end_effector_state.header
        pose.pose = msg.end_effector_state.pose
        self.current_pose = pose
        self.current_pose_time = self.get_clock().now()

    def on_joint_state(self, msg: JointState) -> None:
        self.last_joint_state = msg

    def publish_home_pose(self) -> None:
        if self.home_joint_names and self.home_joint_positions:
            self.send_home_joint_trajectory()
            return

        if self.home_pose is None:
            self.get_logger().warning(
                "home pose is not set; configure home_position or press r after "
                "a current pose is available"
            )
            return

        self.publish_command(mode=TeleopCommand.CARTESIAN_POSE, pose=self.home_pose)

    def send_home_joint_trajectory(self) -> None:
        if len(self.home_joint_names) != len(self.home_joint_positions):
            self.get_logger().error(
                "home_joint_names and home_joint_positions must have the same length"
            )
            return

        restore_servo = False
        if self.auto_switch_for_home:
            mode = self.get_controller_mode()
            restore_servo = (
                mode == ControllerMode.SERVO and self.return_to_servo_after_home
            )
            if mode != ControllerMode.TRAJECTORY:
                self.stop_moveit_servo()
                if not self.switch_controllers(
                    activate=[self.trajectory_controller],
                    deactivate=[self.servo_controller],
                ):
                    return

        if not self.home_action_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().error(
                f"home action server is not available: {self.home_trajectory_action}"
            )
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.home_joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(self.home_joint_positions)
        point.time_from_start.sec = int(self.home_duration_s)
        point.time_from_start.nanosec = int(
            (self.home_duration_s - int(self.home_duration_s)) * 1e9
        )
        goal.trajectory.points = [point]

        self.get_logger().info(
            "sent home joint trajectory: positions=%s duration=%.2fs"
            % (list(self.home_joint_positions), self.home_duration_s)
        )
        future = self.home_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("home joint trajectory was rejected")
            return

        self.get_logger().info("home joint trajectory accepted")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            "home joint trajectory finished: error_code=%s error_string=%s"
            % (result.error_code, result.error_string)
        )
        if restore_servo:
            self.switch_to_servo_controller()

    def toggle_motion_controller(self) -> None:
        mode = self.get_controller_mode()
        if mode == ControllerMode.TRAJECTORY:
            self.switch_to_servo_controller()
        else:
            self.stop_moveit_servo()
            self.switch_controllers(
                activate=[self.trajectory_controller],
                deactivate=[self.servo_controller],
            )

    def switch_to_servo_controller(self) -> bool:
        self.stop_moveit_servo()
        seed_positions = self.current_joint_positions_for_servo()
        if self.seed_servo_on_switch and seed_positions is not None:
            self.publish_servo_seed(seed_positions, count=1)

        switched = self.switch_controllers(
            activate=[self.servo_controller],
            deactivate=[self.trajectory_controller],
        )
        if not switched:
            return False

        if self.seed_servo_on_switch and seed_positions is not None:
            self.publish_servo_seed(seed_positions)
        self.reset_moveit_servo_status()
        self.start_moveit_servo()
        return True

    def stop_moveit_servo(self) -> bool:
        if not self.manage_servo_node_on_switch:
            return True
        return self.call_trigger_service(self.stop_servo_client, "stop servo")

    def start_moveit_servo(self) -> bool:
        if not self.manage_servo_node_on_switch:
            return True
        return self.call_trigger_service(self.start_servo_client, "start servo")

    def reset_moveit_servo_status(self) -> bool:
        if not self.manage_servo_node_on_switch:
            return True
        if not self.reset_servo_status_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning("reset servo status service is not available")
            return False
        future = self.reset_servo_status_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is None:
            self.get_logger().warning("reset servo status service call failed")
            return False
        self.get_logger().info("reset servo status")
        return True

    def call_trigger_service(self, client, label: str) -> bool:
        if not client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning("%s service is not available" % label)
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        response = future.result()
        if response is None:
            self.get_logger().warning("%s service call failed" % label)
            return False
        if not response.success:
            self.get_logger().warning("%s returned false: %s" % (label, response.message))
            return False
        self.get_logger().info("%s: %s" % (label, response.message))
        return True

    def current_joint_positions_for_servo(self) -> list[float] | None:
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.last_joint_state is not None:
                break

        if self.last_joint_state is None:
            self.get_logger().warning(
                "joint state is unavailable; cannot seed servo controller"
            )
            return None

        joint_positions = dict(
            zip(self.last_joint_state.name, self.last_joint_state.position)
        )
        missing = [name for name in self.home_joint_names if name not in joint_positions]
        if missing:
            self.get_logger().warning(
                "joint state is missing servo joints: %s" % missing
            )
            return None

        return [float(joint_positions[name]) for name in self.home_joint_names]

    def publish_servo_seed(
        self,
        positions: list[float],
        *,
        count: int | None = None,
    ) -> None:
        publish_count = count if count is not None else self.servo_seed_publish_count
        msg = Float64MultiArray()
        msg.data = list(positions)
        for _ in range(max(publish_count, 1)):
            self.servo_command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=self.servo_seed_publish_period_s)
        self.get_logger().info("seeded servo target with current joints: %s" % positions)

    def get_controller_mode(self) -> ControllerMode:
        if not self.list_controllers_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error(
                "list_controllers service is not available: "
                f"{self.controller_manager_prefix}/list_controllers"
            )
            return ControllerMode.UNKNOWN

        future = self.list_controllers_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is None:
            self.get_logger().error("failed to list controllers")
            return ControllerMode.UNKNOWN

        states = {controller.name: controller.state for controller in future.result().controller}
        trajectory_active = states.get(self.trajectory_controller) == "active"
        servo_active = states.get(self.servo_controller) == "active"

        if trajectory_active:
            self.get_logger().info(f"active controller: {self.trajectory_controller}")
            return ControllerMode.TRAJECTORY
        if servo_active:
            self.get_logger().info(f"active controller: {self.servo_controller}")
            return ControllerMode.SERVO

        self.get_logger().warning(
            "neither configured motion controller is active: %s"
            % states
        )
        return ControllerMode.UNKNOWN

    def switch_controllers(
        self,
        *,
        activate: list[str],
        deactivate: list[str],
    ) -> bool:
        if not self.switch_controller_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(
                "switch_controller service is not available: "
                f"{self.controller_manager_prefix}/switch_controller"
            )
            return False

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(self.switch_controller_timeout_s)
        request.timeout.nanosec = int(
            (
                self.switch_controller_timeout_s
                - int(self.switch_controller_timeout_s)
            )
            * 1e9
        )

        self.get_logger().info(
            "switching controllers: activate=%s deactivate=%s"
            % (activate, deactivate)
        )
        future = self.switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.switch_controller_timeout_s + 0.5,
        )
        response = future.result()
        if response is None or not response.ok:
            self.get_logger().error("controller switch failed")
            return False

        self.get_logger().info("controller switch succeeded")
        return True

    def record_current_as_home(self) -> None:
        current = self._fresh_current_pose()
        if current is None:
            return
        self.home_pose = deepcopy(current)
        self.get_logger().info("recorded current end-effector pose as home")

    def record_current_as_zero(self) -> None:
        current = self._fresh_current_pose()
        if current is None:
            return
        self.zero_pose = deepcopy(current)
        self.get_logger().info("recorded current end-effector pose as zero/neutral")

    def _fresh_current_pose(self) -> PoseStamped | None:
        if self.current_pose is None or self.current_pose_time is None:
            self.get_logger().warning(
                "current end-effector pose is not available yet; "
                "check observation_publisher and TF"
            )
            return None

        age_s = (self.get_clock().now() - self.current_pose_time).nanoseconds * 1e-9
        if age_s > self.current_pose_timeout_s:
            self.get_logger().warning(
                "current end-effector pose is stale: %.3fs old" % age_s
            )
            return None
        return self.current_pose

    def _make_home_pose(self) -> PoseStamped | None:
        position = self._double_array_parameter("home_position")
        if not position:
            return None
        if len(position) != 3:
            self.get_logger().warning("home_position must contain exactly 3 values")
            return None

        orientation = self._home_orientation_xyzw()
        if orientation is None:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose = Pose()
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        return pose

    def _home_orientation_xyzw(self) -> tuple[float, float, float, float] | None:
        rpy = self._double_array_parameter("home_rpy")
        if rpy:
            if len(rpy) != 3:
                self.get_logger().warning("home_rpy must contain exactly 3 values")
                return None
            return self._quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        orientation = self._double_array_parameter("home_orientation_xyzw")
        if len(orientation) != 4:
            self.get_logger().warning(
                "home_orientation_xyzw must contain exactly 4 values"
            )
            return None
        return tuple(float(value) for value in orientation)

    def _double_array_parameter(self, name: str) -> list[float]:
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        if value is None:
            return []
        return [float(item) for item in value]

    @staticmethod
    def _quaternion_from_rpy(
        roll: float,
        pitch: float,
        yaw: float,
    ) -> tuple[float, float, float, float]:
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
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
            rclpy.spin_once(self, timeout_sec=0.0)

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
            elif key == "h":
                self.publish_home_pose()
            elif key == "c":
                self.toggle_motion_controller()
            elif key == "r":
                self.record_current_as_home()
            elif key == "z":
                self.record_current_as_zero()
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
