"""Publish a unified teleoperation observation stream."""

from __future__ import annotations

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState

from teleop_msgs.msg import RobotStatus, TeleopObservation
from tf2_ros import Buffer, TransformException, TransformListener


class ObservationPublisher(Node):
    """Collect robot state topics into one TeleopObservation message."""

    def __init__(self) -> None:
        super().__init__("observation_publisher")

        self._declare_parameters()

        self.robot_name = self.get_parameter("robot_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.end_effector_frame = self.get_parameter("end_effector_frame").value
        self.joint_state_timeout_s = float(
            self.get_parameter("joint_state_timeout_s").value
        )
        self.sequence_id = 0
        self.last_joint_state: JointState | None = None
        self.last_joint_state_time: Time | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.observation_pub = self.create_publisher(
            TeleopObservation,
            self.get_parameter("observation_topic").value,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self.on_joint_state,
            10,
        )

        publish_frequency = float(self.get_parameter("publish_frequency").value)
        timer_period = 1.0 / max(publish_frequency, 1.0)
        self.create_timer(timer_period, self.publish_observation)

        self.get_logger().info(
            "publishing unified observations on "
            f"{self.get_parameter('observation_topic').value}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.last_joint_state = msg
        self.last_joint_state_time = self.get_clock().now()

    def publish_observation(self) -> None:
        if self.last_joint_state is None:
            return

        now = self.get_clock().now()
        msg = TeleopObservation()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        msg.robot_name = self.robot_name
        msg.sequence_id = self.sequence_id
        self.sequence_id += 1

        msg.joint_state = self.last_joint_state
        msg.robot_status = self._make_robot_status(now)

        transform = self._lookup_end_effector_transform()
        if transform is not None:
            msg.end_effector_state.header.stamp = msg.header.stamp
            msg.end_effector_state.header.frame_id = self.base_frame
            msg.end_effector_state.pose.position.x = transform.transform.translation.x
            msg.end_effector_state.pose.position.y = transform.transform.translation.y
            msg.end_effector_state.pose.position.z = transform.transform.translation.z
            msg.end_effector_state.pose.orientation = transform.transform.rotation

        self.observation_pub.publish(msg)

    def _make_robot_status(self, now: Time) -> RobotStatus:
        status = RobotStatus()
        status.header.stamp = now.to_msg()
        status.header.frame_id = self.base_frame
        status.control_mode = RobotStatus.IDLE
        status.error_code = 0
        status.error_message = ""
        status.emergency_stop = False
        status.protective_stop = False
        status.is_moving = self._is_robot_moving()
        status.is_enabled = self._joint_state_alive(now)
        status.is_connected = self._joint_state_alive(now)
        return status

    def _is_robot_moving(self) -> bool:
        if self.last_joint_state is None:
            return False
        return any(abs(velocity) > 1e-4 for velocity in self.last_joint_state.velocity)

    def _joint_state_alive(self, now: Time) -> bool:
        if self.last_joint_state_time is None:
            return False
        age_s = (now - self.last_joint_state_time).nanoseconds * 1e-9
        return age_s <= self.joint_state_timeout_s

    def _lookup_end_effector_transform(self) -> TransformStamped | None:
        if not self.end_effector_frame:
            return None

        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException:
            return None

    def _declare_parameters(self) -> None:
        self.declare_parameter("robot_name", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_effector_frame", "tool0")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("observation_topic", "/teleop/observation")
        self.declare_parameter("publish_frequency", 50.0)
        self.declare_parameter("joint_state_timeout_s", 1.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObservationPublisher()
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
