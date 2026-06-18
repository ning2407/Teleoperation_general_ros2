# teleoperation_general

`teleoperation_general` is a ROS 2 teleoperation package for driving a robot arm through a generic command pipeline. It provides:

- `teleop_manager`: validates, filters, frame-transforms, and forwards teleoperation commands.
- `observation_publisher`: publishes a unified robot observation stream from joint states and TF.
- `keyboard_servo`: interactive keyboard control, home motion, and controller switching.

The current setup is designed for a MoveIt Servo + ros2_control workflow using:

- `joint_trajectory_controller` for planned/home joint motion.
- `forward_position_controller` for Servo position command output.
- `/servo_node` for MoveIt Servo.

## Build

From the workspace root:

```bash
colcon build --packages-select teleoperation_general
source install/setup.bash
```

Rebuild and re-source after editing Python files, launch files, or YAML config.

## Launch Core Nodes

Start the manager and observation publisher:

```bash
ros2 launch teleoperation_general teleop_core.launch.py \
  start_observation:=true
```

`keyboard_servo` is not launched here because it needs an interactive terminal for key input.

## Start Keyboard Control

Open a second terminal:

```bash
cd /home/ning24/workspace/docker/ensampr1
source install/setup.bash

ros2 run teleoperation_general keyboard_servo \
  --ros-args \
  --params-file install/teleoperation_general/share/teleoperation_general/config/teleop_core.yaml
```

## Keyboard Bindings

```text
w / s : +X / -X
a / d : +Y / -Y
q / e : +Z / -Z

i / k : +Rx / -Rx
j / l : +Ry / -Ry
u / o : +Rz / -Rz

space : stop
h     : go to home
c     : toggle trajectory/servo controller
r     : record current end-effector pose as home
z     : record current end-effector pose as zero/neutral
x     : stop and exit
```

## Home Motion

The `h` key sends a `FollowJointTrajectory` goal to:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

The home joint target is configured in `config/teleop_core.yaml`:

```yaml
home_joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
home_joint_positions: [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]
home_duration_s: 3.0
```

When `auto_switch_for_home` is enabled, pressing `h` automatically switches from Servo control to `joint_trajectory_controller` before sending the home trajectory.

By default, `return_to_servo_after_home` is `false`, so the robot stays under `joint_trajectory_controller` after reaching home. Press `c` when you want to switch back to Servo control.

## Controller Switching

The `c` key toggles between:

```text
joint_trajectory_controller <-> forward_position_controller
```

This is implemented through controller manager services:

```text
/controller_manager/list_controllers
/controller_manager/switch_controller
```

Relevant config:

```yaml
controller_manager_prefix: /controller_manager
trajectory_controller: joint_trajectory_controller
servo_controller: forward_position_controller
switch_controller_timeout_s: 2.0
```

When switching back to `forward_position_controller`, the node reads the current `/joint_states` and publishes the current joint positions to:

```text
/forward_position_controller/commands
```

This seeds the forward controller with the current pose so it does not chase an old command.

The node can also stop/reset/start MoveIt Servo during controller switching:

```yaml
manage_servo_node_on_switch: true
servo_node_prefix: /servo_node
```

It uses these services:

```text
/servo_node/stop_servo
/servo_node/reset_servo_status
/servo_node/start_servo
```

## Main Topics

Input:

```text
/teleop/command
/joint_states
/teleop/observation
```

Validated outputs:

```text
/teleop/validated/joint_velocity
/teleop/validated/joint_position
/servo_node/delta_twist_cmds
/teleop/validated/cartesian_pose
```

Servo/ros2_control:

```text
/forward_position_controller/commands
/joint_trajectory_controller/follow_joint_trajectory
```

Status:

```text
/teleop/status
```

## Safety Limits

`teleop_manager` checks velocity, workspace, joint limits, and command freshness.

Important values in `config/teleop_core.yaml`:

```yaml
max_linear_velocity: 0.15
max_angular_velocity: 0.4
workspace_min: [-1.0, -1.0, 0.0]
workspace_max: [1.0, 1.0, 1.2]
command_timeout_s: 0.2
```

If `keyboard_servo.linear_step` is larger than `max_linear_velocity`, the command may be rejected by `teleop_manager`.

## Useful Checks

List controllers:

```bash
ros2 control list_controllers
```

Check who is publishing to the forward controller:

```bash
ros2 topic info /forward_position_controller/commands -v
```

Check MoveIt Servo services:

```bash
ros2 node info /servo_node
```

Watch validated twist commands:

```bash
ros2 topic echo /servo_node/delta_twist_cmds
```

Watch teleop status:

```bash
ros2 topic echo /teleop/status
```

## Common Issues

### `keyboard_servo requires an interactive terminal`

Do not launch `keyboard_servo` through `ros2 launch`. Run it in a separate terminal with `ros2 run`.

### Changed YAML but values did not update

If running with:

```bash
--params-file install/teleoperation_general/share/teleoperation_general/config/teleop_core.yaml
```

then rebuild after editing `src/teleoperation_general/config/teleop_core.yaml`:

```bash
colcon build --packages-select teleoperation_general
source install/setup.bash
```

### Robot jumps back after switching to Servo

This usually means `/servo_node` is still publishing old commands to `/forward_position_controller/commands`. Keep:

```yaml
manage_servo_node_on_switch: true
seed_servo_on_switch: true
```

The keyboard node will stop Servo, seed the current joint target, switch controllers, then restart Servo.

