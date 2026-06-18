# 遥操作包说明

这个 `src` 目录里主要包含机械臂遥操作相关的 ROS 2 包。目前核心包是 `teleoperation_general`，它负责把键盘输入转换成通用遥操作命令，并通过 `teleop_manager` 做安全检查、模式管理和转发。

当前工作流适配的是：

- MoveIt Servo
- ros2_control
- `joint_trajectory_controller`
- `forward_position_controller`
- `/servo_node`

## 主要包

```text
teleoperation_general
teleop_msgs
keyboard_servo
my_test
```

其中当前主要使用的是：

```text
teleoperation_general
teleop_msgs
```

`src/keyboard_servo` 是早期单独发布 `TwistStamped` 的键盘包，现在主要使用 `teleoperation_general` 里的 `keyboard_servo`。

## teleoperation_general 组成

```text
teleop_manager
observation_publisher
keyboard_servo
```

### teleop_manager

订阅：

```text
/teleop/command
```

负责：

- 接收通用遥操作命令
- 检查控制模式
- 做速度、工作空间、关节范围等安全检查
- 对 twist / pose 做坐标系转换
- 输出到 Servo 或其他控制接口

主要输出：

```text
/servo_node/delta_twist_cmds
/teleop/validated/cartesian_pose
/teleop/validated/joint_velocity
/teleop/validated/joint_position
```

### observation_publisher

订阅：

```text
/joint_states
TF: base_link -> tool0
```

发布：

```text
/teleop/observation
```

用于给键盘节点和其他遥操作设备提供当前机械臂状态。

### keyboard_servo

这是当前主要的键盘遥操作节点。

功能：

- 键盘控制末端速度
- 按 `h` 回到 home 关节位姿
- 按 `c` 切换 controller
- 按 `r` 记录当前末端位姿为 home
- 按 `z` 记录当前末端位姿为 zero / neutral

注意：`keyboard_servo` 需要交互式终端，所以不要放到 `ros2 launch` 里启动，要单独用 `ros2 run`。

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select teleoperation_general teleop_msgs
source install/setup.bash
```

如果只改了 `teleoperation_general`：

```bash
colcon build --packages-select teleoperation_general
source install/setup.bash
```

## 启动

### 终端 1：启动 teleop 核心节点

```bash
cd /home/ning24/workspace/docker/ensampr1
source install/setup.bash

ros2 launch teleoperation_general teleop_core.launch.py \
  start_observation:=true
```

这个 launch 会启动：

```text
teleop_manager
observation_publisher
```

不会启动 `keyboard_servo`。

### 终端 2：启动键盘控制

```bash
cd /home/ning24/workspace/docker/ensampr1
source install/setup.bash

ros2 run teleoperation_general keyboard_servo \
  --ros-args \
  --params-file install/teleoperation_general/share/teleoperation_general/config/teleop_core.yaml
```

## 键盘按键

```text
w / s : +X / -X
a / d : +Y / -Y
q / e : +Z / -Z

i / k : +Rx / -Rx
j / l : +Ry / -Ry
u / o : +Rz / -Rz

space : 停止
h     : 回到 home
c     : 切换 joint_trajectory_controller / forward_position_controller
r     : 记录当前末端位姿为 home
z     : 记录当前末端位姿为 zero / neutral
x     : 停止并退出
```

## Home 位姿

当前 home 是关节位姿，不是笛卡尔位姿。

配置文件：

```text
src/teleoperation_general/config/teleop_core.yaml
```

对应参数：

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

按 `h` 时，节点会发送 action 到：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

等价于手动执行：

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
"{
  trajectory: {
    joint_names: [
      shoulder_pan_joint,
      shoulder_lift_joint,
      elbow_joint,
      wrist_1_joint,
      wrist_2_joint,
      wrist_3_joint
    ],
    points: [
      {
        positions: [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0],
        time_from_start: {sec: 3, nanosec: 0}
      }
    ]
  }
}"
```

## Controller 切换逻辑

当前使用两个 controller：

```text
joint_trajectory_controller
forward_position_controller
```

配置：

```yaml
controller_manager_prefix: /controller_manager
trajectory_controller: joint_trajectory_controller
servo_controller: forward_position_controller
```

按 `c` 时，`keyboard_servo` 会调用：

```text
/controller_manager/list_controllers
/controller_manager/switch_controller
```

逻辑：

```text
如果当前是 joint_trajectory_controller active：
    切换到 forward_position_controller
否则：
    切换到 joint_trajectory_controller
```

等价命令：

```bash
ros2 control switch_controllers \
  --activate forward_position_controller \
  --deactivate joint_trajectory_controller \
  --strict
```

或者反过来：

```bash
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate forward_position_controller \
  --strict
```

## 为什么切回 Servo 前要 seed 当前关节位置

`forward_position_controller` 的命令 topic 是：

```text
/forward_position_controller/commands
```

它使用的是：

```text
std_msgs/msg/Float64MultiArray
```

这个消息没有关节名，只按数组顺序解释位置。

如果 `servo_node` 或 controller 里还保留着旧目标，切回 `forward_position_controller` 时机械臂可能会跳回旧位置。

所以当前逻辑是：

```text
1. stop /servo_node
2. 读取当前 /joint_states
3. 发布当前关节位置到 /forward_position_controller/commands
4. 切换到 forward_position_controller
5. 再发布几帧当前关节位置
6. reset /servo_node 状态
7. start /servo_node
```

相关参数：

```yaml
joint_state_topic: /joint_states
servo_command_topic: /forward_position_controller/commands
seed_servo_on_switch: true
servo_seed_publish_count: 5
servo_seed_publish_period_s: 0.05
manage_servo_node_on_switch: true
servo_node_prefix: /servo_node
```

## 常用检查命令

查看 controller 状态：

```bash
ros2 control list_controllers
```

查看 `/forward_position_controller/commands` 的发布者：

```bash
ros2 topic info /forward_position_controller/commands -v
```

查看 Servo 节点服务：

```bash
ros2 node info /servo_node
```

查看键盘发出的通用命令：

```bash
ros2 topic echo /teleop/command
```

查看 Servo twist 输入：

```bash
ros2 topic echo /servo_node/delta_twist_cmds
```

查看遥操作状态：

```bash
ros2 topic echo /teleop/status
```

## 参数注意事项

键盘节点的参数在：

```text
src/teleoperation_general/config/teleop_core.yaml
```

但是运行时一般使用的是安装后的配置：

```text
install/teleoperation_general/share/teleoperation_general/config/teleop_core.yaml
```

所以修改 `src` 里的 YAML 后，需要重新编译：

```bash
colcon build --packages-select teleoperation_general
source install/setup.bash
```

否则运行时可能还是旧参数。

## 速度限制

键盘参数：

```yaml
linear_step: 8.0
angular_step: 5.0
```

`teleop_manager` 还有安全限幅：

```yaml
max_linear_velocity: 0.15
max_angular_velocity: 0.4
```

如果键盘发出的速度超过安全限幅，`teleop_manager` 可能会拒绝命令。

## 常见问题

### keyboard_servo requires an interactive terminal

原因：`keyboard_servo` 被放进 `ros2 launch` 启动了，launch 子进程没有交互式键盘输入。

解决：单独开终端运行：

```bash
ros2 run teleoperation_general keyboard_servo \
  --ros-args \
  --params-file install/teleoperation_general/share/teleoperation_general/config/teleop_core.yaml
```

### 修改 linear_step 后没有生效

原因：YAML 参数覆盖了 Python 里的默认值，或者修改了 `src` 但没有重新 build。

检查：

```bash
grep linear_step src/teleoperation_general/config/teleop_core.yaml
grep linear_step install/teleoperation_general/share/teleoperation_general/config/teleop_core.yaml
```

### 按 h 回 home 后，切回 Servo 又回到旧位置

原因：`servo_node` 还在向 `/forward_position_controller/commands` 发布旧目标。

解决：保持：

```yaml
manage_servo_node_on_switch: true
seed_servo_on_switch: true
```

并确保切换日志里能看到：

```text
stop servo
seeded servo target with current joints
start servo
```

