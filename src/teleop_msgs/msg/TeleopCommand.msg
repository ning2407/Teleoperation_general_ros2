std_msgs/Header header

string robot_name
uint32 sequence_id

uint8 IDLE=0
uint8 JOINT_POSITION=1
uint8 JOINT_VELOCITY=2
uint8 CARTESIAN_POSE=3
uint8 CARTESIAN_TWIST=4
uint8 GRIPPER=5

uint8 control_mode
bool enable
bool emergency_stop

string command_frame

sensor_msgs/JointState joint_command
geometry_msgs/PoseStamped target_pose
geometry_msgs/TwistStamped target_twist
teleop_msgs/GripperCommand gripper_command
