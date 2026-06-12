std_msgs/Header header

uint8 IDLE=0
uint8 JOINT_POSITION=1
uint8 JOINT_VELOCITY=2
uint8 CARTESIAN_POSE=3
uint8 CARTESIAN_TWIST=4
uint8 GRIPPER=5

uint8 control_mode
int32 error_code
string error_message

bool emergency_stop
bool protective_stop
bool is_moving
bool is_enabled
bool is_connected
