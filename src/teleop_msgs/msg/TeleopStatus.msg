std_msgs/Header header

string robot_name

uint8 IDLE=0
uint8 JOINT_POSITION=1
uint8 JOINT_VELOCITY=2
uint8 CARTESIAN_POSE=3
uint8 CARTESIAN_TWIST=4
uint8 GRIPPER=5

uint8 active_mode
bool enabled
bool emergency_stop
bool safety_ok
bool input_alive
bool command_timeout

float64 command_age

int32 error_code
string last_stop_reason
string message
