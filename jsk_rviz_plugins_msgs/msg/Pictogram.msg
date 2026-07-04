uint8 ACTION_ADD = 0
uint8 ACTION_DELETE = 1
uint8 ACTION_ROTATE_Z = 2
uint8 ACTION_ROTATE_Y = 3
uint8 ACTION_ROTATE_X = 4
uint8 ACTION_JUMP = 5
uint8 ACTION_JUMP_ONCE = 6

uint8 MODE_PICTOGRAM = 0
uint8 MODE_STRING = 1

std_msgs/Header header
geometry_msgs/Pose pose
uint8 action
uint8 mode
string character
float64 size
float64 ttl
float64 speed
std_msgs/ColorRGBA color
