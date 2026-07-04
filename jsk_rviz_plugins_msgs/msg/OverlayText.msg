# Rich text drawn as a 2D overlay on the rviz render panel.
uint8 ACTION_ADD = 0
uint8 ACTION_DELETE = 1

uint8 action

# Geometry in pixels. width/height <= 0 means "auto size".
int32 width
int32 height
int32 left
int32 top

std_msgs/ColorRGBA bg_color
std_msgs/ColorRGBA fg_color

int32 line_width
float32 text_size
string font
string text
