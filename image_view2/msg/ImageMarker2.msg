# Marker type constants. Declared int32 (not the ROS 1 'byte') so they can
# be assigned to the int32 'type'/'action' fields below; the wire format of
# those fields is unchanged.
int32 CIRCLE=0
int32 LINE_STRIP=1
int32 LINE_LIST=2
int32 POLYGON=3
int32 POINTS=4
int32 FRAMES=5
int32 TEXT=6

int32 LINE_STRIP3D=7
int32 LINE_LIST3D=8
int32 POLYGON3D=9
int32 POINTS3D=10
int32 TEXT3D=11
int32 CIRCLE3D=12

int32 ADD=0
int32 REMOVE=1

std_msgs/Header header
string ns		# namespace, used with id to form a unique id
int32 id          	# unique id within the namespace
int32 type        	# CIRCLE/LINE_STRIP/etc.
int32 action      	# ADD/REMOVE
geometry_msgs/Point position # used for CIRCLE/TEXT, 2D in pixel-coords
geometry_msgs/PointStamped position_3d # used for 3DTEXT
geometry_msgs/PoseStamped pose # used for CIRCLE3D
float32 scale	 	# the diameter for a circle, etc.
float32 width	 	# the width for a line, etc.
std_msgs/ColorRGBA outline_color
byte filled		# whether to fill in the shape with color
std_msgs/ColorRGBA fill_color # color [0.0-1.0]
builtin_interfaces/Duration lifetime       # How long the object should last before being automatically deleted.  0 means forever
byte arc # used for CIRCLE3D
float32 angle # used for CIRCLE3D


geometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POLYGON/POINTS., 2D in pixel coords
PointArrayStamped points_3d # used for 3DLINE_STRIP/3DLINE_LIST/3DPOLYGON/3DPOINTS
std_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.

string[] frames # used for FRAMES, tf names
string text             # used for TEXT, draw size of text is scale
bool left_up_origin     # draw text from left up origin
bool ratio_scale        #Use ratio respected to original image to specify scale and position
