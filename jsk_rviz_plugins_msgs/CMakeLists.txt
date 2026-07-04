cmake_minimum_required(VERSION 3.16)
project(jsk_rviz_plugins_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectFitCommand.msg"
  "msg/OverlayMenu.msg"
  "msg/OverlayText.msg"
  "msg/Pictogram.msg"
  "msg/PictogramArray.msg"
  "msg/RecordCommand.msg"
  "msg/StringStamped.msg"
  "msg/TransformableMarkerOperate.msg"
  "srv/EusCommand.srv"
  "srv/RequestMarkerOperate.srv"
  "srv/Screenshot.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
