cmake_minimum_required(VERSION 3.16)
project(jsk_gui_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/DeviceSensor.msg"
  "msg/Tablet.msg"
  "msg/Touch.msg"
  "srv/YesNo.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
