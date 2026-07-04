cmake_minimum_required(VERSION 3.16)
project(jsk_interactive_marker_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(jsk_recognition_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTrajectoryPointWithType.msg"
  "msg/JointTrajectoryWithType.msg"
  "msg/MarkerDimensions.msg"
  "msg/MarkerMenu.msg"
  "msg/MarkerPose.msg"
  "msg/MoveModel.msg"
  "msg/MoveObject.msg"
  "msg/PoseStampedWithName.msg"
  "msg/SnapFootPrintInput.msg"
  "srv/GetJointState.srv"
  "srv/GetMarkerDimensions.srv"
  "srv/GetTransformableMarkerColor.srv"
  "srv/GetTransformableMarkerExistence.srv"
  "srv/GetTransformableMarkerFocus.srv"
  "srv/GetTransformableMarkerPose.srv"
  "srv/GetType.srv"
  "srv/IndexRequest.srv"
  "srv/MarkerSetPose.srv"
  "srv/RemoveParentMarker.srv"
  "srv/SetHeuristic.srv"
  "srv/SetMarkerDimensions.srv"
  "srv/SetParentMarker.srv"
  "srv/SetPose.srv"
  "srv/SetTransformableMarkerColor.srv"
  "srv/SetTransformableMarkerFocus.srv"
  "srv/SetTransformableMarkerPose.srv"
  "srv/SnapFootPrint.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs visualization_msgs jsk_recognition_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
