cmake_minimum_required(VERSION 3.16)
project(image_view2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MouseEvent.msg"
  "msg/PointArrayStamped.msg"
  "msg/ImageMarker2.msg"
  "srv/ChangeMode.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# image_view2 node (Phase 1: minimal viewer). Uses this package's own
# generated interfaces via the C++ typesupport target.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Target name must differ from the rosidl interface target (== PROJECT_NAME),
# so build as image_view2_node but install the binary as "image_view2".
add_executable(image_view2_node
  src/image_view2_node.cpp
  src/image_view2.cpp)
set_target_properties(image_view2_node PROPERTIES OUTPUT_NAME image_view2)
target_include_directories(image_view2_node PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_view2_node
  ${cpp_typesupport_target}
  ${OpenCV_LIBS})
ament_target_dependencies(image_view2_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  cv_bridge
  image_transport
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS image_view2_node
  DESTINATION lib/${PROJECT_NAME})

# points_rectangle_extractor node (Phase 5): message_filters only, no PCL.
add_executable(points_rectangle_extractor
  src/points_rectangle_extractor.cpp)
ament_target_dependencies(points_rectangle_extractor
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters)

install(TARGETS points_rectangle_extractor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_image_view2.py
    TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
