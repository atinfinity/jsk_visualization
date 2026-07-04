cmake_minimum_required(VERSION 3.16)
project(jsk_interactive_marker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(jsk_interactive_marker_msgs REQUIRED)
find_package(jsk_rviz_plugins_msgs REQUIRED)
find_package(jsk_recognition_msgs REQUIRED)
find_package(jsk_footstep_msgs REQUIRED)
find_package(jsk_visualization_utils REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

set(COMMON_DEPS_TARGETS
  rclcpp::rclcpp
  interactive_markers::interactive_markers
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_kdl::tf2_kdl
  urdf::urdf
  ament_index_cpp::ament_index_cpp
  yaml-cpp::yaml-cpp
  orocos-kdl
  jsk_visualization_utils::jsk_visualization_utils
  ${jsk_interactive_marker_msgs_TARGETS}
  ${jsk_rviz_plugins_msgs_TARGETS}
  ${jsk_recognition_msgs_TARGETS}
  ${jsk_footstep_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})

# shared library: helpers/utils used by most executables
add_library(${PROJECT_NAME} SHARED
  src/interactive_marker_helpers.cpp
  src/interactive_marker_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_DEPS_TARGETS})

# helper macro for node executables
macro(im_add_executable target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE ${PROJECT_NAME} ${COMMON_DEPS_TARGETS})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endmacro()

# Group (c) pilots
im_add_executable(marker_6dof src/marker_6dof.cpp)
im_add_executable(bounding_box_marker src/bounding_box_marker.cpp)

# Group (b): transformable core
add_library(transformable_core SHARED
  src/transformable_object.cpp
  src/yaml_menu_handler.cpp
  src/parent_and_child_interactive_marker_server.cpp
  src/transformable_interactive_server.cpp)
target_include_directories(transformable_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(transformable_core PUBLIC ${COMMON_DEPS_TARGETS})
install(TARGETS transformable_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

im_add_executable(transformable_server_sample src/transformable_server_sample.cpp)
target_link_libraries(transformable_server_sample PRIVATE transformable_core)

# rviz panel plugin
add_library(jsk_interactive_marker_rviz_plugins SHARED
  src/rviz_plugins/transformable_marker_operator.cpp)
target_include_directories(jsk_interactive_marker_rviz_plugins PRIVATE include)
target_compile_definitions(jsk_interactive_marker_rviz_plugins PRIVATE QT_NO_KEYWORDS)
target_link_libraries(jsk_interactive_marker_rviz_plugins PUBLIC
  Qt5::Widgets
  rviz_common::rviz_common
  ${COMMON_DEPS_TARGETS})
install(TARGETS jsk_interactive_marker_rviz_plugins
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# --- ported group by group ---
# Group (c): point_cloud_config_marker triangle_foot door_foot polygon_marker world2yaml
# Group (d): interactive_point_cloud pointcloud_cropper
# Group (e): urdf_model_marker urdf_control_marker interactive_marker_interface camera_info_publisher
# Group (f): footstep_marker

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config models urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rclcpp_action interactive_markers tf2 tf2_ros tf2_eigen
  tf2_geometry_msgs tf2_kdl urdf ament_index_cpp yaml-cpp orocos_kdl_vendor
  jsk_interactive_marker_msgs jsk_rviz_plugins_msgs jsk_recognition_msgs
  jsk_footstep_msgs jsk_visualization_utils visualization_msgs geometry_msgs
  sensor_msgs std_msgs std_srvs)
ament_package()
