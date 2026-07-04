cmake_minimum_required(VERSION 3.16)
project(jsk_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(jsk_rviz_plugins_msgs REQUIRED)
find_package(jsk_visualization_utils REQUIRED)
find_package(jsk_recognition_msgs REQUIRED)
find_package(jsk_footstep_msgs REQUIRED)
find_package(jsk_hark_msgs REQUIRED)
find_package(jsk_gui_msgs REQUIRED)
find_package(view_controller_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(people_msgs QUIET)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

# Phase 1 sources, grouped by shared infrastructure. Groups are enabled as
# their port to ROS 2 completes.
set(SOURCE_FILES
  # Group A: overlay foundation
  src/overlay_utils.cpp
  src/overlay_text_display.cpp
  src/string_display.cpp
  # Group B: 2D overlay family
  src/overlay_menu_display.cpp
  src/pie_chart_display.cpp
  src/plotter_2d_display.cpp
  src/linear_gauge_display.cpp
  src/overlay_diagnostic_display.cpp
  src/overlay_image_display.cpp
  src/image_transport_hints_property.cpp
  src/overlay_picker_tool.cpp
  # Group C: 3D scene displays
  src/facing_visualizer.cpp
  src/target_visualizer_display.cpp
  src/pictogram_display.cpp
  src/pictogram_array_display.cpp
  src/diagnostics_display.cpp
  src/tf_trajectory_display.cpp
  src/twist_stamped_display.cpp
  src/pose_array_display.cpp
  src/camera_info_display.cpp
  src/normal_display.cpp
  src/normal_visual.cpp
  # Group D: tools and panels
  src/close_all_tool.cpp
  src/open_all_tool.cpp
  src/screenshot_listener_tool.cpp
  src/publish_topic.cpp
  src/empty_service_call_interface.cpp
  src/record_action.cpp
  src/object_fit_operator.cpp
  src/robot_command_interface.cpp
  src/cancel_action.cpp
  src/select_point_cloud_publish_action.cpp
  # Group E: capture and rviz-internal displays
  src/rviz_scene_publisher.cpp
  src/video_capture_display.cpp
  src/quiet_interactive_marker_display.cpp
  # Phase 2: displays depending on formerly ROS 1-only message packages
  src/color_utils.cpp
  src/footstep_display.cpp
  src/ambient_sound_display_groovy.cpp
  src/ambient_sound_visual.cpp
  src/bounding_box_display.cpp
  src/bounding_box_array_display.cpp
  src/torus_array_display.cpp
  src/segment_array_display.cpp
  src/human_skeleton_array_display.cpp
  src/simple_occupancy_grid_array_display.cpp
  src/polygon_array_display.cpp
  src/yes_no_button_interface.cpp
  src/tablet_controller_panel.cpp
  src/view_controller/tablet_view_controller.cpp
  src/overlay_camera_display.cpp
)

if(people_msgs_FOUND)
  list(APPEND SOURCE_FILES src/people_position_measurement_array_display.cpp)
else()
  message(WARNING "people_msgs not found: skipping PeoplePositionMeasurementArray display")
endif()

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  jsk_visualization_utils::jsk_visualization_utils
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  image_transport::image_transport
  interactive_markers::interactive_markers
  cv_bridge::cv_bridge
  image_geometry::image_geometry
  message_filters::message_filters
  ${jsk_rviz_plugins_msgs_TARGETS}
  ${jsk_recognition_msgs_TARGETS}
  ${jsk_footstep_msgs_TARGETS}
  ${jsk_hark_msgs_TARGETS}
  ${jsk_gui_msgs_TARGETS}
  ${view_controller_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${action_msgs_TARGETS})
if(people_msgs_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE JSK_RVIZ_PLUGINS_HAVE_PEOPLE_MSGS)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${people_msgs_TARGETS})
endif()

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config icons launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

# Ported rclpy executables. Scripts still waiting on message packages that
# have no ROS 2 Jazzy release stay uninstalled (see git history / Phase 2).
install(PROGRAMS
  scripts/capture_rviz_text.py
  scripts/float32_to_overlay_text.py
  scripts/link_marker_publisher.py
  scripts/overlay_menu_bridge.py
  scripts/pointcloud_information.py
  scripts/pr2_visualization.py
  scripts/relay_camera_info.py
  scripts/relay_screenshot.py
  scripts/rosconsole_overlay_text.py
  scripts/static_overlay_text.py
  scripts/string_to_overlay_text.py
  scripts/tf_diff_text.py
  scripts/twist_stamped_add_header.py
  scripts/classification_result_visualizer.py
  scripts/rviz_mouse_point_to_tablet.py
  test/test_overlay_text_interface.py
  samples/bounding_box_sample.py
  samples/camera_sample.py
  samples/diagnostics_sample.py
  samples/footstep_sample.py
  samples/normal_sample.py
  samples/overlay_menu_sample.py
  samples/overlay_sample.py
  samples/pictogram.py
  samples/pictogram_all.py
  samples/piechart_sample.py
  samples/polygon_array_sample.py
  samples/segment_array_sample.py
  samples/sparse_occupancy_grid_sample.py
  samples/torus_array_sample.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_overlay_text_interface_launch.py
    TIMEOUT 60)
  add_launch_test(test/test_samples_launch.py
    TIMEOUT 180)
  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  if(TARGET test_plugin_load)
    target_link_libraries(test_plugin_load
      Qt5::Widgets
      rviz_common::rviz_common
      pluginlib::pluginlib)
    target_compile_definitions(test_plugin_load PRIVATE QT_NO_KEYWORDS)
  endif()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rviz_common rviz_rendering rviz_default_plugins rviz_ogre_vendor
  jsk_rviz_plugins_msgs jsk_visualization_utils jsk_recognition_msgs jsk_footstep_msgs jsk_hark_msgs
  jsk_gui_msgs view_controller_msgs
  std_msgs std_srvs geometry_msgs sensor_msgs
  diagnostic_msgs action_msgs cv_bridge image_geometry image_transport
  interactive_markers message_filters tf2_ros)
ament_package()
