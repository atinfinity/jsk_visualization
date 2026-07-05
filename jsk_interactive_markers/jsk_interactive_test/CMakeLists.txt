cmake_minimum_required(VERSION 3.16)
project(jsk_interactive_test)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/publish_goal_marker.py
  scripts/trackpoint_controller.py
  DESTINATION lib/${PROJECT_NAME})

# mesh/texture resources referenced by publish_goal_marker.py
install(DIRECTORY scripts spacenav
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
