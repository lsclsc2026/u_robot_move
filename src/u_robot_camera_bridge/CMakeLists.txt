cmake_minimum_required(VERSION 3.8)
project(u_robot_camera_bridge)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/front_camera_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME front_camera_bridge_node)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
