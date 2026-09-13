cmake_minimum_required(VERSION 3.8)
project(u_robot_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/save_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
