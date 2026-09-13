cmake_minimum_required(VERSION 3.8)
project(u_robot_navigation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(operator_goal_adapter_node src/operator_goal_adapter_node.cpp)
target_compile_features(operator_goal_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(operator_goal_adapter_node
  diagnostic_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

add_executable(velocity_deadband_adapter_node src/velocity_deadband_adapter_node.cpp)
target_compile_features(velocity_deadband_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(velocity_deadband_adapter_node geometry_msgs rclcpp)

install(TARGETS operator_goal_adapter_node velocity_deadband_adapter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
