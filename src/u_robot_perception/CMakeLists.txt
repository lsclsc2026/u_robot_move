cmake_minimum_required(VERSION 3.8)
project(u_robot_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(navigation_cloud_filter_node src/navigation_cloud_filter_node.cpp)
target_compile_features(navigation_cloud_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(navigation_cloud_filter_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(cloud_timestamp_normalizer_node src/cloud_timestamp_normalizer_node.cpp)
target_compile_features(cloud_timestamp_normalizer_node PUBLIC cxx_std_17)
ament_target_dependencies(cloud_timestamp_normalizer_node rclcpp sensor_msgs)

install(TARGETS navigation_cloud_filter_node cloud_timestamp_normalizer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
