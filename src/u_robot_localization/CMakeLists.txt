cmake_minimum_required(VERSION 3.8)
project(u_robot_localization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(localization_health src/localization_health.cpp)
target_compile_features(localization_health PUBLIC cxx_std_17)
target_include_directories(localization_health PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(localization_monitor_node src/localization_monitor_node.cpp)
target_compile_features(localization_monitor_node PUBLIC cxx_std_17)
target_link_libraries(localization_monitor_node localization_health)
ament_target_dependencies(localization_monitor_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

add_executable(initial_pose_adapter_node src/initial_pose_adapter_node.cpp)
target_compile_features(initial_pose_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(initial_pose_adapter_node
  geometry_msgs
  rclcpp
  std_srvs
)

install(TARGETS localization_health localization_monitor_node initial_pose_adapter_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_localization_health test/test_localization_health.cpp)
  target_link_libraries(test_localization_health localization_health)
endif()

ament_export_include_directories(include)
ament_export_libraries(localization_health)
ament_package()
