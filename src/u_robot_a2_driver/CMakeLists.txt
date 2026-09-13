cmake_minimum_required(VERSION 3.8)
project(u_robot_a2_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_api REQUIRED)
# The host has an installed SDK, while the development container builds from
# the checked-out SDK source. Avoid the host's incomplete /usr/local export.
set(UNITREE_SDK2_SOURCE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../sdk/unitree_sdk2)
if(EXISTS /opt/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake)
  find_package(unitree_sdk2 REQUIRED
    PATHS /opt/unitree_robotics/lib/cmake/unitree_sdk2
    NO_DEFAULT_PATH)
else()
  set(BUILD_EXAMPLES OFF CACHE BOOL "Build Unitree SDK examples" FORCE)
  add_subdirectory(${UNITREE_SDK2_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2 EXCLUDE_FROM_ALL)
endif()

add_library(a2_command_guard src/command_guard.cpp)
target_compile_features(a2_command_guard PUBLIC cxx_std_17)
target_include_directories(a2_command_guard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(a2_driver_node src/a2_driver_node.cpp)
target_compile_features(a2_driver_node PUBLIC cxx_std_17)
target_link_libraries(a2_driver_node a2_command_guard)
ament_target_dependencies(a2_driver_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  std_srvs
  unitree_api)

# Standalone native-SDK commissioning probe. It deliberately does not use ROS:
# run it separately after disabling the ROS driver control gate.
add_executable(a2_sport_probe src/a2_sport_probe.cpp)
target_compile_features(a2_sport_probe PUBLIC cxx_std_17)
target_link_libraries(a2_sport_probe unitree_sdk2)
target_include_directories(a2_sport_probe PRIVATE
  ${UNITREE_SDK2_SOURCE_DIR}/include)

add_executable(a2_sport_backend src/a2_sport_backend.cpp)
target_compile_features(a2_sport_backend PUBLIC cxx_std_17)
target_link_libraries(a2_sport_backend unitree_sdk2)
target_include_directories(a2_sport_backend PRIVATE
  include ${UNITREE_SDK2_SOURCE_DIR}/include)

install(TARGETS a2_command_guard a2_driver_node a2_sport_probe a2_sport_backend
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_guard test/test_command_guard.cpp)
  target_link_libraries(test_command_guard a2_command_guard)
  target_include_directories(test_command_guard PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
