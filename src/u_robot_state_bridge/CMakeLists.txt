cmake_minimum_required(VERSION 3.8)
project(u_robot_state_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Threads REQUIRED)

# SDK2 is a pinned vendor checkout next to this workspace, not an ament package
# in the A2 development image. Import its already-built library explicitly so
# this package does not depend on a machine-global, unversioned SDK install.
set(UNITREE_SDK2_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../sdk/unitree_sdk2"
  CACHE PATH "Path to the pinned Unitree SDK2 checkout")
if(NOT EXISTS "${UNITREE_SDK2_ROOT}/include/unitree/robot/channel/channel_subscriber.hpp")
  message(FATAL_ERROR "Pinned Unitree SDK2 headers not found: ${UNITREE_SDK2_ROOT}")
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(UNITREE_SDK2_ARCH aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(UNITREE_SDK2_ARCH x86_64)
else()
  message(FATAL_ERROR "Unsupported SDK2 target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(UNITREE_SDK2_LIBRARY
  "${UNITREE_SDK2_ROOT}/lib/${UNITREE_SDK2_ARCH}/libunitree_sdk2.a")
set(UNITREE_DDSC_LIBRARY
  "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}/libddsc.so")
set(UNITREE_DDSCXX_LIBRARY
  "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}/libddscxx.so")
foreach(vendor_library
    "${UNITREE_SDK2_LIBRARY}" "${UNITREE_DDSC_LIBRARY}" "${UNITREE_DDSCXX_LIBRARY}")
  if(NOT EXISTS "${vendor_library}")
    message(FATAL_ERROR "Pinned Unitree SDK2 library not found: ${vendor_library}")
  endif()
endforeach()
add_library(unitree_sdk2_vendor STATIC IMPORTED)
set_target_properties(unitree_sdk2_vendor PROPERTIES
  IMPORTED_LOCATION "${UNITREE_SDK2_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES
    "${UNITREE_SDK2_ROOT}/include;${UNITREE_SDK2_ROOT}/thirdparty/include;${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx"
  INTERFACE_LINK_LIBRARIES
    "${UNITREE_DDSC_LIBRARY};${UNITREE_DDSCXX_LIBRARY};Threads::Threads")

add_library(odometry_utils src/odometry_utils.cpp)
target_compile_features(odometry_utils PUBLIC cxx_std_17)
target_include_directories(odometry_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(state_bridge_node src/state_bridge_node.cpp)
target_compile_features(state_bridge_node PUBLIC cxx_std_17)
target_link_libraries(state_bridge_node odometry_utils)
ament_target_dependencies(state_bridge_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp tf2_ros)

add_executable(joint_state_bridge_node src/joint_state_bridge_node.cpp)
target_compile_features(joint_state_bridge_node PUBLIC cxx_std_17)
target_include_directories(joint_state_bridge_node PRIVATE include)
ament_target_dependencies(joint_state_bridge_node
  diagnostic_msgs geometry_msgs rclcpp sensor_msgs visualization_msgs)

add_executable(native_low_state_reader src/native_low_state_reader.cpp)
target_compile_features(native_low_state_reader PUBLIC cxx_std_17)
target_link_libraries(native_low_state_reader unitree_sdk2_vendor)
target_include_directories(native_low_state_reader PRIVATE include)

install(TARGETS odometry_utils state_bridge_node joint_state_bridge_node native_low_state_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry_utils test/test_odometry_utils.cpp)
  target_link_libraries(test_odometry_utils odometry_utils)
  target_include_directories(test_odometry_utils PRIVATE include)
  ament_add_gtest(test_native_joint_packet test/test_native_joint_packet.cpp)
  target_include_directories(test_native_joint_packet PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
