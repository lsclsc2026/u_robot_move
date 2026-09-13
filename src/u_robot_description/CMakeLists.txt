cmake_minimum_required(VERSION 3.8)
project(u_robot_description)

find_package(ament_cmake REQUIRED)

get_filename_component(DEVELOPMENT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(VENDOR_DESCRIPTION_ROOT
  "${DEVELOPMENT_ROOT}/sdk/unitree_ros/robots/a2_description"
  CACHE PATH "Path to the official Unitree A2 description")
set(VENDOR_URDF "${VENDOR_DESCRIPTION_ROOT}/urdf/a2.urdf")

if(NOT EXISTS "${VENDOR_URDF}")
  message(FATAL_ERROR "Official A2 URDF not found: ${VENDOR_URDF}")
endif()

# Keep the official source unchanged. Only rewrite mesh URIs in the generated copy
# so RViz and downstream tools can resolve them through the ROS package index.
file(READ "${VENDOR_URDF}" A2_URDF_CONTENT)
string(REPLACE "../meshes/" "package://u_robot_description/meshes/"
  A2_URDF_CONTENT "${A2_URDF_CONTENT}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/a2.urdf" "${A2_URDF_CONTENT}")

# The isolated preview uses a fixed world root so Foxglove can perform the whole
# kinematic assembly from /joint_states without relying on an external root TF.
set(A2_PREVIEW_URDF_CONTENT "${A2_URDF_CONTENT}")
string(REPLACE
  "<robot name=\"a2\">"
  "<robot name=\"a2_preview\">\n  <link name=\"world\"/>\n  <joint name=\"world_to_base_link\" type=\"fixed\">\n    <parent link=\"world\"/>\n    <child link=\"base_link\"/>\n    <origin xyz=\"0 0 0.527\" rpy=\"0 0 0\"/>\n  </joint>"
  A2_PREVIEW_URDF_CONTENT
  "${A2_PREVIEW_URDF_CONTENT}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/a2_preview.urdf" "${A2_PREVIEW_URDF_CONTENT}")

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/a2.urdf"
  "${CMAKE_CURRENT_BINARY_DIR}/a2_preview.urdf"
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY "${VENDOR_DESCRIPTION_ROOT}/meshes" DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/a2_stand_joint_publisher.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES "${DEVELOPMENT_ROOT}/sdk/unitree_ros/LICENSE"
  DESTINATION share/${PROJECT_NAME})

ament_package()
