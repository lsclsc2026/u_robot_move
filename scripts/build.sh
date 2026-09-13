#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
development_dir="$(cd -- "${workspace_dir}/.." && pwd)"
vendor_ros2_dir="${development_dir}/sdk/unitree_ros2/cyclonedds_ws/src/unitree"

source /opt/ros/humble/setup.bash
set -u

cd "${workspace_dir}"
colcon build \
  --symlink-install \
  --base-paths \
    src \
    "${vendor_ros2_dir}/unitree_api" \
    "${vendor_ros2_dir}/unitree_go" \
    "${vendor_ros2_dir}/unitree_hg" \
  --event-handlers console_cohesion+
