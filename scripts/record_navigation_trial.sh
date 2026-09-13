#!/usr/bin/env bash
set -eo pipefail

usage() {
  echo "Usage: $0 [label] [map_yaml]"
  echo "Start this after navigation.launch.py, but before initial pose and patrol."
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

workspace="/home/unitree/unitree_robot_development/u_robot_move"
sdk_setup="/home/unitree/unitree_robot_development/sdk/unitree_ros2/cyclonedds_ws/install/setup.bash"
trial_root="${NAV_TRIAL_ROOT:-${workspace}/navigation_trial_bags}"
label="${1:-patrol}"
default_map="/home/unitree/data/maps/room_01_clean.yaml"
if [[ ! -f "${default_map}" ]]; then
  default_map="/home/unitree/unitree-data/maps/room_01_clean.yaml"
fi
map_yaml="${2:-${default_map}}"

# Make labels safe as directory names while retaining readable ASCII names.
label="${label//[^[:alnum:]_.-]/_}"
if [[ -z "${label}" ]]; then
  label="patrol"
fi

source /opt/ros/humble/setup.bash
if [[ -f "${sdk_setup}" ]]; then
  source "${sdk_setup}"
fi
source "${workspace}/install/setup.bash"
unset CYCLONEDDS_URI
set -u

# Accept the path printed inside the navigation container as well as the host
# path used by Codex and the offline analysis scripts.
if [[ ! -f "${map_yaml}" && "${map_yaml}" == /home/unitree/data/* ]]; then
  map_yaml="/home/unitree/unitree-data/${map_yaml#/home/unitree/data/}"
fi
if [[ ! -f "${map_yaml}" && "${map_yaml}" == /home/unitree/unitree-data/* ]]; then
  map_yaml="/home/unitree/data/${map_yaml#/home/unitree/unitree-data/}"
fi

stamp="$(date +%Y%m%d_%H%M%S)"
trial_dir="${trial_root}/${label}_${stamp}"
bag_dir="${trial_dir}/bag"
params_dir="${trial_dir}/parameters"
snapshot_dir="${trial_dir}/snapshot"
mkdir -p "${params_dir}" "${snapshot_dir}"

echo "Preparing navigation trial: ${trial_dir}"

{
  echo "trial=${label}"
  echo "start_time=$(date --iso-8601=seconds)"
  echo "workspace=${workspace}"
  echo "map_yaml=${map_yaml}"
  echo "host=$(hostname)"
} > "${trial_dir}/manifest.txt"

nodes=(
  /amcl
  /localization_monitor
  /planner_server
  /controller_server
  /bt_navigator
  /behavior_server
  /velocity_smoother
  /velocity_deadband_adapter
  /collision_monitor
  /global_costmap/global_costmap
  /local_costmap/local_costmap
  /waypoint_patrol
  /a2_driver
)

node_list="$(timeout 3 ros2 node list 2>/dev/null || true)"
for node in "${nodes[@]}"; do
  if grep -Fxq "${node}" <<< "${node_list}"; then
    safe_name="${node#/}"
    safe_name="${safe_name//\//_}"
    (
      timeout 2 ros2 param dump "${node}" \
        > "${params_dir}/${safe_name}.yaml" 2>/dev/null || true
      if [[ ! -s "${params_dir}/${safe_name}.yaml" ]]; then
        rm -f "${params_dir}/${safe_name}.yaml"
      fi
    ) &
  fi
done
wait

cp "${workspace}/src/u_robot_navigation/config/nav2_params.yaml" \
  "${snapshot_dir}/nav2_params.yaml"
cp "${workspace}/src/u_robot_navigation/config/collision_monitor.yaml" \
  "${snapshot_dir}/collision_monitor.yaml"
cp "${workspace}/src/u_robot_navigation/config/velocity_deadband_adapter.yaml" \
  "${snapshot_dir}/velocity_deadband_adapter.yaml"
cp "${workspace}/src/u_robot_patrol/config/patrol.yaml" \
  "${snapshot_dir}/patrol.yaml"
cp "${workspace}/src/u_robot_localization/config/localization.yaml" \
  "${snapshot_dir}/localization.yaml"

if [[ -f "${map_yaml}" ]]; then
  cp "${map_yaml}" "${snapshot_dir}/"
  map_dir="$(dirname "${map_yaml}")"
  map_stem="$(basename "${map_yaml}" .yaml)"
  map_image="$(sed -n 's/^[[:space:]]*image[[:space:]]*:[[:space:]]*//p' "${map_yaml}" | head -1)"
  map_image="${map_image//\"/}"
  map_image="${map_image//\'/}"
  if [[ -n "${map_image}" && -f "${map_dir}/${map_image}" ]]; then
    cp "${map_dir}/${map_image}" "${snapshot_dir}/"
  fi
  if [[ -f "${map_dir}/${map_stem}_waypoints.yaml" ]]; then
    cp "${map_dir}/${map_stem}_waypoints.yaml" "${snapshot_dir}/"
  fi
fi

topics=(
  /navigation/operator_intervention
  /wirelesscontroller
  /waypoint_patrol/status
  /waypoint_patrol/active
  /navigation/waypoints
  /operator/initialpose
  /initialpose
  /operator/goal_pose
  /move_base_simple/goal
  /plan
  /local_plan
  /cmd_vel_nav
  /cmd_vel
  /cmd_vel_shaped
  /cmd_vel_safe
  /odometry/a2
  /amcl_pose
  /particle_cloud
  /localization/scan_alignment
  /localization/quality_ok
  /tf
  /tf_static
  /map
  /global_costmap/costmap
  /global_costmap/costmap_raw
  /global_costmap/published_footprint
  /local_costmap/costmap
  /local_costmap/costmap_raw
  /local_costmap/clearing_endpoints
  /local_costmap/published_footprint
  /visualization/robot_footprint
  /collision_monitor_state
  /collision_monitor/approach_footprint
  /navigation/obstacles
  /scan
  /camera/front/image/compressed
  /diagnostics
  /parameter_events
  /rosout
  /sportmodestate
  /lf/sportmodestate
  /api/sport/request
  /navigate_to_pose/_action/goal
  /navigate_to_pose/_action/result
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/status
  /follow_path/_action/goal
  /follow_path/_action/result
  /follow_path/_action/feedback
  /follow_path/_action/status
  /compute_path_to_pose/_action/goal
  /compute_path_to_pose/_action/result
  /compute_path_to_pose/_action/feedback
  /compute_path_to_pose/_action/status
)

echo
echo "Navigation trial recording is ready."
echo "Trial directory: ${trial_dir}"
echo "Now set initial pose, enable control, and start the patrol."
echo "Use Ctrl+C here only after the entire test (including intervention) ends."
echo

set +e
ros2 bag record \
  --output "${bag_dir}" \
  --max-bag-size 2147483648 \
  --max-cache-size 268435456 \
  --include-hidden-topics \
  --include-unpublished-topics \
  "${topics[@]}"
record_status=$?
set -e

{
  echo "end_time=$(date --iso-8601=seconds)"
  echo "record_status=${record_status}"
} >> "${trial_dir}/manifest.txt"

if [[ -f "${bag_dir}/metadata.yaml" ]]; then
  ros2 bag info "${bag_dir}" | tee "${trial_dir}/bag_info.txt"
fi

echo
echo "Recording finished: ${trial_dir}"
exit "${record_status}"
