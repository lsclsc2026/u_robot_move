#!/usr/bin/env bash
set -eo pipefail

network_interface="${UNITREE_NETWORK_INTERFACE:-eth0}"

source /opt/ros/humble/setup.bash
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network_interface}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"

if ! ip link show "${network_interface}" >/dev/null 2>&1; then
  echo "ERROR: network interface '${network_interface}' does not exist" >&2
  exit 1
fi

echo "ROS_DISTRO=${ROS_DISTRO}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "UNITREE_NETWORK_INTERFACE=${network_interface}"
ip -br address show "${network_interface}"

echo
echo "Required A2 topics discovered:"
required_topics=(
  /api/sport/request
  /api/sport/response
  /dog_odom
  /dog_imu_raw
  /lf/sportmodestate
  /unitree/slam_lidar/points
)

topic_list="$(timeout 8s ros2 topic list)"
missing=0
for topic in "${required_topics[@]}"; do
  if grep -Fxq "${topic}" <<<"${topic_list}"; then
    echo "  OK      ${topic}"
  else
    echo "  MISSING ${topic}"
    missing=1
  fi
done

if (( missing != 0 )); then
  echo "ERROR: one or more required topics are unavailable" >&2
  exit 2
fi

echo "Environment check passed. No command was published."
