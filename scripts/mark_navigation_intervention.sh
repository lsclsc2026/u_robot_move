#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 BEGIN|END|NOTE [description]" >&2
  exit 2
fi

workspace="/home/unitree/unitree_robot_development/u_robot_move"
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
unset CYCLONEDDS_URI
set -u

kind="${1^^}"
shift
case "${kind}" in
  BEGIN|END|NOTE) ;;
  *)
    echo "First argument must be BEGIN, END, or NOTE" >&2
    exit 2
    ;;
esac

description="$*"
message="${kind}|$(date --iso-8601=seconds)|${description}"
message="${message//\\/\\\\}"
message="${message//\"/\\\"}"

ros2 topic pub --once --wait-matching-subscriptions 1 \
  /navigation/operator_intervention \
  std_msgs/msg/String \
  "{data: \"${message}\"}"
