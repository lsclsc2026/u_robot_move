#!/usr/bin/env bash
# Host-side package deployment. Never starts the receiver or robot control.
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
container="${TELEOP_CONTAINER:-unitree-dev}"
binary="${1:-/home/unitree/a2_joystick_lab/bin/a2_network_bridge}"
if [[ ! -f "$binary" ]]; then
  echo "Missing deployed receiver: $binary" >&2
  exit 2
fi
if [[ "$(docker inspect -f '{{.State.Running}}' "$container")" != true ]]; then
  echo "Container must already be running: $container" >&2
  exit 2
fi
if [[ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$container")" != host ]]; then
  echo 'This migration requires Docker host networking for UDP and native DDS' >&2
  exit 2
fi
# Refuse to replace executing binaries. This probe has no hardware side effects.
docker exec "$container" python3 -c '
import pathlib, sys
blocked=[]
for p in pathlib.Path("/proc").iterdir():
    if not p.name.isdigit() or int(p.name)==__import__("os").getpid(): continue
    try:
        args=(p/"cmdline").read_bytes().split(b"\0")
        names={pathlib.Path(a.decode(errors="replace")).name for a in args[:2]}
        if names & {"a2_network_bridge", "teleop_receiver.py", "a2_sport_backend", "a2_driver_node"}:
            blocked.append(p.name)
    except (OSError, ValueError): pass
if blocked:
    sys.exit("Stop active teleop/navigation before deployment; PIDs="+",".join(blocked))'
mkdir -p "$workspace/src/u_robot_teleop/vendor/bin"
cp -- "$binary" "$workspace/src/u_robot_teleop/vendor/bin/a2_network_bridge"
chmod +x "$workspace/src/u_robot_teleop/vendor/bin/a2_network_bridge"
sha256sum "$workspace/src/u_robot_teleop/vendor/bin/a2_network_bridge"
docker exec "$container" mkdir -p "$workspace/src/u_robot_teleop" "$workspace/scripts"
docker cp "$workspace/src/u_robot_teleop/." "$container:$workspace/src/u_robot_teleop/"
docker cp "$workspace/src/u_robot_a2_driver/include/u_robot_a2_driver/sport_control_lock.hpp" \
  "$container:$workspace/src/u_robot_a2_driver/include/u_robot_a2_driver/sport_control_lock.hpp"
docker cp "$workspace/src/u_robot_a2_driver/src/a2_sport_backend.cpp" \
  "$container:$workspace/src/u_robot_a2_driver/src/a2_sport_backend.cpp"
docker cp "$workspace/src/u_robot_bringup/launch/base.launch.py" \
  "$container:$workspace/src/u_robot_bringup/launch/base.launch.py"
docker cp "$workspace/scripts/pc1_ble_docker.py" "$container:$workspace/scripts/pc1_ble_docker.py"
docker exec -w "$workspace" "$container" bash -c \
  'source /opt/ros/humble/setup.bash && unset CYCLONEDDS_URI && colcon build --packages-select u_robot_teleop u_robot_a2_driver u_robot_bringup --symlink-install'
echo 'Installed u_robot_teleop and shared Sport control lock. No receiver was started.'
