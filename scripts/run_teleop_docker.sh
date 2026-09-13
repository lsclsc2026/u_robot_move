#!/usr/bin/env bash
# Host/SSH entry: forward arguments to the installed Docker receiver supervisor.
set -eo pipefail
container_workspace="${TELEOP_CONTAINER_WORKSPACE:-/home/unitree/unitree_robot_development/u_robot_move}"
container="${TELEOP_CONTAINER:-unitree-review}"
if [[ "${1:-}" == --container ]]; then
  container="${2:?missing container name}"
  shift 2
fi
if [[ ! "$container" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo 'Invalid container name' >&2
  exit 2
fi
# Old host-only receivers do not participate in the new shared lock.
# Refuse their simultaneous use instead of killing another user's process.
python3 -c '
import os, pathlib, sys
host_namespace=os.stat("/proc/self/ns/mnt").st_ino
for p in pathlib.Path("/proc").iterdir():
    if not p.name.isdigit(): continue
    try:
        exe=(p/"exe").resolve(strict=True)
        namespace=os.stat(p/"ns/mnt").st_ino
        if exe.name=="a2_network_bridge" and namespace==host_namespace:
            sys.exit("Host receiver is already running (pid="+p.name+"). Stop the old receiver first.")
    except (OSError, ValueError): pass'
exec docker exec -i "$container" \
  "$container_workspace/install/u_robot_teleop/lib/u_robot_teleop/teleop_receiver.py" "$@"
