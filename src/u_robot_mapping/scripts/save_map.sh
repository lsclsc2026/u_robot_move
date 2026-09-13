#!/usr/bin/env bash
set -eo pipefail

usage() {
  echo "用法: ros2 run u_robot_mapping save_map.sh <地图名称> [输出目录]" >&2
  echo "示例: ros2 run u_robot_mapping save_map.sh room_01" >&2
}

map_name="${1:-}"
output_dir="${2:-/home/unitree/data/maps}"

if [[ -z "${map_name}" ]]; then
  usage
  exit 2
fi
if [[ ! "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo "ERROR: 地图名称只能包含字母、数字、点、下划线和连字符" >&2
  exit 2
fi
if [[ ! -d "${output_dir}" ]]; then
  echo "ERROR: 输出目录不存在: ${output_dir}" >&2
  exit 3
fi
if [[ ! -w "${output_dir}" ]]; then
  echo "ERROR: 输出目录不可写: ${output_dir}" >&2
  exit 3
fi

map_prefix="${output_dir}/${map_name}"

source /opt/ros/humble/setup.bash

if [[ -f /home/unitree/unitree_robot_development/u_robot_move/install/setup.bash ]]; then
  source /home/unitree/unitree_robot_development/u_robot_move/install/setup.bash
fi
set -u

map_info="$(timeout 8s ros2 topic info /map -v 2>/dev/null || true)"
map_type="$(sed -n 's/^Type: //p' <<< "${map_info}" | sed -n '1p')"
map_publisher_count="$(sed -n 's/^Publisher count: //p' <<< "${map_info}" | sed -n '1p')"

if [[ "${map_type}" != "nav_msgs/msg/OccupancyGrid" ]]; then
  echo "ERROR: 未发现有效的 /map，请先启动 manual_mapping.launch.py" >&2
  exit 5
fi
if [[ "${map_publisher_count}" != "1" ]]; then
  echo "ERROR: /map 当前有 ${map_publisher_count:-0} 个发布者，拒绝保存以免写入错误地图" >&2
  echo "请停止导航/localization 等旧会话，只保留 manual_mapping.launch.py" >&2
  exit 5
fi
if ! grep -Fq "Node name: slam_toolbox" <<< "${map_info}"; then
  echo "ERROR: /map 的唯一发布者不是 slam_toolbox，拒绝保存" >&2
  exit 5
fi
if ! timeout 8s ros2 topic echo /map --once --field info >/dev/null 2>&1; then
  echo "ERROR: /map 存在但未能在 8 秒内收到地图数据" >&2
  exit 5
fi

serialize_service_type="$(
  timeout 8s ros2 service type /slam_toolbox/serialize_map 2>/dev/null || true
)"
if [[ "${serialize_service_type}" != "slam_toolbox/srv/SerializePoseGraph" ]]; then
  echo "ERROR: SLAM Toolbox 保存服务不可用" >&2
  exit 5
fi

# The map name is an operator-owned slot. Once the live mapping source and save
# service have been validated, replace every artifact belonging to that slot
# without confirmation or backup. Explicit paths avoid touching similarly named
# maps such as room1_clean or room10.
overwritten_files=()
for suffix in yaml pgm png posegraph data; do
  candidate="${map_prefix}.${suffix}"
  if [[ -e "${candidate}" ]]; then
    overwritten_files+=("${candidate}")
  fi
done
if (( ${#overwritten_files[@]} > 0 )); then
  echo "直接覆盖同名地图: ${map_prefix}"
  rm -f -- \
    "${map_prefix}.yaml" \
    "${map_prefix}.pgm" \
    "${map_prefix}.png" \
    "${map_prefix}.posegraph" \
    "${map_prefix}.data"
fi

echo "保存占据栅格地图: ${map_prefix}.yaml / ${map_prefix}.pgm"
ros2 run nav2_map_server map_saver_cli -f "${map_prefix}"

echo "保存可继续编辑的 SLAM 位姿图: ${map_prefix}.posegraph / ${map_prefix}.data"
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '${map_prefix}'}"

echo "地图保存请求已完成: ${map_prefix}"
