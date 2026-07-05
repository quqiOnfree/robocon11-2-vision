#!/usr/bin/env bash
set -Eeo pipefail

RED=$'\033[1;31m'
YELLOW=$'\033[1;33m'
RESET=$'\033[0m'
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PIDS=()
CLEANED=0

cleanup() {
  local status=$?
  (( CLEANED )) && return
  CLEANED=1
  trap - EXIT INT TERM
  printf '\n%b\n' "${YELLOW}[建图系统] 正在停止所有节点...${RESET}"
  for pid in "${PIDS[@]:-}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]:-}"; do wait "$pid" 2>/dev/null || true; done
  exit "$status"
}
trap cleanup EXIT INT TERM

start_process() {
  local name=$1
  shift
  "$@" </dev/null &
  PIDS+=("$!")
  printf '[启动] %-24s pid=%s\n' "$name" "$!"
}

printf '请选择建图半场（标准地图名为 official_map_blue / official_map_red）：\n  [1] 蓝方 Blue\n  [2] 红方 Red\n'
read -r -p "请选择: " map_zone_choice
case "$map_zone_choice" in
  1) MAP_ZONE=blue ;;
  2) MAP_ZONE=red ;;
  *) printf '%b\n' "${RED}[错误] 半场选择无效。${RESET}" >&2; exit 1 ;;
esac
MAP_DIR="${PROJECT_ROOT}/maps/official_map_${MAP_ZONE}"

[[ -f /opt/ros/humble/setup.bash ]] || { echo "缺少 ROS Humble" >&2; exit 1; }
[[ -f "${PROJECT_ROOT}/install/setup.bash" ]] || { echo "请先 colcon build" >&2; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/install/setup.bash"
set -u
for package in livox_ros_driver2 fast_lio fast_lio_sam_sc_qn_ros2; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || {
    printf '%b\n' "${RED}[错误] 未找到 ROS2 包: ${package}${RESET}" >&2
    exit 1
  }
done

mkdir -p "${PROJECT_ROOT}/maps"
if [[ -e "$MAP_DIR" ]]; then
  backup="${MAP_DIR}.backup_$(date +%Y%m%d_%H%M%S)"
  mv -- "$MAP_DIR" "$backup"
  printf '%b\n' "${YELLOW}[安全备份] 旧地图已移动到: ${backup}${RESET}"
fi

printf '\n%b\n' "${RED}============================================================${RESET}"
printf '%b\n' "${RED}  ${MAP_ZONE^^} 半场独立建图：只覆盖当前半场和稳定的周边结构。${RESET}"
printf '%b\n' "${RED}  完成后在另一个已 source 的终端执行：${RESET}"
printf '%b\n' "${RED}  ros2 service call /r2/sam/save_map std_srvs/srv/Trigger \"{}\"${RESET}"
printf '%b\n' "${RED}  看到 success=True 和保存成功日志后，才可在本终端 Ctrl+C。${RESET}"
printf '%b\n' "${RED}  保存目录：${MAP_DIR}${RESET}"
printf '%b\n\n' "${RED}============================================================${RESET}"

start_process "Livox Mid-360S 驱动" \
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py
sleep 3
kill -0 "${PIDS[0]}" 2>/dev/null || { echo "Livox 驱动启动失败" >&2; exit 1; }

start_process "FAST-LIO + SAM 建图" \
  ros2 launch fast_lio_sam_sc_qn_ros2 system_run.launch.py \
    use_sim_time:=false rviz:=false run_simple_odom:=false \
    map_save_directory:="$MAP_DIR"

set +e
wait -n "${PIDS[@]}"
status=$?
set -e
printf '%b\n' "${RED}[错误] 建图关键进程退出（status=${status}）。${RESET}" >&2
exit "$status"
