#!/usr/bin/env bash
set -Eeuo pipefail

RED=$'\033[1;31m'
YELLOW=$'\033[1;33m'
RESET=$'\033[0m'

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="${R2_MAP_DIR:-${PROJECT_ROOT}/maps/official_field_map}"
PIDS=()
NAMES=()
CLEANED=0

cleanup() {
  local status=$?
  if (( CLEANED )); then
    return
  fi
  CLEANED=1
  trap - EXIT INT TERM
  printf '\n%b\n' "${YELLOW}[建图系统] 正在停止所有节点...${RESET}"
  for pid in "${PIDS[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "${PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
  exit "$status"
}
trap cleanup EXIT INT TERM

start_process() {
  local name=$1
  shift
  "$@" &
  PIDS+=("$!")
  NAMES+=("$name")
}

require_package() {
  ros2 pkg prefix "$1" >/dev/null 2>&1 || {
    printf '%b\n' "${RED}[错误] 未找到 ROS2 包: $1。请先在 ${PROJECT_ROOT} 完成 colcon build。${RESET}" >&2
    exit 1
  }
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf '%b\n' "${RED}[错误] /opt/ros/humble/setup.bash 不存在。${RESET}" >&2
  exit 1
fi
if [[ ! -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  printf '%b\n' "${RED}[错误] ${PROJECT_ROOT}/install/setup.bash 不存在，请先 colcon build。${RESET}" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/install/setup.bash"

require_package livox_ros_driver2
require_package fast_lio
require_package fast_lio_sam_sc_qn_ros2

mkdir -p "$(dirname -- "$MAP_DIR")"
if [[ -e "$MAP_DIR" ]]; then
  backup="${MAP_DIR}.backup_$(date +%Y%m%d_%H%M%S)"
  mv -- "$MAP_DIR" "$backup"
  printf '%b\n' "${YELLOW}[安全备份] 旧地图已移动到: ${backup}${RESET}"
fi

printf '\n%b\n' "${RED}============================================================${RESET}"
printf '%b\n' "${RED}  探场建图模式：请推车走完整个正式赛场，覆盖红蓝双方区域。${RESET}"
printf '%b\n' "${RED}  走完后在另一个已 source 环境的终端执行：${RESET}"
printf '%b\n' "${RED}  ros2 service call /r2/sam/save_map std_srvs/srv/Trigger \"{}\"${RESET}"
printf '%b\n' "${RED}  必须看到 success=True / 保存成功 Log 后，再回本终端按 Ctrl+C。${RESET}"
printf '%b\n' "${RED}  地图保存目录：${MAP_DIR}${RESET}"
printf '%b\n\n' "${RED}============================================================${RESET}"

start_process "Livox 驱动" \
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py
sleep 3
if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
  printf '%b\n' "${RED}[错误] Livox 驱动启动失败。${RESET}" >&2
  exit 1
fi

start_process "FAST-LIO + SAM 建图" \
  ros2 launch fast_lio_sam_sc_qn_ros2 system_run.launch.py \
    use_sim_time:=false \
    rviz:=false \
    run_simple_odom:=false \
    map_save_directory:="$MAP_DIR"

# 任一关键进程退出都终止整个建图系统，防止操作手在残缺链路下继续采图。
set +e
wait -n "${PIDS[@]}"
status=$?
set -e
printf '%b\n' "${RED}[错误] 建图关键进程提前退出（status=${status}），正在关闭其余节点。${RESET}" >&2
exit "$status"
