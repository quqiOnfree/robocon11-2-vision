#!/usr/bin/env bash
set -Eeo pipefail

RED=$'\033[1;31m'
GREEN=$'\033[1;32m'
YELLOW=$'\033[1;33m'
RESET=$'\033[0m'
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="${PROJECT_ROOT}/maps"
PIDS=()
CLEANED=0

cleanup() {
  local status=$?
  (( CLEANED )) && return
  CLEANED=1
  trap - EXIT INT TERM
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

select_map() {
  local maps=()
  while IFS= read -r dir; do
    [[ -f "$dir/poses.csv" && -d "$dir/keyframes" ]] && maps+=("$dir")
  done < <(find "$MAPS_DIR" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort)
  ((${#maps[@]} > 0)) || {
    printf '%b\n' "${RED}[错误] maps/ 下没有有效地图。${RESET}" >&2
    return 1
  }
  printf '可用地图：\n' >&2
  for i in "${!maps[@]}"; do printf '  [%d] %s\n' "$((i + 1))" "${maps[$i]}" >&2; done
  local choice
  read -r -p "选择地图编号: " choice
  [[ "$choice" =~ ^[0-9]+$ ]] || return 1
  (( choice >= 1 && choice <= ${#maps[@]} )) || return 1
  printf '%s' "${maps[$((choice - 1))]}"
}

[[ -f /opt/ros/humble/setup.bash && -f "${PROJECT_ROOT}/install/setup.bash" ]] || {
  echo "ROS 环境或工程 install 不存在" >&2; exit 1;
}
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/install/setup.bash"
set -u

printf '寻点调试模式：\n  [1] 纯里程计 + fallback 坐标纠正\n  [2] 预制地图重定位\n'
read -r -p "请选择: " mode_choice
case "$mode_choice" in
  1) MODE=fallback; ODOM_TOPIC=/Odometry; ZONE=blue; GAME=normal; MAP_DIR="" ;;
  2) MODE=localization; ODOM_TOPIC=/r2/global_odometry; GAME=normal ;;
  *) echo "选择无效" >&2; exit 1 ;;
esac
if [[ "$MODE" == localization ]]; then
  printf '调试半区：\n  [1] 蓝方 Blue\n  [2] 红方 Red\n'
  read -r -p "请选择: " debug_zone_choice
  case "$debug_zone_choice" in
    1) ZONE=blue ;;
    2) ZONE=red ;;
    *) echo "选择无效" >&2; exit 1 ;;
  esac
  MAP_DIR="$(select_map)"
fi

start_process "Livox Mid-360S 驱动" \
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py
sleep 3
start_process "FAST-LIO 前端" \
  ros2 launch fast_lio mapping.launch.py use_sim_time:=false rviz:=false
if [[ "$MODE" == localization ]]; then
  start_process "SC-QN 全局重定位" \
    ros2 launch fast_lio_localization_sc_qn_ros2 localization_sc_qn.launch.py \
      use_sim_time:=false map_directory:="$MAP_DIR" \
      use_position_prior:=true \
      expected_x_mm:=0.0 expected_y_mm:=0.0
fi
sleep 2

printf '\n%b\n' "${GREEN}调试链路已启动。此终端由 r2_pose_reporter 独占。${RESET}"
printf '%b\n' "${GREEN}输入 q：立即显示状态，并启动一次 5 秒静止平均供抄录。${RESET}"
printf '%b\n' "${YELLOW}本脚本不启动 r2_serial，不会向 MCU 下发位置。${RESET}"

set +e
ros2 run fast_lio simple_odom --ros-args \
  -p mode:="$MODE" -p game:="$GAME" -p zone:="$ZONE" \
  -p odom_topic:="$ODOM_TOPIC" \
  -p localized_topic:=/r2/localized
status=$?
set -e
exit "$status"
