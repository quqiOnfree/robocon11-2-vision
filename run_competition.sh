#!/usr/bin/env bash
set -Eeuo pipefail

RED=$'\033[1;31m'
GREEN=$'\033[1;32m'
YELLOW=$'\033[1;33m'
RESET=$'\033[0m'

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="${R2_MAP_DIR:-${PROJECT_ROOT}/maps/official_field_map}"
SERIAL_PORT="${R2_SERIAL_PORT:-/dev/ttyACM0}"
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
  printf '\n%b\n' "${YELLOW}[比赛系统] 正在停止所有节点...${RESET}"
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
  printf '[启动] %-24s pid=%s\n' "$name" "$!"
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

for package in \
  livox_ros_driver2 fast_lio fast_lio_localization_sc_qn_ros2 \
  r2_serial path_planning_node hand_input_path; do
  require_package "$package"
done

if [[ ! -f "${MAP_DIR}/poses.csv" || ! -d "${MAP_DIR}/keyframes" ]]; then
  printf '%b\n' "${RED}[错误] 正式地图不完整: ${MAP_DIR}${RESET}" >&2
  printf '%b\n' "${RED}至少需要 poses.csv 和 keyframes/。禁止在无地图状态进入比赛。${RESET}" >&2
  exit 1
fi
if [[ ! -e "$SERIAL_PORT" ]]; then
  printf '%b\n' "${YELLOW}[警告] 串口 ${SERIAL_PORT} 当前不存在；r2_serial 会持续自动重连。${RESET}"
fi

# 1. Livox Mid-360S 驱动
start_process "Livox Mid-360S 驱动" \
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py
sleep 3

# 2. FAST-LIO 纯前端
start_process "FAST-LIO 前端" \
  ros2 launch fast_lio mapping.launch.py use_sim_time:=false rviz:=false

# 3. 基于预制关键帧地图的全局重定位
start_process "SC-QN 全局重定位" \
  ros2 launch fast_lio_localization_sc_qn_ros2 localization_sc_qn.launch.py \
    use_sim_time:=false \
    map_directory:="$MAP_DIR"

# 4. 唯一串口收发节点
start_process "r2_serial 串口收发" \
  ros2 launch r2_serial r2_data_downlink.launch.py \
    serial_port:="$SERIAL_PORT" \
    serial_debug_raw:=false

# 5. 位姿上报与红蓝半区映射。半区机制在节点内默认启用，且只在 /r2/localized=true 后锁区。
start_process "R2 位姿上报/半区映射" \
  ros2 run fast_lio simple_odom --ros-args \
    -p odom_topic:=/r2/global_odometry \
    -p localized_topic:=/r2/localized

# 6. 路径规划与手输方块 GUI；本方案不启动视觉节点。
start_process "路径规划" \
  ros2 run path_planning_node path_planning_node
start_process "手输方块 GUI" \
  ros2 run hand_input_path hand_input_path_node

sleep 3
for i in "${!PIDS[@]}"; do
  if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
    printf '%b\n' "${RED}[错误] ${NAMES[$i]} 启动后提前退出。${RESET}" >&2
    exit 1
  fi
done

printf '\n%b\n' "${GREEN}============================================================${RESET}"
printf '%b\n' "${GREEN}  比赛系统已全线拉起！${RESET}"
printf '%b\n' "${GREEN}  请确认定位日志出现 Cold-start match accepted。${RESET}"
printf '%b\n' "${GREEN}  请观察是否输出 [Zone Detected] 和 /r2/localized: true。${RESET}"
printf '%b\n' "${GREEN}  状态检查：ros2 topic echo /r2/localized --once --qos-durability transient_local${RESET}"
printf '%b\n' "${GREEN}  按 Ctrl+C 将安全停止全部节点。${RESET}"
printf '%b\n\n' "${GREEN}============================================================${RESET}"

# 任一节点异常退出，立即关闭整套系统，避免剩余节点继续控制底盘。
set +e
wait -n "${PIDS[@]}"
status=$?
set -e
printf '%b\n' "${RED}[错误] 某个比赛节点已退出（status=${status}），正在关闭整套系统。${RESET}" >&2
exit "$status"
