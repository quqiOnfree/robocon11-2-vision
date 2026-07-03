#!/usr/bin/env bash
set -Eeo pipefail

RED=$'\033[1;31m'
GREEN=$'\033[1;32m'
YELLOW=$'\033[1;33m'
RESET=$'\033[0m'
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="${PROJECT_ROOT}/maps"
SERIAL_PORT="${R2_SERIAL_PORT:-/dev/ttyACM0}"
PIDS=()
NAMES=()
CLEANED=0
MAIN_PID=$$

cleanup() {
  local status=$?
  (( CLEANED )) && return
  CLEANED=1
  trap - EXIT INT TERM
  printf '\n%b\n' "${YELLOW}[比赛系统] 正在停止所有节点...${RESET}"
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
  NAMES+=("$name")
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

monitor_processes() {
  while sleep 1; do
    for i in "${!PIDS[@]}"; do
      if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
        printf '%b\n' "${RED}[致命] ${NAMES[$i]} 已退出，关闭整套系统。${RESET}" >&2
        kill -TERM "$MAIN_PID" 2>/dev/null || true
        return
      fi
    done
  done
}

[[ -f /opt/ros/humble/setup.bash && -f "${PROJECT_ROOT}/install/setup.bash" ]] || {
  echo "ROS 环境或工程 install 不存在" >&2; exit 1;
}
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/install/setup.bash"
set -u
for package in livox_ros_driver2 fast_lio r2_serial path_planning_node hand_input_path; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || { echo "未找到 ROS2 包: $package" >&2; exit 1; }
done

printf '半区选择：\n  [1] 蓝方 Blue\n  [2] 红方 Red\n'
read -r -p "请选择: " zone_choice
case "$zone_choice" in
  1) ZONE=blue ;;
  2) ZONE=red ;;
  *) echo "选择无效" >&2; exit 1 ;;
esac

printf '赛制选择：\n  [1] 挑战赛 Challenge\n  [2] 正赛 Normal\n'
read -r -p "请选择: " game_choice
case "$game_choice" in
  1) GAME=challenge ;;
  2) GAME=normal ;;
  *) echo "选择无效" >&2; exit 1 ;;
esac

printf '定位模式：\n  [1] 正常重定位 Localization\n  [2] 纯里程计强制纠正 Fallback\n'
read -r -p "请选择: " localization_choice
case "$localization_choice" in
  1) MODE=localization; ODOM_TOPIC=/r2/global_odometry; MAP_DIR="$(select_map)" ;;
  2) MODE=fallback; ODOM_TOPIC=/Odometry; MAP_DIR="" ;;
  *) echo "选择无效" >&2; exit 1 ;;
esac

if [[ "$MODE" == localization ]]; then
  ros2 pkg prefix fast_lio_localization_sc_qn_ros2 >/dev/null 2>&1 || {
    echo "未找到重定位包 fast_lio_localization_sc_qn_ros2" >&2; exit 1;
  }
fi
if [[ ! -e "$SERIAL_PORT" ]]; then
  printf '%b\n' "${YELLOW}[警告] 串口 ${SERIAL_PORT} 当前不存在，r2_serial 将自动重连。${RESET}"
fi

if [[ "" == blue && "" == normal ]]; then
  # START 是 simple_odom 修正后的底盘/MCU 坐标；PRIOR 是 poses.csv 的 map->body 坐标。
  START_X=-310.0; START_Y=-115.0
  PRIOR_X=0.0; PRIOR_Y=0.0

elif [[ "" == blue && "" == challenge ]]; then
  START_X=10000.0; START_Y=6500.0
  PRIOR_X=10000.0; PRIOR_Y=6500.0  # 实测 /r2/global_odometry 后替换

elif [[ "" == red && "" == normal ]]; then
  START_X=-310.0; START_Y=-2950.0
  PRIOR_X=-310.0; PRIOR_Y=-2950.0  # 实测 /r2/global_odometry 后替换
else
  START_X=10000.0; START_Y=-9565.0
  PRIOR_X=10000.0; PRIOR_Y=-9565.0  # 实测 /r2/global_odometry 后替换
fi

printf '\n配置确认：zone=%s game=%s mode=%s start=(%s,%s) mm\n' \
  "$ZONE" "$GAME" "$MODE" "$START_X" "$START_Y"
read -r -p "输入 YES 确认启动: " confirmation
[[ "$confirmation" == YES ]] || { echo "已取消"; exit 0; }

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
      expected_x_mm:="$PRIOR_X" expected_y_mm:="$PRIOR_Y"
fi

# 路径节点先启动并等待 DDS 发现，避免串口接通瞬间丢失首条 0x0301/0x031D。
start_process "路径规划" ros2 run path_planning_node path_planning_node
sleep 1
start_process "r2_serial 串口收发" \
  ros2 launch r2_serial r2_data_downlink.launch.py \
    serial_port:="$SERIAL_PORT" serial_debug_raw:=false \
    match_zone_topic:=/r2/match_zone
start_process "手输方块 GUI" ros2 run hand_input_path hand_input_path_node

sleep 2
for i in "${!PIDS[@]}"; do
  kill -0 "${PIDS[$i]}" 2>/dev/null || {
    printf '%b\n' "${RED}[错误] ${NAMES[$i]} 启动失败。${RESET}" >&2
    exit 1
  }
done
monitor_processes &
PIDS+=("$!")
NAMES+=("进程监视器")

printf '\n%b\n' "${GREEN}============================================================${RESET}"
printf '%b\n' "${GREEN}  比赛系统已全线拉起！${RESET}"
printf '%b\n' "${GREEN}  请在此终端内按 r1（回启动区）或 r2（回重试区）进行异常恢复。${RESET}"
printf '%b\n' "${GREEN}  q：显示状态并执行一次 5 秒静止平均。${RESET}"
printf '%b\n' "${GREEN}============================================================${RESET}"

# 位姿节点必须占据前台，确保 q/r1/r2 不会被任何后台节点抢走。
set +e
ros2 run fast_lio simple_odom --ros-args \
  -p mode:="$MODE" -p game:="$GAME" -p zone:="$ZONE" \
  -p odom_topic:="$ODOM_TOPIC" -p localized_topic:=/r2/localized \
  -p match_zone_topic:=/r2/match_zone \
  -p "initial_target_point:=[$START_X, $START_Y]"
status=$?
set -e
exit "$status"
