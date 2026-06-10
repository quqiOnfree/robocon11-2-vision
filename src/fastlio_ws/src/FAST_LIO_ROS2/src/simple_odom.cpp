#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "r2_serial/msg/serial_packet.hpp"
#include "r2_serial/serial_protocol.hpp"

namespace simple_odom_protocol = r2_serial::protocol;

class SimpleOdomSender : public rclcpp::Node {
public:
  enum MissionState {
    IDLE,
    MOVING,
    WAITING_ATOMIC_UP,
    WAITING_ATOMIC_DOWN,
    STAGED_UP_WAIT_APPROACH_ARRIVAL,
    STAGED_UP_WAIT_HIGH_MODE,
    STAGED_UP_WAIT_LOWERING_COMPLETED,
    STAGED_UP_WAIT_EXIT_ARRIVAL,
    SEQ_WAIT_DOWN_COMPLETED,
    SEQ_WAIT_HOME_ARRIVAL
  };

  enum GripperState {
    GRIPPER_IDLE = 0,      // 视觉不干预，夹爪锁定。
    GRIPPER_TRACKING = 1,  // 视觉伺服微调。
    GRIPPER_GRAB = 2       // 闭合收取。
  };

  struct TargetPose {
    std::int16_t x_mm;
    std::int16_t y_mm;
    std::int16_t yaw_deg;
  };

  SimpleOdomSender() : Node("simple_odom"), current_state_(IDLE) {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    const auto deprecated_serial_port = declare_parameter<std::string>("serial_port", "");
    (void)declare_parameter<bool>("serial_debug_raw", false);
    if (!deprecated_serial_port.empty()) {
      RCLCPP_WARN(get_logger(),
                  "simple_odom 已不再直接打开串口，serial_port=%s 将被忽略；请启动 r2_data_downlink 管理串口。",
                  deprecated_serial_port.c_str());
    }
    downlink_packet_topic_ = declare_parameter<std::string>(
        "downlink_packet_topic", "/r2_serial/downlink/packet");
    uplink_packet_topic_ = declare_parameter<std::string>(
        "uplink_packet_topic", "/r2_serial/uplink/packet");
    height_compensation_enabled_ = declare_parameter<bool>(
        "height_compensation.enabled", true);
    height_compensation_x_per_z_ = declare_parameter<double>(
        "height_compensation.x_per_z", -0.52);
    height_compensation_y_per_z_ = declare_parameter<double>(
        "height_compensation.y_per_z", 0.0);
    height_compensation_use_initial_z_ = declare_parameter<bool>(
        "height_compensation.use_initial_z_as_reference", true);
    height_compensation_reference_z_ = declare_parameter<double>(
        "height_compensation.reference_z", 0.0);
    const auto staged_values = declare_parameter<std::vector<double>>(
        "sequence.staged_up_targets", std::vector<double>{});
    const auto home_values = declare_parameter<std::vector<double>>(
        "sequence.home_target", std::vector<double>{0.0, 0.0, 0.0});

    configureSequenceTargets(staged_values, home_values);

    downlink_packet_pub_ = create_publisher<r2_serial::msg::SerialPacket>(
        downlink_packet_topic_, 50);
    uplink_packet_sub_ = create_subscription<r2_serial::msg::SerialPacket>(
        uplink_packet_topic_, 50,
        std::bind(&SimpleOdomSender::handlePacket, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&SimpleOdomSender::odomCallback, this, std::placeholders::_1));
    gripper_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
        "/vision/weapon_cmd_state", 10);

    pole_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
        "/vision/pole_cmd_state", 10);

    keyboard_thread_ = std::thread(&SimpleOdomSender::keyboardLoop, this);
    keyboard_thread_.detach();

    RCLCPP_INFO(get_logger(), "订阅里程计: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "下发数据发布到: %s", downlink_packet_topic_.c_str());
    RCLCPP_INFO(get_logger(), "MCU 回传订阅自: %s", uplink_packet_topic_.c_str());
    if (height_compensation_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "升降高度补偿已启用: x_per_z=%.6f y_per_z=%.6f reference=%s%.3f",
                  height_compensation_x_per_z_, height_compensation_y_per_z_,
                  height_compensation_use_initial_z_ ? "initial_z=" : "fixed_z=",
                  height_compensation_reference_z_);
    }

    std::printf("\n======================================================\n");
    std::printf("|==================== 系统启动成功 ==================|\n");
    std::printf("| T x y yaw (下发目标)     | A x y yaw (手动发位置)  |\n");
    std::printf("| Q (查询状态)             | L n (查询近 n 条日志)   |\n");
    std::printf("| U (上台阶) / D (下台阶)  | S (紧急停止)            |\n");
    std::printf("| H (进入高位) / N (进入低位)                         |\n");
    std::printf("| M (执行分步上台阶序列)   | J1/J2/J3 (切换夹爪状态) |\n");
    std::printf("======================================================\n> ");
  }

private:
  uint64_t nowMs() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  std::optional<std::int16_t> checkedInt16(double value, const char *field) const {
    const double rounded = std::round(value);
    if (!std::isfinite(rounded) ||
        rounded < std::numeric_limits<std::int16_t>::min() ||
        rounded > std::numeric_limits<std::int16_t>::max()) {
      RCLCPP_WARN(get_logger(), "%s 超出 int16_t 范围: %.3f", field, value);
      return std::nullopt;
    }
    return static_cast<std::int16_t>(rounded);
  }

  std::optional<TargetPose> targetFromValues(const std::vector<double> &values,
                                             std::size_t offset) const {
    const auto x_mm = checkedInt16(values[offset], "目标 X(mm)");
    const auto y_mm = checkedInt16(values[offset + 1], "目标 Y(mm)");
    const auto yaw_deg = checkedInt16(values[offset + 2], "目标 Yaw(deg)");
    if (!x_mm || !y_mm || !yaw_deg) {
      return std::nullopt;
    }
    return TargetPose{*x_mm, *y_mm, *yaw_deg};
  }

  void configureSequenceTargets(const std::vector<double> &staged_values,
                                const std::vector<double> &home_values) {
    if (staged_values.size() == 9) {
      have_staged_up_targets_ = true;
      for (std::size_t index = 0; index < staged_up_targets_.size(); ++index) {
        const auto target = targetFromValues(staged_values, index * 3);
        if (!target) {
          have_staged_up_targets_ = false;
          break;
        }
        staged_up_targets_[index] = *target;
      }
    } else if (!staged_values.empty()) {
      RCLCPP_WARN(get_logger(),
                  "sequence.staged_up_targets 需要 9 个数，实际为 %zu；M 流程已禁用",
                  staged_values.size());
    }

    if (home_values.size() == 3) {
      const auto target = targetFromValues(home_values, 0);
      if (target) {
        home_target_ = *target;
      }
    }
  }

  bool eventPayloadLooksValid(const r2_serial::msg::SerialPacket &packet,
                              const char *name) {
    // 兼容两种 MCU 写法：老版事件无 payload；新版表格里事件 payload 为 uint16_t。
    if (packet.payload.empty() || packet.payload.size() == sizeof(std::uint16_t)) {
      return true;
    }
    RCLCPP_WARN(get_logger(), "忽略格式错误的%s包: payload=%zu", name,
                packet.payload.size());
    return false;
  }

  void handlePacket(const r2_serial::msg::SerialPacket::SharedPtr packet) {
    {
      std::lock_guard<std::mutex> lock(rx_mutex_);
      char buf[64];
      std::snprintf(buf, sizeof(buf), "code=0x%04x | payload=%zu bytes",
                    packet->code, packet->payload.size());
      rx_history_.push_back(buf);
      if (rx_history_.size() > 50) {
        rx_history_.pop_front();
      }
    }

    switch (packet->code) {
    case simple_odom_protocol::kMotionCompleted:
      if (eventPayloadLooksValid(*packet, "底盘到达目标")) handleMotionCompleted();
      return;
    case simple_odom_protocol::kHighModeEntered:
      if (eventPayloadLooksValid(*packet, "高位模式已进入")) handleHighModeEntered();
      return;
    case simple_odom_protocol::kLoweringCompleted:
      if (eventPayloadLooksValid(*packet, "降位完成")) handleLoweringCompleted();
      return;
    case simple_odom_protocol::kAtomicStairUpCompleted:
      if (eventPayloadLooksValid(*packet, "上台阶完成")) handleAtomicStairUpCompleted();
      return;
    case simple_odom_protocol::kAtomicStairDownCompleted:
      if (eventPayloadLooksValid(*packet, "下台阶完成")) handleAtomicStairDownCompleted();
      return;
    case simple_odom_protocol::kEmergencyStopCompleted:
      if (eventPayloadLooksValid(*packet, "急停完成")) handleEmergencyStopCompleted();
      return;
    default:
      return;
    }
  }

  std::string getStateString() const {
    switch (current_state_.load()) {
    case IDLE: return "IDLE (待机)";
    case MOVING: return "MOVING (正前往目标 T)";
    case WAITING_ATOMIC_UP: return "WAITING_ATOMIC_UP (等待上台阶完成)";
    case WAITING_ATOMIC_DOWN: return "WAITING_ATOMIC_DOWN (等待下台阶完成)";
    case STAGED_UP_WAIT_APPROACH_ARRIVAL: return "M序列: 等待到达台阶前";
    case STAGED_UP_WAIT_HIGH_MODE: return "M序列: 等待高位模式展开";
    case STAGED_UP_WAIT_LOWERING_COMPLETED: return "M序列: 等待降位完成";
    case STAGED_UP_WAIT_EXIT_ARRIVAL: return "M序列: 等待完全驶出台阶";
    case SEQ_WAIT_DOWN_COMPLETED: return "M序列: 等待下台阶完成";
    case SEQ_WAIT_HOME_ARRIVAL: return "M序列: 等待返回原点";
    default: return "UNKNOWN";
    }
  }

  void keyboardLoop() {
    std::string line;
    while (rclcpp::ok()) {
      std::getline(std::cin, line);
      if (line.empty()) continue;

      char first_char = static_cast<char>(
          std::tolower(static_cast<unsigned char>(line[0])));

      if (first_char == 'q') {
        printStatus();
        continue;
      }

      if (first_char == 'l') {
        printRecentLogs(line);
        continue;
      }

      if (first_char == 's') {
        std::printf("\n[急停下发] 发送急停命令 0x0105，退出移动状态并清空下发节点待发送队列。\n> ");
        current_state_ = IDLE;
        sendPacket(simple_odom_protocol::kEmergencyStop, {}, true);
        continue;
      }

      if (first_char == 'h') {
        sendAtomicCommand(simple_odom_protocol::kEnterHighMode,
                          "\n[手动指令] 进入高位模式");
        continue;
      }

      if (first_char == 'n') {
        sendAtomicCommand(simple_odom_protocol::kEnterLowMode,
                          "\n[手动指令] 进入低位模式");
        continue;
      }

      if (first_char == 'a') {
        handleManualPosition(line);
        continue;
      }

      if (first_char == 't') {
        handleTargetCommand(line);
        continue;
      }

      if (first_char == 'm') {
        startStagedSequence();
        continue;
      }

      if (first_char == 'u') {
        sendAtomicCommand(simple_odom_protocol::kAtomicStairUp,
                          "\n[手动指令] 原子上台阶");
        current_state_ = WAITING_ATOMIC_UP;
        continue;
      }

      if (first_char == 'd') {
        sendAtomicCommand(simple_odom_protocol::kAtomicStairDown,
                          "\n[手动指令] 原子下台阶");
        current_state_ = WAITING_ATOMIC_DOWN;
        continue;
      }

      if (first_char == 'j') {
        handleGripperCommand(line);
        continue;
      }

      std::printf("未知指令，请参考启动菜单。\n> ");
    }
  }

  void printStatus() {
    const auto last_send = last_send_time_ms_.load();
    const bool have_success = last_send != 0;
    const std::uint64_t time_since_last_send = have_success ? nowMs() - last_send : 0;
    const bool downlink_connected =
        downlink_packet_pub_ && downlink_packet_pub_->get_subscription_count() > 0;
    const bool is_sending_ok = downlink_connected && have_success && time_since_last_send < 1000;

    std::printf("\n================ [当前状态报告] ================\n");
    std::printf("实时雷达坐标 : X: %d mm | Y: %d mm | Z: %d mm | Yaw: %d deg\n",
                current_x_.load(), current_y_.load(), current_z_.load(), current_yaw_deg_.load());
    if (height_compensation_enabled_) {
      std::printf("升降补偿状态 : ref_z=%d mm | dx=%d mm | dy=%d mm\n",
                  height_compensation_reference_z_mm_.load(),
                  height_compensation_dx_mm_.load(),
                  height_compensation_dy_mm_.load());
    }
    std::printf("下发节点连接 : %s (topic: %s)\n",
                downlink_connected ? "已连接" : "未发现订阅者",
                downlink_packet_topic_.c_str());
    if (have_success) {
      std::printf("下发发布状态 : %s (距上次发布 %llu ms)\n",
                  is_sending_ok ? "正常发布中" : "异常或静默",
                  static_cast<unsigned long long>(time_since_last_send));
      std::printf("最近发布包   : code=0x%04x payload=%zu bytes\n",
                  last_success_code_.load(), last_success_bytes_.load());
    } else {
      std::printf("下发发布状态 : 尚无成功发布记录\n");
    }
    std::printf("发布统计     : 成功=%llu 失败=%llu\n",
                static_cast<unsigned long long>(tx_success_count_.load()),
                static_cast<unsigned long long>(tx_failure_count_.load()));
    std::printf("小车运动状态 : %s\n", getStateString().c_str());
    std::printf("================================================\n> ");
  }

  void printRecentLogs(const std::string &line) {
    int count = 10;
    std::sscanf(line.c_str(), "%*c %d", &count);
    count = std::max(1, std::min(count, 50));
    std::lock_guard<std::mutex> lock(rx_mutex_);
    std::printf("\n[MCU 最新回传信息 (近 %d 条)]\n", count);
    if (rx_history_.empty()) {
      std::printf("  (空，暂无收到任何数据；请确认 r2_serial 的 r2_data_downlink 正在发布 /r2_serial/uplink/packet)\n");
    } else {
      int displayed = 0;
      for (auto it = rx_history_.rbegin(); it != rx_history_.rend() && displayed < count;
           ++it, ++displayed) {
        std::printf("  -> %s\n", it->c_str());
      }
    }
    std::printf("> ");
  }

  void handleManualPosition(const std::string &line) {
    int ax, ay, ayaw;
    if (std::sscanf(line.c_str(), "%*c %d %d %d", &ax, &ay, &ayaw) == 3) {
      const std::vector<double> values{static_cast<double>(ax),
                                       static_cast<double>(ay),
                                       static_cast<double>(ayaw)};
      const auto pose = targetFromValues(values, 0);
      if (pose && sendPosition(pose->x_mm, pose->y_mm, pose->yaw_deg)) {
        std::printf("\n[手动汇报] 手动下发位置成功: X=%d Y=%d Yaw=%d\n> ",
                    pose->x_mm, pose->y_mm, pose->yaw_deg);
      } else {
        std::printf("\n[手动汇报失败] 坐标超出 int16_t 范围或下发节点未连接。\n> ");
      }
    } else {
      std::printf("格式错误！请输入: A x y yaw\n> ");
    }
  }

  void handleTargetCommand(const std::string &line) {
    int tx, ty, tyaw;
    if (std::sscanf(line.c_str(), "%*c %d %d %d", &tx, &ty, &tyaw) == 3) {
      const std::vector<double> values{static_cast<double>(tx),
                                       static_cast<double>(ty),
                                       static_cast<double>(tyaw)};
      const auto target = targetFromValues(values, 0);
      if (target) {
        sendTarget(*target);
        current_state_ = MOVING;
      }
    } else {
      std::printf("格式错误！请输入: T x y yaw\n> ");
    }
  }

  void handleGripperCommand(const std::string &line) {
    if (line.size() < 2) {
      std::printf("\n格式错误，请输入 J1, J2 或 J3\n> ");
      return;
    }

    char second_char = line[1];
    if (second_char == '1') {
      publishGripperState(GRIPPER_IDLE);
      std::printf("\n[视觉夹爪] 手动切换 -> J1: 待机/锁定状态\n> ");
    } else if (second_char == '2') {
      publishGripperState(GRIPPER_TRACKING);
      std::printf("\n[视觉夹爪] 手动切换 -> J2: 视觉微调伺服状态\n> ");
    } else if (second_char == '3') {
      publishGripperState(GRIPPER_GRAB);
      std::printf("\n[视觉夹爪] 手动切换 -> J3: 闭合收网状态\n> ");
    } else {
      std::printf("\n未知 J 指令，请输入 J1, J2 或 J3\n> ");
    }
  }

  void startStagedSequence() {
    if (!have_staged_up_targets_) {
      std::printf("\n[M 流程未启动] 请设置 sequence.staged_up_targets。\n> ");
      return;
    }
    std::printf("\n[自动序列启动] 开始执行分步上台阶流程...\n");
    sendTarget(staged_up_targets_[0]);
    current_state_ = STAGED_UP_WAIT_APPROACH_ARRIVAL;
  }

  std::vector<std::uint8_t> positionPayload(std::int16_t x_mm,
                                            std::int16_t y_mm,
                                            std::int16_t yaw_deg) const {
    std::vector<std::uint8_t> payload;
    payload.reserve(6);
    simple_odom_protocol::appendInt16Le(payload, x_mm);
    simple_odom_protocol::appendInt16Le(payload, y_mm);
    simple_odom_protocol::appendInt16Le(payload, yaw_deg);
    return payload;
  }

  std::vector<std::uint8_t> targetPayload(const TargetPose &target) const {
    return positionPayload(target.x_mm, target.y_mm, target.yaw_deg);
  }

  bool sendPacket(std::uint16_t code,
                  const std::vector<std::uint8_t> &payload,
                  bool clear_pending = false) {
    if (!downlink_packet_pub_ || downlink_packet_pub_->get_subscription_count() == 0) {
      tx_failure_count_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "未发现 r2_serial/r2_data_downlink 订阅者，暂不发布 code=0x%04x", code);
      return false;
    }

    r2_serial::msg::SerialPacket msg;
    msg.code = code;
    msg.payload = payload;
    msg.clear_pending = clear_pending;
    downlink_packet_pub_->publish(msg);

    last_send_time_ms_.store(nowMs());
    last_success_code_.store(code);
    last_success_bytes_.store(payload.size());
    tx_success_count_.fetch_add(1);
    return true;
  }

  void sendTarget(const TargetPose &target) {
    sendPacket(simple_odom_protocol::kTargetPose, targetPayload(target));
    std::printf("\n[下发目标] code=0x%04x X=%d mm Y=%d mm Yaw=%d deg\n> ",
                simple_odom_protocol::kTargetPose, target.x_mm, target.y_mm,
                target.yaw_deg);
  }

  bool sendPosition(std::int16_t x_mm, std::int16_t y_mm, std::int16_t yaw_deg) {
    return sendPacket(simple_odom_protocol::kPoseUpdate,
                      positionPayload(x_mm, y_mm, yaw_deg));
  }

  void sendAtomicCommand(std::uint16_t code, const char *message) {
    sendPacket(code, {});
    std::printf("%s: code=0x%04x\n> ", message, code);
  }

  void warnUnexpected(const char *event) {
    (void)event;
    // 当前阶段保持终端清爽，必要时再打开这里的状态机调试输出。
  }

  void handleMotionCompleted() {
    switch (current_state_.load()) {
    case MOVING:
      std::printf("\n[移动完成] 收到底盘到位包，底盘已停稳。\n> ");
      current_state_ = IDLE;
      return;
    case STAGED_UP_WAIT_APPROACH_ARRIVAL:
      std::printf("\n[分步上台阶 1/4] 已到达台阶前，等待高位模式反馈。\n> ");
      current_state_ = STAGED_UP_WAIT_HIGH_MODE;
      return;
    case STAGED_UP_WAIT_EXIT_ARRIVAL:
      std::printf("\n[分步上台阶 4/4] 已到达台阶后目标，开始原子下台阶。\n> ");
      sendAtomicCommand(simple_odom_protocol::kAtomicStairDown, "[序列指令] 原子下台阶");
      current_state_ = SEQ_WAIT_DOWN_COMPLETED;
      return;
    case SEQ_WAIT_HOME_ARRIVAL:
      std::printf("\n[序列完成] 成功返回原点。\n> ");
      current_state_ = IDLE;
      return;
    default:
      warnUnexpected("底盘到达目标");
      return;
    }
  }

  void handleHighModeEntered() {
    if (current_state_ != STAGED_UP_WAIT_HIGH_MODE) return;
    std::printf("\n[分步上台阶 2/4] 已进入高位模式，下发台阶内目标。\n> ");
    sendTarget(staged_up_targets_[1]);
    current_state_ = STAGED_UP_WAIT_LOWERING_COMPLETED;
  }

  void handleLoweringCompleted() {
    if (current_state_ != STAGED_UP_WAIT_LOWERING_COMPLETED) return;
    std::printf("\n[分步上台阶 3/4] 降位完成，下发离开台阶目标。\n> ");
    sendTarget(staged_up_targets_[2]);
    current_state_ = STAGED_UP_WAIT_EXIT_ARRIVAL;
  }

  void handleAtomicStairUpCompleted() {
    if (current_state_ != WAITING_ATOMIC_UP) return;
    std::printf("\n[手动上台阶完成] 收到 0x0204。\n> ");
    current_state_ = IDLE;
  }

  void handleAtomicStairDownCompleted() {
    if (current_state_ == WAITING_ATOMIC_DOWN) {
      std::printf("\n[手动下台阶完成] 收到 0x0205。\n> ");
      current_state_ = IDLE;
      return;
    }
    if (current_state_ == SEQ_WAIT_DOWN_COMPLETED) {
      std::printf("\n[序列] 下台阶完成，返回原点。\n> ");
      sendTarget(home_target_);
      current_state_ = SEQ_WAIT_HOME_ARRIVAL;
      return;
    }
  }

  void handleEmergencyStopCompleted() {
    std::printf("\n[急停完成] 收到 0x0206，状态已回到待机。\n> ");
    current_state_ = IDLE;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    constexpr double kOffsetX = 0.0834;
    constexpr double kOffsetY = 0.2661;

    const double lidar_z = msg->pose.pose.position.z;
    double lidar_x = msg->pose.pose.position.x;
    double lidar_y = msg->pose.pose.position.y;

    if (height_compensation_enabled_) {
      if (height_compensation_use_initial_z_ && !height_reference_initialized_.exchange(true)) {
        height_compensation_reference_z_ = lidar_z;
      }
      const double dz = lidar_z - height_compensation_reference_z_;
      const double dx = dz * height_compensation_x_per_z_;
      const double dy = dz * height_compensation_y_per_z_;
      lidar_x -= dx;
      lidar_y -= dy;
      height_compensation_reference_z_mm_.store(
          static_cast<std::int16_t>(std::lround(height_compensation_reference_z_ * 1000.0)));
      height_compensation_dx_mm_.store(static_cast<std::int16_t>(std::lround(dx * 1000.0)));
      height_compensation_dy_mm_.store(static_cast<std::int16_t>(std::lround(dy * 1000.0)));
    }

    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    const double base_x = lidar_x - (kOffsetX * std::cos(yaw) - kOffsetY * std::sin(yaw));
    const double base_y = lidar_y - (kOffsetX * std::sin(yaw) + kOffsetY * std::cos(yaw));
    const auto mm_x = checkedInt16(std::lround(base_x * 1000.0), "位置 X");
    const auto mm_y = checkedInt16(std::lround(base_y * 1000.0), "位置 Y");
    const auto yaw_deg = checkedInt16(std::lround(yaw * 180.0 / M_PI), "位置 Yaw");
    if (!mm_x || !mm_y || !yaw_deg) return;

    current_x_.store(*mm_x);
    current_y_.store(*mm_y);
    const auto mm_z = checkedInt16(std::lround(lidar_z * 1000.0), "位置 Z");
    if (mm_z) {
      current_z_.store(*mm_z);
    }
    current_yaw_deg_.store(*yaw_deg);

    // 持续上报当前位置，但只发布到统一下发节点，不直接占用串口。
    sendPosition(*mm_x, *mm_y, *yaw_deg);
  }

  void publishGripperState(std::uint8_t state) {
    if (current_gripper_state_ == state) {
      return;
    }
    std_msgs::msg::UInt8 msg;
    msg.data = state;

    gripper_state_pub_->publish(msg);
    if (pole_state_pub_) {
      pole_state_pub_->publish(msg);
    }
    
    current_gripper_state_ = state;
  }

  std::thread keyboard_thread_;
  std::string odom_topic_;
  std::string downlink_packet_topic_;
  std::string uplink_packet_topic_;
  bool height_compensation_enabled_{true};
  bool height_compensation_use_initial_z_{true};
  double height_compensation_x_per_z_{-0.52};
  double height_compensation_y_per_z_{0.0};
  double height_compensation_reference_z_{0.0};
  std::atomic<bool> height_reference_initialized_{false};

  std::array<TargetPose, 3> staged_up_targets_{};
  TargetPose home_target_{0, 0, 0};
  bool have_staged_up_targets_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<r2_serial::msg::SerialPacket>::SharedPtr uplink_packet_sub_;
  rclcpp::Publisher<r2_serial::msg::SerialPacket>::SharedPtr downlink_packet_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gripper_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pole_state_pub_;

  std::atomic<MissionState> current_state_;
  std::atomic<std::int16_t> current_x_{0};
  std::atomic<std::int16_t> current_y_{0};
  std::atomic<std::int16_t> current_z_{0};
  std::atomic<std::int16_t> current_yaw_deg_{0};
  std::atomic<std::int16_t> height_compensation_reference_z_mm_{0};
  std::atomic<std::int16_t> height_compensation_dx_mm_{0};
  std::atomic<std::int16_t> height_compensation_dy_mm_{0};

  std::atomic<std::uint64_t> last_send_time_ms_{0};
  std::atomic<std::uint64_t> tx_success_count_{0};
  std::atomic<std::uint64_t> tx_failure_count_{0};
  std::atomic<std::uint16_t> last_success_code_{0};
  std::atomic<std::size_t> last_success_bytes_{0};
  std::deque<std::string> rx_history_;
  std::mutex rx_mutex_;

  std::uint8_t current_gripper_state_ = GRIPPER_IDLE;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleOdomSender>());
  rclcpp::shutdown();
  return 0;
}
