#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <asio.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "r2_serial/msg/serial_packet.hpp"
#include "r2_serial/serial_connector.hpp"
#include "r2_serial/serial_protocol.hpp"

namespace protocol = r2_serial::protocol;

namespace {

std::vector<std::uint8_t> makeNavPayload(std::int16_t x_mm,
                                         std::int16_t y_mm,
                                         std::int16_t yaw_deg) {
  std::vector<std::uint8_t> payload;
  payload.reserve(6);
  protocol::appendInt16Le(payload, x_mm);
  protocol::appendInt16Le(payload, y_mm);
  protocol::appendInt16Le(payload, yaw_deg);
  return payload;
}

std::optional<std::int16_t> checkedInt16(double value) {
  const double rounded = std::round(value);
  if (!std::isfinite(rounded) ||
      rounded < std::numeric_limits<std::int16_t>::min() ||
      rounded > std::numeric_limits<std::int16_t>::max()) {
    return std::nullopt;
  }
  return static_cast<std::int16_t>(rounded);
}

std::string bytesToHex(const std::uint8_t *data, std::size_t size) {
  std::ostringstream bytes;
  bytes << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < size; ++i) {
    if (i != 0) {
      bytes << ' ';
    }
    bytes << std::setw(2) << static_cast<int>(data[i]);
  }
  return bytes.str();
}

}  

class R2DataDownlinkNode : public rclcpp::Node {
public:
  using packet_t = SerialConnector::packet_t;

  R2DataDownlinkNode()
      : Node("r2_data_downlink"),
        work_guard_(asio::make_work_guard(io_context_)) {
    readParameters();
    initializeSerial(true);
    createInterfaces();
    createReconnectTimer();
    startConsoleThread();

    RCLCPP_INFO(get_logger(),
                "R2 串口收发节点已启动: 串口=%s, 下发话题=%s, 回传话题=%s, 里程计转发=%s",
                serial_port_.c_str(), raw_packet_topic_.c_str(),
                uplink_packet_topic_.c_str(),
                pose_odom_topic_.empty() ? "<关闭>" : pose_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "串口节点控制台: 输入 L n 可查看最近 n 条下位机回传包");
  }

  ~R2DataDownlinkNode() override {
    work_guard_.reset();
    io_context_.stop();
    if (io_context_thread_.joinable()) {
      io_context_thread_.join();
    }
    serial_connector_.reset();
  }

private:
  void readParameters() {
    // 所有话题都做成参数，后续联调时可以 launch 覆盖，不需要改源码。
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    serial_debug_raw_ = declare_parameter<bool>("serial_debug_raw", false);
    write_rate_limit_enabled_ = declare_parameter<bool>("write_rate_limit.enabled", true);
    write_min_interval_ms_ = declare_parameter<int>("write_rate_limit.min_interval_ms", 10);
    debug_print_pose_tx_ = declare_parameter<bool>("debug.print_pose_tx", false);
    debug_pose_tx_summary_ms_ = declare_parameter<int>("debug.pose_tx_summary_ms", 20000);
    debug_drop_summary_every_n_ = declare_parameter<int>("debug.drop_summary_every_n", 50);
    reconnect_enabled_ = declare_parameter<bool>("reconnect.enabled", true);
    reconnect_interval_ms_ = declare_parameter<int>("reconnect.interval_ms", 1000);
    reconnect_log_every_n_ = declare_parameter<int>("reconnect.log_every_n", 10);

    raw_packet_topic_ = declare_parameter<std::string>(
        "topics.raw_packet", "/r2_serial/downlink/packet");
    raw_packet_r2_topic_ = declare_parameter<std::string>(
        "topics.raw_packet_r2", "/r2/downlink/packet");
    raw_packet_legacy_topic_ = declare_parameter<std::string>(
        "topics.raw_packet_legacy", "");
    nav_position_topic_ = declare_parameter<std::string>(
        "topics.nav_position_mm", "/r2_serial/downlink/nav_position_mm");
    nav_target_topic_ = declare_parameter<std::string>(
        "topics.nav_target_mm", "/r2_serial/downlink/nav_target_mm");
    stair_up_topic_ = declare_parameter<std::string>(
        "topics.stair_up", "/r2_serial/downlink/stair_up");
    stair_down_topic_ = declare_parameter<std::string>(
        "topics.stair_down", "/r2_serial/downlink/stair_down");
    emergency_stop_topic_ = declare_parameter<std::string>(
        "topics.emergency_stop", "/r2_serial/downlink/emergency_stop");
    enter_high_mode_topic_ = declare_parameter<std::string>(
        "topics.enter_high_mode", "/r2_serial/downlink/enter_high_mode");
    enter_low_mode_topic_ = declare_parameter<std::string>(
        "topics.enter_low_mode", "/r2_serial/downlink/enter_low_mode");
    pose_odom_topic_ = declare_parameter<std::string>("topics.pose_odom", "");

    path_forward_topic_ = declare_parameter<std::string>(
        "topics.path.forward", "/r2_serial/downlink/path/forward");
    path_backward_topic_ = declare_parameter<std::string>(
        "topics.path.backward", "/r2_serial/downlink/path/backward");
    path_turn_left_90_topic_ = declare_parameter<std::string>(
        "topics.path.turn_left_90", "/r2_serial/downlink/path/turn_left_90");
    path_turn_right_90_topic_ = declare_parameter<std::string>(
        "topics.path.turn_right_90", "/r2_serial/downlink/path/turn_right_90");
    path_shift_left_topic_ = declare_parameter<std::string>(
        "topics.path.shift_left", "/r2_serial/downlink/path/shift_left");
    path_shift_right_topic_ = declare_parameter<std::string>(
        "topics.path.shift_right", "/r2_serial/downlink/path/shift_right");
    path_grab_low_kfs_topic_ = declare_parameter<std::string>(
        "topics.path.grab_low_kfs", "/r2_serial/downlink/path/grab_low_kfs");
    path_grab_mid_kfs_topic_ = declare_parameter<std::string>(
        "topics.path.grab_mid_kfs", "/r2_serial/downlink/path/grab_mid_kfs");
    path_grab_high_kfs_topic_ = declare_parameter<std::string>(
        "topics.path.grab_high_kfs", "/r2_serial/downlink/path/grab_high_kfs");
    path_replace_kfs_topic_ = declare_parameter<std::string>(
        "topics.path.replace_kfs", "/r2_serial/downlink/path/replace_kfs");
    path_no_command_topic_ = declare_parameter<std::string>(
        "topics.path.no_command", "/r2_serial/downlink/path/no_command");
    path_turn_around_180_topic_ = declare_parameter<std::string>(
      "topics.path.turn_around_180", "/r2_serial/downlink/path/turn_around_180");

    uplink_packet_topic_ = declare_parameter<std::string>(
        "topics.uplink_packet", "/r2_serial/uplink/packet");
    uplink_packet_r2_topic_ = declare_parameter<std::string>(
        "topics.uplink_packet_r2", "/r2/uplink/packet");
    uplink_event_topic_ = declare_parameter<std::string>(
        "topics.uplink_event_code", "/r2_serial/uplink/event_code");
    uplink_event_r2_topic_ = declare_parameter<std::string>(
        "topics.uplink_event_code_r2", "/r2/uplink/event_code");
    path_request_topic_ = declare_parameter<std::string>(
        "topics.path.request_next", "/r2_serial/uplink/path_request_next");
    path_request_new_topic_ = declare_parameter<std::string>(
        "topics.path.request_next_new", "/r2_serial/uplink/path_request_next_new");
    vision_weapon_pole_state_topic_ = declare_parameter<std::string>(
        "topics.vision_weapon_pole_state", "/vision/weapon_pole_cmd_state_2");
  }

  void initializeSerial(bool initial_attempt) {
    // 串口只在这里打开；断线重连时会重新创建连接对象，并丢弃旧连接里的待发包。
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_connector_ && serial_connected_.load()) {
        return;
      }
    }

    try {
      auto connector = std::make_shared<SerialConnector>(serial_port_, io_context_);
      const int interval_ms = write_rate_limit_enabled_
                                  ? std::max(0, write_min_interval_ms_)
                                  : 0;
      connector->setMinWriteInterval(std::chrono::milliseconds(interval_ms));
      connector->setErrorHandler([this](std::error_code ec) {
        handleSerialError(ec);
      });
      if (serial_debug_raw_) {
        connector->setRawReceiveHandler(
            [this](const std::uint8_t *data, std::size_t size) {
              RCLCPP_INFO(get_logger(), "串口原始接收: %zu bytes [%s]", size,
                          bytesToHex(data, size).c_str());
            });
      }

      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_connector_ = std::move(connector);
        serial_connected_.store(true);
        reconnect_failure_count_.store(0);
        disconnected_drop_count_.store(0);
      }

      if (!io_context_thread_.joinable()) {
        io_context_thread_ = std::thread([this]() { io_context_.run(); });
      }
      startAsyncReceive();
      RCLCPP_INFO(get_logger(), "串口配置成功: %s, 115200 8N1", serial_port_.c_str());
      RCLCPP_INFO(get_logger(), "串口下发限速: %s, 最小间隔=%d ms",
                  interval_ms > 0 ? "开启" : "关闭", interval_ms);
    } catch (const std::exception &ex) {
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_connector_.reset();
        serial_connected_.store(false);
      }
      if (initial_attempt) {
        RCLCPP_ERROR(get_logger(), "串口启动失败: %s (%s)，等待自动重连",
                     serial_port_.c_str(), ex.what());
      } else {
        const auto attempts = reconnect_failure_count_.fetch_add(1) + 1;
        const auto log_every = std::max(1, reconnect_log_every_n_);
        if (attempts % static_cast<std::uint64_t>(log_every) == 0) {
          RCLCPP_WARN(get_logger(),
                      "串口仍未恢复: %s (%s)，已重试 %llu 次",
                      serial_port_.c_str(), ex.what(),
                      static_cast<unsigned long long>(attempts));
        }
      }
    }
  }

  void createReconnectTimer() {
    if (!reconnect_enabled_) {
      return;
    }
    const int interval_ms = std::max(100, reconnect_interval_ms_);
    reconnect_timer_ = create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this]() {
          if (!serial_connected_.load()) {
            initializeSerial(false);
          }
        });
    RCLCPP_INFO(get_logger(), "串口自动重连: 开启，间隔=%d ms", interval_ms);
  }

  void startConsoleThread() {
    console_thread_ = std::thread(&R2DataDownlinkNode::consoleLoop, this);
    console_thread_.detach();
  }

  void consoleLoop() {
    std::string line;
    while (rclcpp::ok() && std::getline(std::cin, line)) {
      if (line.empty()) {
        continue;
      }
      const char cmd = static_cast<char>(
          std::tolower(static_cast<unsigned char>(line.front())));
      if (cmd == 'l') {
        printRecentUplink(line);
      } else {
        std::printf("\n[r2_serial] 当前只支持 L n：查看最近 n 条下位机回传包。\n");
      }
    }
  }

  void printRecentUplink(const std::string &line) {
    int count = 10;
    std::sscanf(line.c_str(), "%*c %d", &count);
    count = std::max(1, std::min(count, 100));

    std::lock_guard<std::mutex> lock(uplink_history_mutex_);
    std::printf("\n========== [最近 %d 条下位机回传包] ==========\n", count);
    if (uplink_history_.empty()) {
      std::printf("  暂无回传包。请确认下位机已发送 AA55...55AA 协议包，且串口连接正常。\n");
    } else {
      int printed = 0;
      for (auto it = uplink_history_.rbegin();
           it != uplink_history_.rend() && printed < count; ++it, ++printed) {
        std::printf("  %s\n", it->c_str());
      }
    }
    std::printf("=============================================\n");
  }

  void handleSerialError(const std::error_code &ec) {
    if (ec == asio::error::operation_aborted) {
      return;
    }
    const bool was_connected = serial_connected_.exchange(false);
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_connector_) {
        serial_connector_->clearPendingWrites();
        serial_connector_.reset();
      }
    }
    if (was_connected) {
      RCLCPP_WARN(get_logger(),
                  "串口连接断开: %s (%s)。已清空未发送队列，等待设备恢复后自动重连。",
                  serial_port_.c_str(), ec.message().c_str());
    }
  }

  void createInterfaces() {
    createUplinkPacketPublisher(uplink_packet_topic_);
    createUplinkPacketPublisher(uplink_packet_r2_topic_);
    createUplinkEventPublisher(uplink_event_topic_);
    createUplinkEventPublisher(uplink_event_r2_topic_);
    path_request_pub_ = create_publisher<std_msgs::msg::Empty>(path_request_topic_, 10);
    path_request_new_pub_ = create_publisher<std_msgs::msg::UInt16>(path_request_new_topic_, 10);
    vision_weapon_pole_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
        vision_weapon_pole_state_topic_, 10);

    // 原始包入口：推荐 /r2_serial/downlink/packet，同时兼容旧 /r2/downlink/packet。
    createRawPacketSubscription(raw_packet_topic_);
    createRawPacketSubscription(raw_packet_r2_topic_);
    createRawPacketSubscription(raw_packet_legacy_topic_);

    nav_position_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        nav_position_topic_, 50,
        [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
          sendNavArray(protocol::kPoseUpdate, *msg, false, "nav_position_mm");
        });

    nav_target_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        nav_target_topic_, 50,
        [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
          sendNavArray(protocol::kTargetPose, *msg, false, "nav_target_mm");
        });

    stair_up_sub_ = createEmptyCommandSubscription(
        stair_up_topic_, protocol::kAtomicStairUp, false);
    stair_down_sub_ = createEmptyCommandSubscription(
        stair_down_topic_, protocol::kAtomicStairDown, false);
    emergency_stop_sub_ = createEmptyCommandSubscription(
        emergency_stop_topic_, protocol::kEmergencyStop, true);
    enter_high_mode_sub_ = createEmptyCommandSubscription(
        enter_high_mode_topic_, protocol::kEnterHighMode, false);
    enter_low_mode_sub_ = createEmptyCommandSubscription(
        enter_low_mode_topic_, protocol::kEnterLowMode, false);

    path_forward_sub_ = createEmptyCommandSubscription(
        path_forward_topic_, protocol::kPathForward, false);
    path_backward_sub_ = createEmptyCommandSubscription(
        path_backward_topic_, protocol::kPathBackward, false);
    path_turn_left_90_sub_ = createEmptyCommandSubscription(
        path_turn_left_90_topic_, protocol::kPathTurnLeft90, false);
    path_turn_right_90_sub_ = createEmptyCommandSubscription(
        path_turn_right_90_topic_, protocol::kPathTurnRight90, false);
    path_shift_left_sub_ = createEmptyCommandSubscription(
        path_shift_left_topic_, protocol::kPathShiftLeft, false);
    path_shift_right_sub_ = createEmptyCommandSubscription(
        path_shift_right_topic_, protocol::kPathShiftRight, false);
    path_grab_low_kfs_sub_ = createEmptyCommandSubscription(
        path_grab_low_kfs_topic_, protocol::kPathGrabLowKfs, false);
    path_grab_mid_kfs_sub_ = createEmptyCommandSubscription(
        path_grab_mid_kfs_topic_, protocol::kPathGrabMidKfs, false);
    path_grab_high_kfs_sub_ = createEmptyCommandSubscription(
        path_grab_high_kfs_topic_, protocol::kPathGrabHighKfs, false);
    path_replace_kfs_sub_ = createEmptyCommandSubscription(
        path_replace_kfs_topic_, protocol::kPathReplaceKfs, false);
    path_no_command_sub_ = createEmptyCommandSubscription(
        path_no_command_topic_, protocol::kPathNoCommand, false);
    path_turn_around_180_sub_ = createEmptyCommandSubscription(
        path_turn_around_180_topic_, protocol::kPathTurnAround180, false);

    if (!pose_odom_topic_.empty()) {
      pose_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          pose_odom_topic_, 50,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg) { sendOdomPose(*msg); });
    }
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr createEmptyCommandSubscription(
      const std::string &topic, std::uint16_t code, bool clear_pending) {
    return create_subscription<std_msgs::msg::Empty>(
        topic, 10,
        [this, code, clear_pending](const std_msgs::msg::Empty::SharedPtr) {
          sendPacket(code, {}, clear_pending);
        });
  }

  void createRawPacketSubscription(const std::string &topic) {
    if (topic.empty() || std::find(raw_packet_topics_.begin(), raw_packet_topics_.end(), topic) !=
                             raw_packet_topics_.end()) {
      return;
    }
    raw_packet_topics_.push_back(topic);
    raw_packet_subs_.push_back(create_subscription<r2_serial::msg::SerialPacket>(
        topic, 50,
        [this, topic](const r2_serial::msg::SerialPacket::SharedPtr msg) {
          RCLCPP_DEBUG(get_logger(), "收到 ROS 下发包: topic=%s code=0x%04x payload=%zu",
                       topic.c_str(), msg->code, msg->payload.size());
          sendPacket(msg->code, msg->payload, msg->clear_pending);
        }));
    RCLCPP_INFO(get_logger(), "订阅下发原始包: %s", topic.c_str());
  }

  void createUplinkPacketPublisher(const std::string &topic) {
    if (topic.empty() || std::find(uplink_packet_topics_.begin(), uplink_packet_topics_.end(), topic) !=
                             uplink_packet_topics_.end()) {
      return;
    }
    uplink_packet_topics_.push_back(topic);
    uplink_packet_pubs_.push_back(create_publisher<r2_serial::msg::SerialPacket>(topic, 50));
    RCLCPP_INFO(get_logger(), "发布回传原始包: %s", topic.c_str());
  }

  void createUplinkEventPublisher(const std::string &topic) {
    if (topic.empty() || std::find(uplink_event_topics_.begin(), uplink_event_topics_.end(), topic) !=
                             uplink_event_topics_.end()) {
      return;
    }
    uplink_event_topics_.push_back(topic);
    uplink_event_pubs_.push_back(create_publisher<std_msgs::msg::UInt16>(topic, 50));
    RCLCPP_INFO(get_logger(), "发布回传事件码: %s", topic.c_str());
  }


  void startAsyncReceive() {
    std::shared_ptr<SerialConnector> connector;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!serial_connector_ || !serial_connected_.load()) {
        return;
      }
      connector = serial_connector_;
    }
    connector->asyncReceive([this](std::error_code ec, packet_t packet) {
      if (!ec) {
        publishUplinkPacket(packet);
      }
      if (serial_connected_.load()) {
        startAsyncReceive();
      }
    });
  }

  void sendNavArray(std::uint16_t code,
                    const std_msgs::msg::Int16MultiArray &msg,
                    bool clear_pending,
                    const char *name) {
    if (msg.data.size() < 3) {
      RCLCPP_WARN(get_logger(), "%s 需要至少 3 个 int16: x_mm,y_mm,yaw_deg", name);
      return;
    }
    sendPacket(code, makeNavPayload(msg.data[0], msg.data[1], msg.data[2]), clear_pending);
  }

  void sendOdomPose(const nav_msgs::msg::Odometry &msg) {
    const auto x_mm = checkedInt16(std::lround(msg.pose.pose.position.x * 1000.0));
    const auto y_mm = checkedInt16(std::lround(msg.pose.pose.position.y * 1000.0));

    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf2::Matrix3x3 matrix(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    matrix.getRPY(roll, pitch, yaw);
    const auto yaw_deg = checkedInt16(std::lround(yaw * 180.0 / M_PI));
    if (!x_mm || !y_mm || !yaw_deg) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "里程计位置超出 int16 下发范围，跳过 0x0101");
      return;
    }
    sendPacket(protocol::kPoseUpdate, makeNavPayload(*x_mm, *y_mm, *yaw_deg), false);
  }

  bool sendPacket(std::uint16_t code,
                  const std::vector<std::uint8_t> &payload,
                  bool clear_pending) {
    // 这里才真正封成 AA 55 ... 55 AA 的 CRC16 数据包并写入串口。
    auto packet = std::make_shared<packet_t>(code, payload.begin(), payload.end(), gdut::build_packet);
    if (!*packet) {
      tx_failure_count_.fetch_add(1);
      RCLCPP_WARN(get_logger(), "串口包构造失败: code=0x%04x", code);
      return false;
    }

    const bool log_tx_detail =
        serial_debug_raw_ && (debug_print_pose_tx_ || code != protocol::kPoseUpdate);

    std::shared_ptr<SerialConnector> connector;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!serial_connector_ || !serial_connected_.load()) {
        tx_failure_count_.fetch_add(1);
        const auto dropped = disconnected_drop_count_.fetch_add(1) + 1;
        const auto log_every = std::max(1, debug_drop_summary_every_n_);
        if (dropped % static_cast<std::uint64_t>(log_every) == 0) {
          RCLCPP_WARN(get_logger(),
                      "串口未连接，已丢弃 %llu 个下发包，最近丢弃 code=0x%04x",
                      static_cast<unsigned long long>(dropped), code);
        }
        return false;
      }
      connector = serial_connector_;
    }

    if (log_tx_detail) {
      std::vector<std::uint8_t> bytes(packet->begin(), packet->end());
      RCLCPP_INFO(get_logger(), "串口原始发送: code=0x%04x %zu bytes [%s]",
                  code, bytes.size(), bytesToHex(bytes.data(), bytes.size()).c_str());
    }
    if (clear_pending) {
      connector->clearPendingWrites();
    }
    connector->asyncSend(
        *packet, [this, code, packet, log_tx_detail](std::error_code ec,
                                                     std::size_t bytes_transferred) {
          if (!ec && bytes_transferred == static_cast<std::size_t>(packet->size())) {
            tx_success_count_.fetch_add(1);
            if (log_tx_detail) {
              RCLCPP_INFO(get_logger(), "串口写入完成: code=0x%04x bytes=%zu",
                          code, bytes_transferred);
            } else if (serial_debug_raw_ && code == protocol::kPoseUpdate) {
              const auto hidden = suppressed_pose_tx_count_.fetch_add(1) + 1;
              RCLCPP_INFO_THROTTLE(
                  get_logger(), *get_clock(), debug_pose_tx_summary_ms_,
                  "已隐藏 0x0101 位置下发包详细日志，成功发送累计=%llu；如需完整打印，设置 debug_print_pose_tx:=true",
                  static_cast<unsigned long long>(hidden));
            }
          } else {
            tx_failure_count_.fetch_add(1);
            RCLCPP_WARN(get_logger(),
                        "串口写入失败: code=0x%04x bytes=%zu/%zu error=%s",
                        code, bytes_transferred,
                        static_cast<std::size_t>(packet->size()),
                        ec ? ec.message().c_str() : "short write");
          }
        });
    return true;
  }

  void publishUplinkPacket(const packet_t &packet) {
    r2_serial::msg::SerialPacket msg;
    msg.code = packet.code();
    msg.payload.assign(packet.body_data(), packet.body_data() + packet.body_size());
    msg.clear_pending = false;
    recordUplinkHistory(msg);
    for (const auto &pub : uplink_packet_pubs_) {
      pub->publish(msg);
    }

    std_msgs::msg::UInt16 event_msg;
    event_msg.data = packet.code();
    for (const auto &pub : uplink_event_pubs_) {
      pub->publish(event_msg);
    }

    publishVisionStateCommand(packet);
    publishPathRequest(packet);
    publishPathRequestNew(packet);

    if (serial_debug_raw_) {
      const auto payload_hex = bytesToHex(packet.body_data(), packet.body_size());
      RCLCPP_INFO(get_logger(), "收到有效串口包: code=0x%04x payload=[%s] len=%u bytes",
                  packet.code(), payload_hex.empty() ? "<empty>" : payload_hex.c_str(),
                  packet.body_size());
    }
  }

  void recordUplinkHistory(const r2_serial::msg::SerialPacket &msg) {
    std::ostringstream line;
    line << "code=0x" << std::hex << std::setw(4) << std::setfill('0') << msg.code
         << std::dec << " payload=[";
    if (msg.payload.empty()) {
      line << "<empty>";
    } else {
      line << bytesToHex(msg.payload.data(), msg.payload.size());
    }
    line << "] len=" << msg.payload.size() << " bytes";

    std::lock_guard<std::mutex> lock(uplink_history_mutex_);
    uplink_history_.push_back(line.str());
    while (uplink_history_.size() > 100) {
      uplink_history_.pop_front();
    }
  }

  void publishPathRequest(const packet_t &packet) {
    // 0x0301 是路径规划组最关心的触发信号：下位机请求上位机给下一条动作。
    if (packet.code() != protocol::kPathRequestNext) {
      return;
    }
    if (packet.body_size() != 0) {
      RCLCPP_WARN(get_logger(), "0x0301 请求下一条路径指令应为空 payload，实际=%u bytes，仍转发请求",
                  packet.body_size());
    }
    std_msgs::msg::Empty msg;
    path_request_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "转发路径规划请求: code=0x%04x -> %s",
                packet.code(), path_request_topic_.c_str());
  }

  void publishPathRequestNew(const packet_t &packet) {
    if (packet.code() != protocol::kPathRequestNextNew) {
      return;
    }
    if (packet.body_size() < sizeof(std::uint16_t)) {
      RCLCPP_ERROR(this->get_logger(),
        "new path planning request with a empty payload,"
        " ignore this request");
      return;
    }
    std_msgs::msg::UInt16 msg;
    std::uint16_t data{};
    data |= packet.body_data()[0];
    data |= (packet.body_data()[1] << 8) & 0xFF;
    msg.data = data;
    path_request_new_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "转发路径规划请求: code=0x%04x -> %s, index=%d",
                packet.code(), path_request_topic_.c_str(), static_cast<int>(msg.data));
  }

  void publishVisionStateCommand(const packet_t &packet) {
    // 0x0002/0x0003/0x0004 都按新版视觉统一状态控制处理，统一转发到 /vision/weapon_pole_cmd_state_2。
    const char *name = nullptr;
    if (packet.code() == protocol::kVisionStateLegacy) {
      name = "视觉统一状态(0x0002)";
    } else if (packet.code() == protocol::kVisionState) {
      name = "视觉统一状态(0x0003)";
    } else if (packet.code() == protocol::kVisionStateCompat) {
      name = "视觉统一状态(0x0004)";
    } else {
      return;
    }

    std::optional<int> state;
    if (packet.body_size() == sizeof(std::uint8_t)) {
      state = static_cast<int>(*packet.body_data());
    } else if (const auto state16 = protocol::readInt16Le(packet.body_data(), packet.body_size())) {
      state = static_cast<int>(*state16);
    }

    if (!state) {
      const auto payload_hex = bytesToHex(packet.body_data(), packet.body_size());
      RCLCPP_WARN(get_logger(),
                  "忽略格式错误的%s包: code=0x%04x payload=[%s] len=%u bytes，应为 uint8 或 int16_t",
                  name, packet.code(), payload_hex.empty() ? "<empty>" : payload_hex.c_str(),
                  packet.body_size());
      return;
    }
    if (*state < 0 || *state > std::numeric_limits<std::uint8_t>::max()) {
      RCLCPP_WARN(get_logger(), "忽略超出 UInt8 范围的%s: code=0x%04x state=%d",
                  name, packet.code(), *state);
      return;
    }
    if (*state > 4) {
      RCLCPP_WARN(get_logger(), "%s state=%d 超出当前视觉约定 0/1/2/3/4，仍转发给视觉节点处理",
                  name, *state);
    }

    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<std::uint8_t>(*state);
    vision_weapon_pole_state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "转发%s: code=0x%04x state=%d -> %s", name, packet.code(),
                *state, vision_weapon_pole_state_topic_.c_str());
  }

  asio::io_context io_context_;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  std::shared_ptr<SerialConnector> serial_connector_;
  std::thread io_context_thread_;
  std::thread console_thread_;
  std::mutex serial_mutex_;
  std::mutex uplink_history_mutex_;
  std::deque<std::string> uplink_history_;
  std::atomic<bool> serial_connected_{false};
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  std::string serial_port_;
  bool serial_debug_raw_{false};
  bool write_rate_limit_enabled_{true};
  int write_min_interval_ms_{10};
  bool debug_print_pose_tx_{false};
  int debug_pose_tx_summary_ms_{20000};
  int debug_drop_summary_every_n_{50};
  bool reconnect_enabled_{true};
  int reconnect_interval_ms_{1000};
  int reconnect_log_every_n_{10};

  std::string raw_packet_topic_;
  std::string raw_packet_r2_topic_;
  std::string raw_packet_legacy_topic_;
  std::string nav_position_topic_;
  std::string nav_target_topic_;
  std::string stair_up_topic_;
  std::string stair_down_topic_;
  std::string emergency_stop_topic_;
  std::string enter_high_mode_topic_;
  std::string enter_low_mode_topic_;
  std::string pose_odom_topic_;

  std::string path_forward_topic_;
  std::string path_backward_topic_;
  std::string path_turn_left_90_topic_;
  std::string path_turn_right_90_topic_;
  std::string path_shift_left_topic_;
  std::string path_shift_right_topic_;
  std::string path_grab_low_kfs_topic_;
  std::string path_grab_mid_kfs_topic_;
  std::string path_grab_high_kfs_topic_;
  std::string path_replace_kfs_topic_;
  std::string path_no_command_topic_;
  std::string path_turn_around_180_topic_;

  std::string uplink_packet_topic_;
  std::string uplink_packet_r2_topic_;
  std::string uplink_event_topic_;
  std::string uplink_event_r2_topic_;
  std::string path_request_topic_;
  std::string path_request_new_topic_;
  std::string vision_weapon_pole_state_topic_;

  std::atomic<std::uint64_t> tx_success_count_{0};
  std::atomic<std::uint64_t> tx_failure_count_{0};
  std::atomic<std::uint64_t> suppressed_pose_tx_count_{0};
  std::atomic<std::uint64_t> disconnected_drop_count_{0};
  std::atomic<std::uint64_t> reconnect_failure_count_{0};

  std::vector<std::string> raw_packet_topics_;
  std::vector<std::string> uplink_packet_topics_;
  std::vector<std::string> uplink_event_topics_;
  std::vector<rclcpp::Subscription<r2_serial::msg::SerialPacket>::SharedPtr> raw_packet_subs_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr nav_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr nav_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stair_up_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stair_down_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr enter_high_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr enter_low_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_forward_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_backward_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_turn_left_90_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_turn_right_90_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_shift_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_shift_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_grab_low_kfs_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_grab_mid_kfs_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_grab_high_kfs_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_replace_kfs_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_no_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_turn_around_180_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_odom_sub_;

  std::vector<rclcpp::Publisher<r2_serial::msg::SerialPacket>::SharedPtr> uplink_packet_pubs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr> uplink_event_pubs_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_request_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr path_request_new_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vision_weapon_pole_state_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R2DataDownlinkNode>());
  rclcpp::shutdown();
  return 0;
}
