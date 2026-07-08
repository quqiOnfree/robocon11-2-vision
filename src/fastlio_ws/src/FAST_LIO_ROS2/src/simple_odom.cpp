#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "r2_serial/msg/current_pose.hpp"
#include "r2_serial/msg/initial_position.hpp"
#include "r2_serial/msg/serial_packet.hpp"
#include "r2_serial/serial_protocol.hpp"

namespace protocol = r2_serial::protocol;

class R2PoseReporter : public rclcpp::Node {
public:
  enum class Mode : std::uint8_t { kLocalization, kOdometry };
  enum class Zone : std::uint8_t { kBlue = 0, kRed = 1 };

  R2PoseReporter() : Node("r2_pose_reporter") {
    readParameters();
    validateParameters();

    downlink_packet_pub_ = create_publisher<r2_serial::msg::SerialPacket>(
        downlink_packet_topic_, 50);
    initial_position_pub_ = create_publisher<r2_serial::msg::InitialPosition>(
        initial_position_topic_, rclcpp::QoS(1).transient_local().reliable());
    current_pose_pub_ = create_publisher<r2_serial::msg::CurrentPose>(
        current_pose_topic_, 20);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 20,
        std::bind(&R2PoseReporter::odomCallback, this, std::placeholders::_1));
    localized_sub_ = create_subscription<std_msgs::msg::Bool>(
        localized_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (mode_ == Mode::kLocalization) {
            localization_confirmed_.store(msg->data);
          }
        });
    status_srv_ = create_service<std_srvs::srv::Trigger>(
        status_service_name_,
        std::bind(&R2PoseReporter::handleStatusService, this,
                  std::placeholders::_1, std::placeholders::_2));
    relocalization_client_ = create_client<std_srvs::srv::Trigger>(
        relocalization_service_name_);

    if (mode_ == Mode::kOdometry) {
      localization_confirmed_.store(true);
    }

    status_line_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(status_line_interval_seconds_)),
        std::bind(&R2PoseReporter::printStatusLine, this));

    printStartupSummary();
  }

private:

  static std::uint64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  static std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
  }

  static const char *modeName(Mode mode) {
    return mode == Mode::kLocalization ? "localization" : "odometry";
  }

  static const char *zoneName(Zone zone) {
    return zone == Zone::kBlue ? "Blue" : "Red";
  }

  static double normalizeDeg(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
  }

  static double distance2d(double x1, double y1, double x2, double y2) {
    return std::hypot(x1 - x2, y1 - y2);
  }

  void readParameters() {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    downlink_packet_topic_ = declare_parameter<std::string>(
        "downlink_packet_topic", "/r2_serial/downlink/packet");
    initial_position_topic_ = declare_parameter<std::string>(
        "initial_position_topic", "/r2/initial_position");
    current_pose_topic_ = declare_parameter<std::string>(
        "current_pose_topic", "/r2/current_pose_mm");
    localized_topic_ = declare_parameter<std::string>(
        "localized_topic", "/r2/localized");
    status_service_name_ = declare_parameter<std::string>(
        "status_service_name", "/r2_pose_reporter/report_status");
    relocalization_service_name_ = declare_parameter<std::string>(
        "relocalization_service_name", "/r2/trigger_relocalization");

    const auto mode = lower(declare_parameter<std::string>("mode", "odometry"));
    if (mode == "localization") {
      mode_ = Mode::kLocalization;
    } else if (mode == "odometry") {
      mode_ = Mode::kOdometry;
    } else {
      throw std::invalid_argument("mode 必须是 localization 或 odometry；fallback 已移除");
    }

    const auto zone = lower(declare_parameter<std::string>("zone", "blue"));
    if (zone == "blue") {
      zone_ = Zone::kBlue;
    } else if (zone == "red") {
      zone_ = Zone::kRed;
    } else {
      throw std::invalid_argument("zone 必须显式指定为 blue 或 red");
    }

    initial_position_stabilization_seconds_ = declare_parameter<double>(
        "initial_position.stabilization_seconds", 2.0);
    initial_position_sample_seconds_ = declare_parameter<double>(
        "initial_position.sample_seconds", 5.0);
    base_offset_x_ = declare_parameter<double>("base_offset.x", 0.0847);
    base_offset_y_ = declare_parameter<double>("base_offset.y", -0.2183);

    stationary_guard_enabled_ = declare_parameter<bool>(
        "stationary_guard.enabled", true);
    stationary_guard_lock_seconds_ = declare_parameter<double>(
        "stationary_guard.lock_seconds", 2.0);
    stationary_guard_static_speed_mm_s_ = declare_parameter<double>(
        "stationary_guard.static_speed_mm_s", 35.0);
    stationary_guard_motion_speed_mm_s_ = declare_parameter<double>(
        "stationary_guard.motion_speed_mm_s", 120.0);
    stationary_guard_lock_radius_mm_ = declare_parameter<double>(
        "stationary_guard.lock_radius_mm", 45.0);
    stationary_guard_yaw_threshold_deg_ = declare_parameter<double>(
        "stationary_guard.yaw_threshold_deg", 2.0);
    stationary_guard_max_hold_drift_mm_ = declare_parameter<double>(
        "stationary_guard.max_hold_drift_mm", 1800.0);
    status_line_interval_seconds_ = declare_parameter<double>(
        "status_line.interval_seconds", 0.2);

    const auto deprecated_serial_port = declare_parameter<std::string>("serial_port", "");
    (void)declare_parameter<bool>("serial_debug_raw", false);
    if (!deprecated_serial_port.empty()) {
      RCLCPP_WARN(get_logger(),
                  "r2_pose_reporter 不再直接打开串口，serial_port=%s 将被忽略",
                  deprecated_serial_port.c_str());
    }
  }

  void validateParameters() {
    initial_position_stabilization_seconds_ =
        std::max(0.0, initial_position_stabilization_seconds_);
    initial_position_sample_seconds_ =
        std::max(0.5, initial_position_sample_seconds_);
    if (!std::isfinite(base_offset_x_) || !std::isfinite(base_offset_y_)) {
      throw std::invalid_argument("二维车体外参必须是有限数值");
    }
    stationary_guard_lock_seconds_ =
        std::max(0.2, stationary_guard_lock_seconds_);
    stationary_guard_static_speed_mm_s_ =
        std::max(1.0, stationary_guard_static_speed_mm_s_);
    stationary_guard_motion_speed_mm_s_ =
        std::max(stationary_guard_static_speed_mm_s_ + 1.0,
                 stationary_guard_motion_speed_mm_s_);
    stationary_guard_lock_radius_mm_ =
        std::max(1.0, stationary_guard_lock_radius_mm_);
    stationary_guard_yaw_threshold_deg_ =
        std::max(0.1, stationary_guard_yaw_threshold_deg_);
    stationary_guard_max_hold_drift_mm_ =
        std::max(stationary_guard_lock_radius_mm_,
                 stationary_guard_max_hold_drift_mm_);
    status_line_interval_seconds_ =
        std::max(0.05, status_line_interval_seconds_);
  }

  std::optional<std::int16_t> checkedInt16(double value,
                                           const char *field) const {
    const double rounded = std::round(value);
    if (!std::isfinite(rounded) ||
        rounded < std::numeric_limits<std::int16_t>::min() ||
        rounded > std::numeric_limits<std::int16_t>::max()) {
      RCLCPP_WARN(get_logger(), "%s 超出 int16_t 范围: %.3f", field, value);
      return std::nullopt;
    }
    return static_cast<std::int16_t>(rounded);
  }

  static std::vector<std::uint8_t> positionPayload(std::int16_t x_mm,
                                                   std::int16_t y_mm,
                                                   std::int16_t yaw_deg) {
    std::vector<std::uint8_t> payload;
    payload.reserve(6);
    protocol::appendInt16Le(payload, x_mm);
    protocol::appendInt16Le(payload, y_mm);
    protocol::appendInt16Le(payload, yaw_deg);
    return payload;
  }

  void processInitialPosition(double x_mm, double y_mm) {
    if (initial_position_published_.load()) {
      return;
    }

    const auto now = nowMs();
    if (initial_position_first_pose_ms_ == 0) {
      initial_position_first_pose_ms_ = now;
      RCLCPP_INFO(
          get_logger(),
          "已收到首帧有效位姿，等待 %.1f 秒稳定后开始起点坐标平均",
          initial_position_stabilization_seconds_);
      return;
    }

    const auto stabilization_ms = static_cast<std::uint64_t>(
        std::lround(initial_position_stabilization_seconds_ * 1000.0));
    if (now - initial_position_first_pose_ms_ < stabilization_ms) {
      return;
    }

    if (initial_position_sample_start_ms_ == 0) {
      initial_position_sample_start_ms_ = now;
      initial_position_sum_x_mm_ = 0.0;
      initial_position_sum_y_mm_ = 0.0;
      initial_position_sample_count_ = 0;
      RCLCPP_INFO(get_logger(), "开始 %.1f 秒起点坐标平均",
                  initial_position_sample_seconds_);
    }

    initial_position_sum_x_mm_ += x_mm;
    initial_position_sum_y_mm_ += y_mm;
    ++initial_position_sample_count_;

    const auto sample_ms = static_cast<std::uint64_t>(
        std::lround(initial_position_sample_seconds_ * 1000.0));
    if (now - initial_position_sample_start_ms_ < sample_ms ||
        initial_position_sample_count_ < 10) {
      return;
    }

    const double mean_x_mm =
        initial_position_sum_x_mm_ /
        static_cast<double>(initial_position_sample_count_);
    const double mean_y_mm =
        initial_position_sum_y_mm_ /
        static_cast<double>(initial_position_sample_count_);

    const auto mean_x = checkedInt16(mean_x_mm, "起点位置 X");
    const auto mean_y = checkedInt16(mean_y_mm, "起点位置 Y");
    if (!mean_x || !mean_y) {
      return;
    }

    r2_serial::msg::InitialPosition msg;
    msg.x_mm = *mean_x;
    msg.y_mm = *mean_y;
    initial_position_pub_->publish(msg);

    initial_position_x_mm_.store(*mean_x);
    initial_position_y_mm_.store(*mean_y);
    initial_position_published_.store(true);
    RCLCPP_INFO(
        get_logger(),
        "起点坐标已发布到 %s: x=%d mm y=%d mm samples=%zu",
        initial_position_topic_.c_str(), static_cast<int>(*mean_x),
        static_cast<int>(*mean_y), initial_position_sample_count_);
  }

  void triggerRelocalization(const char *command) {
    localization_confirmed_.store(false);
    if (!relocalization_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(),
                   "%s 失败：服务 %s 不可用，保持停止 0x0101 下发",
                   command, relocalization_service_name_.c_str());
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    relocalization_client_->async_send_request(
        request,
        [this, command](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          try {
            const auto response = future.get();
            if (response->success) {
              RCLCPP_WARN(get_logger(), "%s 已触发全局重定位: %s", command,
                          response->message.c_str());
            } else {
              RCLCPP_ERROR(get_logger(), "%s 重定位请求被拒绝: %s", command,
                           response->message.c_str());
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "%s 重定位服务异常: %s", command, e.what());
          }
        });
  }

  void handleRecoveryCommand(bool retry_area) {
    const char *command = retry_area ? "r2" : "r1";
    if (mode_ == Mode::kLocalization) {
      triggerRelocalization(command);
      return;
    }
    RCLCPP_WARN(get_logger(),
                "%s 在 odometry 模式下不会修改坐标；纯里程计仅输出原始相对位姿",
                command);
  }

  void printStatusLine() const {
    const auto now = nowMs();
    const auto last_pose = last_pose_time_ms_.load();
    const auto last_send = last_publish_time_ms_.load();
    const bool have_pose = last_pose != 0;
    const bool have_send = last_send != 0;
    const bool downlink_connected =
        downlink_packet_pub_ && downlink_packet_pub_->get_subscription_count() > 0;
    const char *guard_state = "off";
    if (stationary_guard_enabled_) {
      guard_state = stationary_guard_locked_atomic_.load() ? "LOCK" : "watch";
    }

    std::printf(
        "\rPOSE mode=%s zone=%s raw=(%d,%d,%d) yaw=%d out=(%d,%d) yaw=%d "
        "guard=%s serial=%s pose_age=%s send_age=%s pub=%lu drop=%lu        ",
        modeName(mode_), zoneName(zone_), current_x_.load(), current_y_.load(),
        current_z_.load(), current_yaw_deg_.load(), output_x_.load(),
        output_y_.load(), output_yaw_deg_.load(), guard_state,
        downlink_connected ? "Y" : "N",
        have_pose ? std::to_string(now - last_pose).c_str() : "NA",
        have_send ? std::to_string(now - last_send).c_str() : "NA",
        static_cast<unsigned long>(publish_success_count_.load()),
        static_cast<unsigned long>(publish_failure_count_.load()));
    std::fflush(stdout);
  }

  std::string buildStatusText() const {
    const auto last_send = last_publish_time_ms_.load();
    const bool have_publish = last_send != 0;
    const std::uint64_t elapsed = have_publish ? nowMs() - last_send : 0;
    const bool downlink_connected =
        downlink_packet_pub_ && downlink_packet_pub_->get_subscription_count() > 0;

    std::ostringstream out;
    out << "\n================ [R2 位姿上报状态] ================\n";
    out << "运行模式     : " << modeName(mode_) << "\n";
    out << "当前锁定半区 : " << zoneName(zone_) << "\n";
    out << "车体中心坐标 : X: " << current_x_.load()
        << " mm | Y: " << current_y_.load()
        << " mm | Z: " << current_z_.load()
        << " mm | Yaw: " << current_yaw_deg_.load() << " deg\n";
    if (mode_ == Mode::kLocalization) {
      out << "定位有效状态 : "
          << (localization_confirmed_.load() ? "有效" : "无效/等待重定位") << "\n";
    } else {
      out << "定位有效状态 : 纯里程计直通（不适用）\n";
    }
    if (initial_position_published_.load()) {
      out << "启动平均坐标 : X: " << initial_position_x_mm_.load()
          << " mm | Y: " << initial_position_y_mm_.load()
          << " mm | 已发布\n";
    } else {
      out << "启动平均坐标 : 等待 2 秒稳定 + 5 秒采样\n";
    }
    out << "实际下发位姿 : X: " << output_x_.load()
        << " mm | Y: " << output_y_.load()
        << " mm | Yaw: " << output_yaw_deg_.load() << " deg\n";
    if (stationary_guard_enabled_) {
      out << "静止漂移保护 : "
          << (stationary_guard_locked_atomic_.load() ? "已锁定" : "监测中")
          << " | anchor=(" << stationary_guard_anchor_x_atomic_.load()
          << ", " << stationary_guard_anchor_y_atomic_.load()
          << ") yaw=" << stationary_guard_anchor_yaw_atomic_.load()
          << " | hold=" << stationary_guard_hold_count_.load() << "\n";
    } else {
      out << "静止漂移保护 : 关闭\n";
    }
    out << "r2_serial连接: " << (downlink_connected ? "已发现订阅者" : "未发现订阅者") << "\n";
    if (have_publish) {
      out << "ROS发布状态  : 距上次发布 " << elapsed << " ms"
          << " | 已发布=" << publish_success_count_.load()
          << " 未发布=" << publish_failure_count_.load() << "\n";
    } else {
      out << "ROS发布状态  : 尚未发布 0x0101\n";
    }
    out << "终端输出     : 持续单行显示；完整状态可调用 /r2_pose_reporter/report_status\n";
    out << "==================================================";
    return out.str();
  }

  void handleStatusService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = true;
    response->message = buildStatusText();
  }

  void applyStationaryGuard(double raw_x_mm, double raw_y_mm, double raw_yaw_deg,
                            double &out_x_mm, double &out_y_mm, double &out_yaw_deg) {
    out_x_mm = raw_x_mm;
    out_y_mm = raw_y_mm;
    out_yaw_deg = raw_yaw_deg;

    if (!stationary_guard_enabled_ || mode_ != Mode::kOdometry) {
      return;
    }

    const auto now = nowMs();
    if (!stationary_guard_have_last_) {
      stationary_guard_have_last_ = true;
      stationary_guard_last_time_ms_ = now;
      stationary_guard_last_x_mm_ = raw_x_mm;
      stationary_guard_last_y_mm_ = raw_y_mm;
      stationary_guard_last_yaw_deg_ = raw_yaw_deg;
      stationary_guard_candidate_start_ms_ = now;
      stationary_guard_candidate_x_mm_ = raw_x_mm;
      stationary_guard_candidate_y_mm_ = raw_y_mm;
      stationary_guard_candidate_yaw_deg_ = raw_yaw_deg;
      return;
    }

    const double dt = std::max(1.0, static_cast<double>(now - stationary_guard_last_time_ms_)) / 1000.0;
    const double step_mm = distance2d(raw_x_mm, raw_y_mm,
                                      stationary_guard_last_x_mm_,
                                      stationary_guard_last_y_mm_);
    const double speed_mm_s = step_mm / dt;
    const double yaw_step_deg = std::abs(
        normalizeDeg(raw_yaw_deg - stationary_guard_last_yaw_deg_));

    stationary_guard_last_time_ms_ = now;
    stationary_guard_last_x_mm_ = raw_x_mm;
    stationary_guard_last_y_mm_ = raw_y_mm;
    stationary_guard_last_yaw_deg_ = raw_yaw_deg;

    if (stationary_guard_locked_) {
      const double drift_mm = distance2d(raw_x_mm, raw_y_mm,
                                         stationary_guard_anchor_x_mm_,
                                         stationary_guard_anchor_y_mm_);
      const double yaw_from_anchor_deg = std::abs(
          normalizeDeg(raw_yaw_deg - stationary_guard_anchor_yaw_deg_));
      const bool looks_static =
          speed_mm_s <= stationary_guard_static_speed_mm_s_ &&
          yaw_step_deg <= stationary_guard_yaw_threshold_deg_ &&
          yaw_from_anchor_deg <= stationary_guard_yaw_threshold_deg_ &&
          drift_mm <= stationary_guard_max_hold_drift_mm_;
      const bool real_motion =
          speed_mm_s >= stationary_guard_motion_speed_mm_s_ ||
          yaw_from_anchor_deg > stationary_guard_yaw_threshold_deg_ ||
          drift_mm > stationary_guard_max_hold_drift_mm_;

      if (looks_static && !real_motion) {
        out_x_mm = stationary_guard_anchor_x_mm_;
        out_y_mm = stationary_guard_anchor_y_mm_;
        out_yaw_deg = stationary_guard_anchor_yaw_deg_;
        stationary_guard_hold_count_.fetch_add(1);
        return;
      }

      stationary_guard_locked_ = false;
      stationary_guard_locked_atomic_.store(false);
      stationary_guard_candidate_start_ms_ = now;
      stationary_guard_candidate_x_mm_ = raw_x_mm;
      stationary_guard_candidate_y_mm_ = raw_y_mm;
      stationary_guard_candidate_yaw_deg_ = raw_yaw_deg;
      RCLCPP_INFO(get_logger(),
                  "静止漂移保护已释放: speed=%.1f mm/s drift=%.1f mm yaw_delta=%.2f deg",
                  speed_mm_s, drift_mm, yaw_from_anchor_deg);
      return;
    }

    const double candidate_dist_mm = distance2d(
        raw_x_mm, raw_y_mm, stationary_guard_candidate_x_mm_,
        stationary_guard_candidate_y_mm_);
    const double candidate_yaw_deg = std::abs(
        normalizeDeg(raw_yaw_deg - stationary_guard_candidate_yaw_deg_));
    const bool stable =
        speed_mm_s <= stationary_guard_static_speed_mm_s_ &&
        candidate_dist_mm <= stationary_guard_lock_radius_mm_ &&
        candidate_yaw_deg <= stationary_guard_yaw_threshold_deg_;

    if (!stable) {
      stationary_guard_candidate_start_ms_ = now;
      stationary_guard_candidate_x_mm_ = raw_x_mm;
      stationary_guard_candidate_y_mm_ = raw_y_mm;
      stationary_guard_candidate_yaw_deg_ = raw_yaw_deg;
      return;
    }

    const auto lock_ms = static_cast<std::uint64_t>(
        std::lround(stationary_guard_lock_seconds_ * 1000.0));
    if (now - stationary_guard_candidate_start_ms_ >= lock_ms) {
      stationary_guard_locked_ = true;
      stationary_guard_locked_atomic_.store(true);
      stationary_guard_anchor_x_mm_ = stationary_guard_candidate_x_mm_;
      stationary_guard_anchor_y_mm_ = stationary_guard_candidate_y_mm_;
      stationary_guard_anchor_yaw_deg_ = stationary_guard_candidate_yaw_deg_;
      stationary_guard_anchor_x_atomic_.store(
          static_cast<std::int16_t>(std::lround(stationary_guard_anchor_x_mm_)));
      stationary_guard_anchor_y_atomic_.store(
          static_cast<std::int16_t>(std::lround(stationary_guard_anchor_y_mm_)));
      stationary_guard_anchor_yaw_atomic_.store(
          static_cast<std::int16_t>(std::lround(stationary_guard_anchor_yaw_deg_)));
      RCLCPP_INFO(get_logger(),
                  "静止漂移保护已锁定: anchor=(%.0f, %.0f) yaw=%.1f deg",
                  stationary_guard_anchor_x_mm_, stationary_guard_anchor_y_mm_,
                  stationary_guard_anchor_yaw_deg_);
      out_x_mm = stationary_guard_anchor_x_mm_;
      out_y_mm = stationary_guard_anchor_y_mm_;
      out_yaw_deg = stationary_guard_anchor_yaw_deg_;
      stationary_guard_hold_count_.fetch_add(1);
    }
  }

  bool publishPosition(std::int16_t x_mm, std::int16_t y_mm,
                       std::int16_t yaw_deg) {
    if (!downlink_packet_pub_ ||
        downlink_packet_pub_->get_subscription_count() == 0) {
      publish_failure_count_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "未发现 r2_serial 订阅者，暂不发布 0x0101");
      return false;
    }
    r2_serial::msg::SerialPacket msg;
    msg.code = protocol::kPoseUpdate;
    msg.payload = positionPayload(x_mm, y_mm, yaw_deg);
    msg.clear_pending = false;
    downlink_packet_pub_->publish(msg);
    last_publish_time_ms_.store(nowMs());
    publish_success_count_.fetch_add(1);
    return true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double lidar_z = msg->pose.pose.position.z;
    const double lidar_x = msg->pose.pose.position.x;
    const double lidar_y = msg->pose.pose.position.y;

    const tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    const double yaw = tf2::getYaw(q);

    const double raw_x_mm =
        (lidar_x - (base_offset_x_ * std::cos(yaw) -
                    base_offset_y_ * std::sin(yaw))) * 1000.0;
    const double raw_y_mm =
        (lidar_y - (base_offset_x_ * std::sin(yaw) +
                    base_offset_y_ * std::cos(yaw))) * 1000.0;
    const auto x_mm = checkedInt16(raw_x_mm, "位置 X");
    const auto y_mm = checkedInt16(raw_y_mm, "位置 Y");
    const auto yaw_deg = checkedInt16(yaw * 180.0 / M_PI, "位置 Yaw");
    if (!x_mm || !y_mm || !yaw_deg) {
      return;
    }

    current_x_.store(*x_mm);
    current_y_.store(*y_mm);
    const auto z_mm = checkedInt16(lidar_z * 1000.0, "位置 Z");
    if (z_mm) {
      current_z_.store(*z_mm);
    }
    current_yaw_deg_.store(*yaw_deg);
    last_pose_time_ms_.store(nowMs());

    if (mode_ == Mode::kLocalization && !localization_confirmed_.load()) {
      return;
    }

    processInitialPosition(raw_x_mm, raw_y_mm);

    // 半区仅用于状态通知；位姿只做固定的雷达到车体中心二维外参换算。
    // odometry 模式下可选启用静止漂移保护，抑制原地慢漂；检测到真实移动会自动释放。
    double output_x_mm = raw_x_mm;
    double output_y_mm = raw_y_mm;
    double output_yaw = yaw * 180.0 / M_PI;
    applyStationaryGuard(raw_x_mm, raw_y_mm, output_yaw,
                         output_x_mm, output_y_mm, output_yaw);

    const auto serial_x = checkedInt16(output_x_mm, "下发位置 X");
    const auto serial_y = checkedInt16(output_y_mm, "下发位置 Y");
    const auto serial_yaw = checkedInt16(output_yaw, "下发位置 Yaw");
    if (!serial_x || !serial_y || !serial_yaw) {
      return;
    }
    output_x_.store(*serial_x);
    output_y_.store(*serial_y);
    output_yaw_deg_.store(*serial_yaw);

    r2_serial::msg::CurrentPose pose_msg;
    pose_msg.x_mm = *serial_x;
    pose_msg.y_mm = *serial_y;
    pose_msg.yaw_deg = *serial_yaw;
    current_pose_pub_->publish(pose_msg);

    publishPosition(*serial_x, *serial_y, *serial_yaw);
  }

  void printStartupSummary() {
    RCLCPP_INFO(get_logger(),
                "R2 pose reporter: mode=%s zone=%s odom=%s",
                modeName(mode_), zoneName(zone_), odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "二维车体外参: x=%.4f m y=%.4f m",
                base_offset_x_, base_offset_y_);
    RCLCPP_INFO(get_logger(),
                "静止漂移保护: %s, lock=%.1fs static<=%.1f mm/s release>=%.1f mm/s",
                stationary_guard_enabled_ ? "开启" : "关闭",
                stationary_guard_lock_seconds_, stationary_guard_static_speed_mm_s_,
                stationary_guard_motion_speed_mm_s_);
    RCLCPP_INFO(get_logger(),
                "odometry 模式不执行启动锚点、runtime_offset 或坐标平移");
    RCLCPP_INFO(
        get_logger(),
        "起点坐标: 首帧有效位姿后等待 %.1f 秒，再平均 %.1f 秒，发布到 %s",
        initial_position_stabilization_seconds_,
        initial_position_sample_seconds_, initial_position_topic_.c_str());
    std::printf("\n============================================================\n");
    std::printf(" R2 位姿上报节点已启动：持续单行输出位姿状态\n");
    std::printf(" 完整状态服务: ros2 service call /r2_pose_reporter/report_status std_srvs/srv/Trigger {}\n");
    std::printf("============================================================\n");
  }

  Mode mode_{Mode::kOdometry};
  Zone zone_{Zone::kBlue};

  std::string odom_topic_;
  std::string downlink_packet_topic_;
  std::string initial_position_topic_;
  std::string current_pose_topic_;
  std::string localized_topic_;
  std::string status_service_name_;
  std::string relocalization_service_name_;

  double base_offset_x_{0.1352};
  double base_offset_y_{-0.2335};
  double initial_position_stabilization_seconds_{2.0};
  double initial_position_sample_seconds_{5.0};

  bool stationary_guard_enabled_{true};
  double stationary_guard_lock_seconds_{2.0};
  double stationary_guard_static_speed_mm_s_{35.0};
  double stationary_guard_motion_speed_mm_s_{120.0};
  double stationary_guard_lock_radius_mm_{45.0};
  double stationary_guard_yaw_threshold_deg_{2.0};
  double stationary_guard_max_hold_drift_mm_{1800.0};
  double status_line_interval_seconds_{0.2};
  bool stationary_guard_have_last_{false};
  bool stationary_guard_locked_{false};
  std::uint64_t stationary_guard_last_time_ms_{0};
  double stationary_guard_last_x_mm_{0.0};
  double stationary_guard_last_y_mm_{0.0};
  double stationary_guard_last_yaw_deg_{0.0};
  std::uint64_t stationary_guard_candidate_start_ms_{0};
  double stationary_guard_candidate_x_mm_{0.0};
  double stationary_guard_candidate_y_mm_{0.0};
  double stationary_guard_candidate_yaw_deg_{0.0};
  double stationary_guard_anchor_x_mm_{0.0};
  double stationary_guard_anchor_y_mm_{0.0};
  double stationary_guard_anchor_yaw_deg_{0.0};

  std::atomic<bool> localization_confirmed_{false};
  std::atomic<bool> initial_position_published_{false};
  std::uint64_t initial_position_first_pose_ms_{0};
  std::uint64_t initial_position_sample_start_ms_{0};
  std::size_t initial_position_sample_count_{0};
  double initial_position_sum_x_mm_{0.0};
  double initial_position_sum_y_mm_{0.0};
  std::atomic<std::int16_t> initial_position_x_mm_{0};
  std::atomic<std::int16_t> initial_position_y_mm_{0};
  std::atomic<bool> stationary_guard_locked_atomic_{false};
  std::atomic<std::int16_t> stationary_guard_anchor_x_atomic_{0};
  std::atomic<std::int16_t> stationary_guard_anchor_y_atomic_{0};
  std::atomic<std::int16_t> stationary_guard_anchor_yaw_atomic_{0};
  std::atomic<std::uint64_t> stationary_guard_hold_count_{0};
  std::atomic<std::uint64_t> last_pose_time_ms_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localized_sub_;
  rclcpp::Publisher<r2_serial::msg::SerialPacket>::SharedPtr downlink_packet_pub_;
  rclcpp::Publisher<r2_serial::msg::InitialPosition>::SharedPtr
      initial_position_pub_;
  rclcpp::Publisher<r2_serial::msg::CurrentPose>::SharedPtr current_pose_pub_;
  rclcpp::TimerBase::SharedPtr status_line_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr relocalization_client_;

  std::atomic<std::int16_t> current_x_{0};
  std::atomic<std::int16_t> current_y_{0};
  std::atomic<std::int16_t> current_z_{0};
  std::atomic<std::int16_t> current_yaw_deg_{0};
  std::atomic<std::int16_t> output_x_{0};
  std::atomic<std::int16_t> output_y_{0};
  std::atomic<std::int16_t> output_yaw_deg_{0};
  std::atomic<std::uint64_t> last_publish_time_ms_{0};
  std::atomic<std::uint64_t> publish_success_count_{0};
  std::atomic<std::uint64_t> publish_failure_count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<R2PoseReporter>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("r2_pose_reporter"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
