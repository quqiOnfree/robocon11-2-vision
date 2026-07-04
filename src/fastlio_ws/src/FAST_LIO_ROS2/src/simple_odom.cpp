#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
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
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "r2_serial/msg/serial_packet.hpp"
#include "r2_serial/serial_protocol.hpp"

namespace protocol = r2_serial::protocol;

class R2PoseReporter : public rclcpp::Node {
public:
  enum class Zone : std::int8_t { kUnlocked = -1, kBlue = 0, kRed = 1 };
  enum class Mode : std::uint8_t { kLocalization, kOdometry };

  R2PoseReporter() : Node("r2_pose_reporter") {
    readParameters();
    validateParameters();

    downlink_packet_pub_ = create_publisher<r2_serial::msg::SerialPacket>(
        downlink_packet_topic_, 50);
    match_zone_pub_ = create_publisher<std_msgs::msg::Int8>(
        match_zone_topic_, rclcpp::QoS(1).transient_local().reliable());
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

    if (configured_zone_ != Zone::kUnlocked) {
      lockZone(configured_zone_, "启动参数");
    }

    if (mode_ == Mode::kOdometry) {
      localization_confirmed_.store(true);
    }

    keyboard_thread_ = std::thread(&R2PoseReporter::keyboardLoop, this);
    keyboard_thread_.detach();

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

  static const char *zoneName(Zone zone) {
    switch (zone) {
    case Zone::kBlue:
      return "Blue";
    case Zone::kRed:
      return "Red";
    default:
      return "Unlocked";
    }
  }

  static const char *modeName(Mode mode) {
    return mode == Mode::kLocalization ? "localization" : "odometry";
  }

  void readParameters() {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    downlink_packet_topic_ = declare_parameter<std::string>(
        "downlink_packet_topic", "/r2_serial/downlink/packet");
    match_zone_topic_ = declare_parameter<std::string>(
        "match_zone_topic", "/r2/match_zone");
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
      configured_zone_ = Zone::kBlue;
    } else if (zone == "red") {
      configured_zone_ = Zone::kRed;
    } else {
      throw std::invalid_argument("zone 必须显式指定为 blue 或 red");
    }

    pose_report_average_seconds_ = declare_parameter<double>(
        "pose_report.average_seconds", 5.0);
    base_offset_x_ = declare_parameter<double>("base_offset.x", 0.1352);
    base_offset_y_ = declare_parameter<double>("base_offset.y", -0.2335);


    const auto deprecated_serial_port = declare_parameter<std::string>("serial_port", "");
    (void)declare_parameter<bool>("serial_debug_raw", false);
    if (!deprecated_serial_port.empty()) {
      RCLCPP_WARN(get_logger(),
                  "r2_pose_reporter 不再直接打开串口，serial_port=%s 将被忽略",
                  deprecated_serial_port.c_str());
    }
  }

  void validateParameters() {
    pose_report_average_seconds_ = std::max(0.5, pose_report_average_seconds_);
    if (!std::isfinite(base_offset_x_) || !std::isfinite(base_offset_y_)) {
      throw std::invalid_argument("二维车体外参必须是有限数值");
    }
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

  void publishMatchZone(Zone zone) {
    if (zone == Zone::kUnlocked || !match_zone_pub_) {
      return;
    }
    std_msgs::msg::Int8 msg;
    msg.data = static_cast<std::int8_t>(zone);
    match_zone_pub_->publish(msg);
  }

  void lockZone(Zone zone, const char *reason) {
    zone_.store(zone);
    publishMatchZone(zone);
    RCLCPP_INFO(get_logger(),
                "================ [Zone Detected] Locked to %s zone (%s) ================",
                zone == Zone::kBlue ? "BLUE" : "RED", reason);
  }

  void beginAverage(const std::string &reason) {
    std::lock_guard<std::mutex> lock(average_mutex_);
    average_active_ = true;
    average_reason_ = reason;
    average_start_ms_ = 0;
    average_count_ = 0;
    average_sum_x_ = 0.0;
    average_sum_y_ = 0.0;
    average_sum_sin_yaw_ = 0.0;
    average_sum_cos_yaw_ = 0.0;
    RCLCPP_INFO(get_logger(), "开始 %.1f 秒静止平均: %s",
                pose_report_average_seconds_, reason.c_str());
  }

  void processAverage(double x_mm, double y_mm, double yaw_rad) {
    std::string completed_reason;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_yaw_deg = 0.0;
    bool completed = false;

    {
      std::lock_guard<std::mutex> lock(average_mutex_);
      if (!average_active_) {
        return;
      }
      const auto now = nowMs();
      if (average_start_ms_ == 0) {
        average_start_ms_ = now;
      }
      ++average_count_;
      average_sum_x_ += x_mm;
      average_sum_y_ += y_mm;
      average_sum_sin_yaw_ += std::sin(yaw_rad);
      average_sum_cos_yaw_ += std::cos(yaw_rad);

      const auto required_ms = static_cast<std::uint64_t>(
          std::lround(pose_report_average_seconds_ * 1000.0));
      if (now - average_start_ms_ < required_ms || average_count_ < 10) {
        return;
      }

      mean_x = average_sum_x_ / static_cast<double>(average_count_);
      mean_y = average_sum_y_ / static_cast<double>(average_count_);
      mean_yaw_deg = std::atan2(average_sum_sin_yaw_, average_sum_cos_yaw_) *
                     180.0 / M_PI;
      completed_reason = average_reason_;
      average_active_ = false;
      completed = true;
    }

    if (completed) {
      RCLCPP_INFO(get_logger(),
                  "静止平均完成 [%s]: x=%.1f mm y=%.1f mm yaw=%.2f deg",
                  completed_reason.c_str(), mean_x, mean_y, mean_yaw_deg);
    }
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

  void keyboardLoop() {
    std::string line;
    while (rclcpp::ok() && std::getline(std::cin, line)) {
      line = lower(line);
      line.erase(std::remove_if(line.begin(), line.end(),
                                [](unsigned char c) { return std::isspace(c); }),
                 line.end());
      if (line == "q") {
        printStatus();
        beginAverage("q 手动测量");
      } else if (line == "r1") {
        handleRecoveryCommand(false);
      } else if (line == "r2") {
        handleRecoveryCommand(true);
      } else if (!line.empty()) {
        std::printf("\n[r2_pose_reporter] 支持命令: q / r1 / r2\n> ");
      }
    }
  }

  std::string buildStatusText() const {
    const auto last_send = last_publish_time_ms_.load();
    const bool have_publish = last_send != 0;
    const std::uint64_t elapsed = have_publish ? nowMs() - last_send : 0;
    const bool downlink_connected =
        downlink_packet_pub_ && downlink_packet_pub_->get_subscription_count() > 0;

    bool averaging = false;
    std::size_t samples = 0;
    {
      std::lock_guard<std::mutex> lock(average_mutex_);
      averaging = average_active_;
      samples = average_count_;
    }

    std::ostringstream out;
    out << "\n================ [R2 位姿上报状态] ================\n";
    out << "运行模式     : " << modeName(mode_) << "\n";
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
    out << "当前锁定半区 : " << zoneName(zone_.load()) << "\n";
    out << "实际下发位姿 : X: " << output_x_.load()
        << " mm | Y: " << output_y_.load()
        << " mm | Yaw: " << output_yaw_deg_.load() << " deg\n";
    out << "静止平均     : " << (averaging ? "进行中" : "空闲")
        << " | samples=" << samples << "\n";
    out << "r2_serial连接: " << (downlink_connected ? "已发现订阅者" : "未发现订阅者") << "\n";
    if (have_publish) {
      out << "ROS发布状态  : 距上次发布 " << elapsed << " ms"
          << " | 已发布=" << publish_success_count_.load()
          << " 未发布=" << publish_failure_count_.load() << "\n";
    } else {
      out << "ROS发布状态  : 尚未发布 0x0101\n";
    }
    out << "命令         : q=状态+5秒平均；r1/r2=仅 localization 触发重定位\n";
    out << "==================================================";
    return out.str();
  }

  void printStatus() {
    const auto text = buildStatusText();
    std::printf("%s\n> ", text.c_str());
  }

  void handleStatusService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = true;
    response->message = buildStatusText();
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
    processAverage(raw_x_mm, raw_y_mm, yaw);

    if (mode_ == Mode::kLocalization && !localization_confirmed_.load()) {
      return;
    }

    // 半区仅用于状态通知；位姿只做固定的雷达到车体中心二维外参换算。
    const double output_x_mm = raw_x_mm;
    const double output_y_mm = raw_y_mm;
    const double output_yaw = yaw * 180.0 / M_PI;

    const auto serial_x = checkedInt16(output_x_mm, "下发位置 X");
    const auto serial_y = checkedInt16(output_y_mm, "下发位置 Y");
    const auto serial_yaw = checkedInt16(output_yaw, "下发位置 Yaw");
    if (!serial_x || !serial_y || !serial_yaw) {
      return;
    }
    output_x_.store(*serial_x);
    output_y_.store(*serial_y);
    output_yaw_deg_.store(*serial_yaw);
    publishPosition(*serial_x, *serial_y, *serial_yaw);
  }

  void printStartupSummary() {
    RCLCPP_INFO(get_logger(),
                "R2 pose reporter: mode=%s zone=%s odom=%s",
                modeName(mode_), zoneName(configured_zone_), odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "二维车体外参: x=%.4f m y=%.4f m",
                base_offset_x_, base_offset_y_);
    RCLCPP_INFO(get_logger(),
                "odometry 模式不执行启动锚点、runtime_offset 或坐标平移");
    std::printf("\n============================================================\n");
    std::printf(" R2 位姿上报节点已启动：q；r1/r2 仅用于 localization 重定位\n");
    std::printf("============================================================\n> ");
  }

  Mode mode_{Mode::kOdometry};
  Zone configured_zone_{Zone::kUnlocked};
  std::atomic<Zone> zone_{Zone::kUnlocked};

  std::string odom_topic_;
  std::string downlink_packet_topic_;
  std::string match_zone_topic_;
  std::string localized_topic_;
  std::string status_service_name_;
  std::string relocalization_service_name_;

  double base_offset_x_{0.1352};
  double base_offset_y_{-0.2335};
  double pose_report_average_seconds_{5.0};
  mutable std::mutex average_mutex_;
  bool average_active_{false};
  std::string average_reason_;
  std::uint64_t average_start_ms_{0};
  std::size_t average_count_{0};
  double average_sum_x_{0.0};
  double average_sum_y_{0.0};
  double average_sum_sin_yaw_{0.0};
  double average_sum_cos_yaw_{0.0};

  std::atomic<bool> localization_confirmed_{false};

  std::thread keyboard_thread_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localized_sub_;
  rclcpp::Publisher<r2_serial::msg::SerialPacket>::SharedPtr downlink_packet_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr match_zone_pub_;
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
