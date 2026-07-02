#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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
#include <thread>
#include <vector>

#include "r2_serial/msg/serial_packet.hpp"
#include "r2_serial/serial_protocol.hpp"

namespace protocol = r2_serial::protocol;

class R2PoseReporter : public rclcpp::Node {
public:
  enum class Zone : std::uint8_t {
    kUnlocked,
    kBlue,
    kRed,
  };

  R2PoseReporter() : Node("r2_pose_reporter") {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    downlink_packet_topic_ = declare_parameter<std::string>(
        "downlink_packet_topic", "/r2_serial/downlink/packet");
    status_service_name_ = declare_parameter<std::string>(
        "status_service_name", "/r2_pose_reporter/report_status");
    localized_topic_ = declare_parameter<std::string>(
        "localized_topic", "/r2/localized");

    // 兼容旧启动参数：simple_odom 现在不再直接打开串口，串口统一交给 r2_serial 节点。
    const auto deprecated_serial_port = declare_parameter<std::string>("serial_port", "");
    (void)declare_parameter<bool>("serial_debug_raw", false);
    if (!deprecated_serial_port.empty()) {
      RCLCPP_WARN(get_logger(),
                  "r2_pose_reporter 不再直接打开串口，serial_port=%s 将被忽略；请启动 r2_serial/r2_data_downlink。",
                  deprecated_serial_port.c_str());
    }

    height_compensation_enabled_ = declare_parameter<bool>(
        "height_compensation.enabled", true);
    height_compensation_auto_disable_for_global_ = declare_parameter<bool>(
        "height_compensation.auto_disable_for_global_odometry", true);
    height_compensation_x_per_z_ = declare_parameter<double>(
        "height_compensation.x_per_z", -0.52);
    height_compensation_y_per_z_ = declare_parameter<double>(
        "height_compensation.y_per_z", 0.0);
    height_compensation_use_initial_z_ = declare_parameter<bool>(
        "height_compensation.use_initial_z_as_reference", true);
    height_compensation_reference_z_ = declare_parameter<double>(
        "height_compensation.reference_z", 0.0);
    base_offset_x_ = declare_parameter<double>("base_offset.x", 0.1352);
    base_offset_y_ = declare_parameter<double>("base_offset.y", -0.2335);
    blue_start_point_ = declare_parameter<std::vector<double>>(
        "blue_start_point", std::vector<double>{-310.0, -115.0});
    red_start_point_ = declare_parameter<std::vector<double>>(
        "red_start_point", std::vector<double>{-310.0, -2950.0});
    mirror_center_y_ = declare_parameter<double>("mirror_center_y", -1532.5);
    validateZoneParameters();

    if (height_compensation_auto_disable_for_global_ && isGlobalOdomTopic(odom_topic_)) {
      height_compensation_enabled_ = false;
      height_compensation_disabled_for_global_ = true;
    }

    downlink_packet_pub_ = create_publisher<r2_serial::msg::SerialPacket>(
        downlink_packet_topic_, 50);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&R2PoseReporter::odomCallback, this, std::placeholders::_1));
    localized_sub_ = create_subscription<std_msgs::msg::Bool>(
        localized_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            localization_confirmed_.store(true);
          }
        });
    status_srv_ = create_service<std_srvs::srv::Trigger>(
        status_service_name_,
        std::bind(&R2PoseReporter::handleStatusService, this,
                  std::placeholders::_1, std::placeholders::_2));

    keyboard_thread_ = std::thread(&R2PoseReporter::keyboardLoop, this);
    keyboard_thread_.detach();

    RCLCPP_INFO(get_logger(), "订阅里程计: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "位姿上报发布到: %s", downlink_packet_topic_.c_str());
    RCLCPP_INFO(get_logger(), "状态查询服务: %s", status_service_name_.c_str());
    RCLCPP_INFO(get_logger(), "定位有效状态订阅自: %s", localized_topic_.c_str());
    RCLCPP_INFO(get_logger(), "车体中心外参: x=%.4f m y=%.4f m", base_offset_x_, base_offset_y_);
    RCLCPP_INFO(get_logger(),
                "半区侦测参数(mm): BLUE=[%.1f, %.1f] RED=[%.1f, %.1f] mirror_y=%.1f",
                blue_start_point_[0], blue_start_point_[1],
                red_start_point_[0], red_start_point_[1], mirror_center_y_);
    if (height_compensation_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "升降高度补偿已启用: x_per_z=%.6f y_per_z=%.6f reference=%s%.3f",
                  height_compensation_x_per_z_, height_compensation_y_per_z_,
                  height_compensation_use_initial_z_ ? "initial_z=" : "fixed_z=",
                  height_compensation_reference_z_);
    } else if (height_compensation_disabled_for_global_) {
      RCLCPP_INFO(get_logger(),
                  "订阅全局定位 %s，已自动关闭升降高度补偿，避免 field/global 坐标被二次修正。",
                  odom_topic_.c_str());
    }

    std::printf("|===================== R2 位姿上报节点启动 =====================|\n");
    // std::printf("ros2 service call /r2_pose_reporter/report_status std_srvs/srv/Trigger "{}"\n");
    // std::printf("================================================================\n> ");
  }

private:
  static constexpr std::size_t kZoneDetectionSamples = 10;

  static std::uint64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  static bool isGlobalOdomTopic(const std::string &topic) {
    return topic.find("global_odometry") != std::string::npos ||
           topic.find("r2/global") != std::string::npos;
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

  void validateZoneParameters() const {
    if (blue_start_point_.size() != 2 || red_start_point_.size() != 2) {
      throw std::invalid_argument(
          "blue_start_point 和 red_start_point 必须各包含两个数值 [x_mm, y_mm]");
    }
    if (!std::isfinite(blue_start_point_[0]) ||
        !std::isfinite(blue_start_point_[1]) ||
        !std::isfinite(red_start_point_[0]) ||
        !std::isfinite(red_start_point_[1]) ||
        !std::isfinite(mirror_center_y_)) {
      throw std::invalid_argument("半区侦测坐标参数必须是有限数值");
    }
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

  bool updateZoneDetection(double x_mm, double y_mm) {
    if (zone_.load() != Zone::kUnlocked) {
      return true;
    }

    zone_detection_sum_x_ += x_mm;
    zone_detection_sum_y_ += y_mm;
    ++zone_detection_sample_count_;
    if (zone_detection_sample_count_ < kZoneDetectionSamples) {
      return false;
    }

    const double mean_x =
        zone_detection_sum_x_ / static_cast<double>(zone_detection_sample_count_);
    const double mean_y =
        zone_detection_sum_y_ / static_cast<double>(zone_detection_sample_count_);
    const double blue_distance =
        std::hypot(mean_x - blue_start_point_[0], mean_y - blue_start_point_[1]);
    const double red_distance =
        std::hypot(mean_x - red_start_point_[0], mean_y - red_start_point_[1]);
    const Zone detected =
        blue_distance <= red_distance ? Zone::kBlue : Zone::kRed;
    zone_.store(detected);

    RCLCPP_INFO(
        get_logger(),
        "================ [Zone Detected] Locked to %s zone. "
        "mean=(%.1f, %.1f) mm, distance: BLUE=%.1f mm RED=%.1f mm ================",
        detected == Zone::kBlue ? "BLUE" : "RED", mean_x, mean_y,
        blue_distance, red_distance);
    return true;
  }

  void keyboardLoop() {
    std::string line;
    while (rclcpp::ok() && std::getline(std::cin, line)) {
      if (line.empty()) {
        continue;
      }
      const char cmd = static_cast<char>(
          std::tolower(static_cast<unsigned char>(line.front())));
      if (cmd == 'q') {
        printStatus();
      } else {
        std::printf("\n[r2_pose_reporter] 当前只支持 q：查询当前位姿与发布状态。\n> ");
      }
    }
  }

  std::string buildStatusText() const {
    const auto last_send = last_publish_time_ms_.load();
    const bool have_publish = last_send != 0;
    const std::uint64_t elapsed = have_publish ? nowMs() - last_send : 0;
    const bool downlink_connected =
        downlink_packet_pub_ && downlink_packet_pub_->get_subscription_count() > 0;
    const bool recently_published = downlink_connected && have_publish && elapsed < 1000;

    std::ostringstream out;
    out << "\n================ [R2 位姿上报状态] ================\n";
    out << "当前车体坐标 : X: " << current_x_.load()
        << " mm | Y: " << current_y_.load()
        << " mm | Z: " << current_z_.load()
        << " mm | Yaw: " << current_yaw_deg_.load() << " deg\n";
    out << "全局定位确认 : "
        << (localization_confirmed_.load() ? "已收到 /r2/localized=true" : "等待定位成功")
        << "\n";
    out << "当前锁定半区 : " << zoneName(zone_.load()) << "\n";
    out << "实际下发位姿 : X: " << output_x_.load()
        << " mm | Y: " << output_y_.load()
        << " mm | Yaw: " << output_yaw_deg_.load() << " deg\n";
    if (height_compensation_enabled_) {
      out << "升降补偿状态 : ref_z=" << height_compensation_reference_z_mm_.load()
          << " mm | dx=" << height_compensation_dx_mm_.load()
          << " mm | dy=" << height_compensation_dy_mm_.load() << " mm\n";
    }
    out << "里程计输入   : " << odom_topic_ << "\n";
    out << "下发话题     : " << downlink_packet_topic_ << "\n";
    out << "r2_serial连接: " << (downlink_connected ? "已发现订阅者" : "未发现订阅者") << "\n";
    if (have_publish) {
      out << "ROS发布状态  : " << (recently_published ? "正常发布中" : "静默或无订阅者")
          << " (距上次发布 " << elapsed << " ms)\n";
      out << "最近发布包   : code=0x" << std::hex << std::uppercase
          << last_publish_code_.load() << std::dec
          << " payload=" << last_publish_bytes_.load() << " bytes\n";
    } else {
      out << "ROS发布状态  : 尚无成功发布记录\n";
    }
    out << "发布统计     : 已发布=" << publish_success_count_.load()
        << " 未发布=" << publish_failure_count_.load() << "\n";
    out << "说明         : 这里只表示已发布给 r2_serial；串口真实写入结果请看 r2_data_downlink。\n";
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

  bool publishPosition(std::int16_t x_mm, std::int16_t y_mm, std::int16_t yaw_deg) {
    if (!downlink_packet_pub_ || downlink_packet_pub_->get_subscription_count() == 0) {
      publish_failure_count_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "未发现 r2_serial/r2_data_downlink 订阅者，暂不发布 0x0101");
      return false;
    }

    r2_serial::msg::SerialPacket msg;
    msg.code = protocol::kPoseUpdate;
    msg.payload = positionPayload(x_mm, y_mm, yaw_deg);
    msg.clear_pending = false;
    downlink_packet_pub_->publish(msg);

    last_publish_time_ms_.store(nowMs());
    last_publish_code_.store(msg.code);
    last_publish_bytes_.store(msg.payload.size());
    publish_success_count_.fetch_add(1);
    return true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
    tf2::Matrix3x3 matrix(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    matrix.getRPY(roll, pitch, yaw);

    const double base_x = lidar_x - (base_offset_x_ * std::cos(yaw) - base_offset_y_ * std::sin(yaw));
    const double base_y = lidar_y - (base_offset_x_ * std::sin(yaw) + base_offset_y_ * std::cos(yaw));
    const auto x_mm = checkedInt16(std::lround(base_x * 1000.0), "位置 X");
    const auto y_mm = checkedInt16(std::lround(base_y * 1000.0), "位置 Y");
    const auto z_mm = checkedInt16(std::lround(lidar_z * 1000.0), "位置 Z");
    const auto yaw_deg = checkedInt16(std::lround(yaw * 180.0 / M_PI), "位置 Yaw");
    if (!x_mm || !y_mm || !yaw_deg) {
      return;
    }

    current_x_.store(*x_mm);
    current_y_.store(*y_mm);
    if (z_mm) {
      current_z_.store(*z_mm);
    }
    current_yaw_deg_.store(*yaw_deg);

    // 定位成功前以及锁区前均不下发，避免把局部原点误判成蓝区。
    if (!localization_confirmed_.load() || !updateZoneDetection(*x_mm, *y_mm)) {
      return;
    }

    double output_x_mm = *x_mm;
    double output_y_mm = *y_mm;
    double output_yaw = *yaw_deg;
    if (zone_.load() == Zone::kRed) {
      output_y_mm = 2.0 * mirror_center_y_ - output_y_mm;
      output_yaw = -output_yaw;
    }

    const auto mapped_x = checkedInt16(output_x_mm, "映射位置 X");
    const auto mapped_y = checkedInt16(output_y_mm, "映射位置 Y");
    const auto mapped_yaw = checkedInt16(output_yaw, "映射位置 Yaw");
    if (!mapped_x || !mapped_y || !mapped_yaw) {
      return;
    }

    output_x_.store(*mapped_x);
    output_y_.store(*mapped_y);
    output_yaw_deg_.store(*mapped_yaw);
    publishPosition(*mapped_x, *mapped_y, *mapped_yaw);
  }

  std::thread keyboard_thread_;
  std::string odom_topic_;
  std::string downlink_packet_topic_;
  std::string status_service_name_;
  std::string localized_topic_;

  bool height_compensation_enabled_{true};
  bool height_compensation_auto_disable_for_global_{true};
  bool height_compensation_disabled_for_global_{false};
  bool height_compensation_use_initial_z_{true};
  double height_compensation_x_per_z_{-0.52};
  double height_compensation_y_per_z_{0.0};
  double height_compensation_reference_z_{0.0};
  double base_offset_x_{0.1352};
  double base_offset_y_{-0.2335};
  std::vector<double> blue_start_point_;
  std::vector<double> red_start_point_;
  double mirror_center_y_{-1532.5};
  std::atomic<Zone> zone_{Zone::kUnlocked};
  double zone_detection_sum_x_{0.0};
  double zone_detection_sum_y_{0.0};
  std::size_t zone_detection_sample_count_{0};
  std::atomic<bool> localization_confirmed_{false};
  std::atomic<bool> height_reference_initialized_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localized_sub_;
  rclcpp::Publisher<r2_serial::msg::SerialPacket>::SharedPtr downlink_packet_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;

  std::atomic<std::int16_t> current_x_{0};
  std::atomic<std::int16_t> current_y_{0};
  std::atomic<std::int16_t> current_z_{0};
  std::atomic<std::int16_t> current_yaw_deg_{0};
  std::atomic<std::int16_t> output_x_{0};
  std::atomic<std::int16_t> output_y_{0};
  std::atomic<std::int16_t> output_yaw_deg_{0};
  std::atomic<std::int16_t> height_compensation_reference_z_mm_{0};
  std::atomic<std::int16_t> height_compensation_dx_mm_{0};
  std::atomic<std::int16_t> height_compensation_dy_mm_{0};

  std::atomic<std::uint64_t> last_publish_time_ms_{0};
  std::atomic<std::uint64_t> publish_success_count_{0};
  std::atomic<std::uint64_t> publish_failure_count_{0};
  std::atomic<std::uint16_t> last_publish_code_{0};
  std::atomic<std::size_t> last_publish_bytes_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R2PoseReporter>());
  rclcpp::shutdown();
  return 0;
}
