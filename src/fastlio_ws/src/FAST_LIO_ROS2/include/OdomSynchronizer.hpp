#pragma once

#include <cstddef>
#include <deque>
#include <mutex>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

namespace r2_localizer {

// 保存 FAST-LIO 同一时刻的局部位姿和原始里程计消息。
struct OdomSample {
  rclcpp::Time stamp;
  Eigen::Matrix4f camera_init_to_body;
  nav_msgs::msg::Odometry odom;
};

// 负责在两个异步话题之间按时间戳匹配里程计，并封装队列互斥访问。
class OdomSynchronizer {
public:
  explicit OdomSynchronizer(double tolerance_sec,
                            std::size_t max_buffer_size = 200);

  void push(OdomSample sample);
  bool findMatchingOdom(const rclcpp::Time &stamp, OdomSample &match) const;

private:
  double tolerance_sec_;
  std::size_t max_buffer_size_;
  std::deque<OdomSample> buffer_;
  mutable std::mutex mutex_;
};

}  // namespace r2_localizer
