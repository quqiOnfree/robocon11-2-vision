#include "OdomSynchronizer.hpp"

#include <cstdint>
#include <limits>
#include <utility>

namespace r2_localizer {

OdomSynchronizer::OdomSynchronizer(double tolerance_sec,
                                   std::size_t max_buffer_size)
    : tolerance_sec_(tolerance_sec), max_buffer_size_(max_buffer_size) {}

void OdomSynchronizer::push(OdomSample sample) {
  // 只保留最近一段里程计，避免长时间运行后队列无限增长。
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.push_back(std::move(sample));
  while (buffer_.size() > max_buffer_size_) {
    buffer_.pop_front();
  }
}

// 点云与里程计由 FAST-LIO 分别发布，这里选取时间差最小且在容差内的一帧。
bool OdomSynchronizer::findMatchingOdom(const rclcpp::Time &stamp,
                                        OdomSample &match) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return false;
  }
  std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();
  for (const auto &sample : buffer_) {
    const std::int64_t delta = (sample.stamp - stamp).nanoseconds();
    const std::int64_t absolute_delta = delta < 0 ? -delta : delta;
    if (absolute_delta < best_delta) {
      best_delta = absolute_delta;
      match = sample;
    }
  }
  return static_cast<double>(best_delta) * 1e-9 <= tolerance_sec_;
}

}  // namespace r2_localizer
