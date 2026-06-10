#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "MapManager.hpp"

namespace {

using Cloud = r2_localizer::Cloud;
using Point = r2_localizer::Point;
using Matrix4f = r2_localizer::Matrix4f;

struct SlotRoi {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct SlotStats {
  int point_count{0};
  int voxel_count{0};
  float score{0.0F};
  bool occupied{false};
  std::unordered_set<std::int64_t> voxels;
};

Matrix4f transformToMatrix(const geometry_msgs::msg::TransformStamped &tf) {
  Matrix4f matrix = Matrix4f::Identity();
  const auto &q_msg = tf.transform.rotation;
  Eigen::Quaternionf q(static_cast<float>(q_msg.w), static_cast<float>(q_msg.x),
                       static_cast<float>(q_msg.y), static_cast<float>(q_msg.z));
  q.normalize();
  matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  matrix(0, 3) = static_cast<float>(tf.transform.translation.x);
  matrix(1, 3) = static_cast<float>(tf.transform.translation.y);
  matrix(2, 3) = static_cast<float>(tf.transform.translation.z);
  return matrix;
}

std::int64_t voxelKey(const Point &point, double voxel_size) {
  const auto ix = static_cast<std::int64_t>(std::floor(point.x / voxel_size));
  const auto iy = static_cast<std::int64_t>(std::floor(point.y / voxel_size));
  const auto iz = static_cast<std::int64_t>(std::floor(point.z / voxel_size));
  constexpr std::int64_t kMask = (1LL << 21) - 1;
  return ((ix & kMask) << 42) | ((iy & kMask) << 21) | (iz & kMask);
}

}  // namespace

class R2KfsDetectorNode : public rclcpp::Node {
public:
  R2KfsDetectorNode()
      : Node("r2_kfs_detector"), tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    declareParameters();
    readParameters();

    map_manager_ = std::make_unique<r2_localizer::MapManager>(map_path_,
                                                               map_voxel_size_);

    slots_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
        "/r2/kfs_slots", 10);
    scores_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/r2/kfs_scores", 10);
    debug_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/r2/kfs_debug_cloud", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/r2/kfs_markers", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&R2KfsDetectorNode::cloudCallback, this,
                  std::placeholders::_1));

    if (!slots_configured_) {
      RCLCPP_WARN(get_logger(),
                  "KFS detector started with placeholder ROI coordinates. Fill config/r2_kfs_detector.yaml slots.centers before using detection.");
    }
    RCLCPP_INFO(get_logger(),
                "R2 KFS detector ready: map=%s, cloud=%s, slots=%zu, choose top %d",
                map_path_.c_str(), input_cloud_topic_.c_str(), slots_.size(),
                occupied_count_);
  }

private:
  void declareParameters() {
    declare_parameter<std::string>("map_path",
                                   "/root/fastlio_ws/MapPCD/R2_Field_Map_v1.pcd");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("input_cloud_topic", "/cloud_registered_body");
    declare_parameter<double>("map_voxel_size", 0.10);
    declare_parameter<double>("background_radius", 0.08);
    declare_parameter<double>("slot.size_xy", 0.35);
    declare_parameter<double>("slot.z_min_offset", 0.02);
    declare_parameter<double>("slot.z_max_offset", 0.55);
    declare_parameter<double>("slot.voxel_size", 0.05);
    declare_parameter<int>("slot.min_new_points", 20);
    declare_parameter<int>("slot.min_occupied_voxels", 6);
    declare_parameter<int>("occupied_count", 8);
    declare_parameter<std::vector<double>>("slots.centers", defaultSlotCenters());
  }

  static std::vector<double> defaultSlotCenters() {
    std::vector<double> centers;
    centers.reserve(kSlotCount * 3);
    for (int i = 0; i < kSlotCount; ++i) {
      centers.push_back(0.0);
      centers.push_back(0.0);
      centers.push_back(0.0);
    }
    return centers;
  }

  void readParameters() {
    get_parameter("map_path", map_path_);
    get_parameter("map_frame", map_frame_);
    get_parameter("input_cloud_topic", input_cloud_topic_);
    get_parameter("map_voxel_size", map_voxel_size_);
    get_parameter("background_radius", background_radius_);
    get_parameter("slot.size_xy", slot_size_xy_);
    get_parameter("slot.z_min_offset", slot_z_min_offset_);
    get_parameter("slot.z_max_offset", slot_z_max_offset_);
    get_parameter("slot.voxel_size", slot_voxel_size_);
    get_parameter("slot.min_new_points", min_new_points_);
    get_parameter("slot.min_occupied_voxels", min_occupied_voxels_);
    get_parameter("occupied_count", occupied_count_);

    map_voxel_size_ = std::max(0.0, map_voxel_size_);
    background_radius_ = std::max(0.01, background_radius_);
    slot_size_xy_ = std::max(0.01, slot_size_xy_);
    slot_voxel_size_ = std::max(0.01, slot_voxel_size_);
    min_new_points_ = std::max(1, min_new_points_);
    min_occupied_voxels_ = std::max(1, min_occupied_voxels_);
    occupied_count_ = std::clamp(occupied_count_, 1, kSlotCount);
    if (slot_z_max_offset_ <= slot_z_min_offset_) {
      throw std::runtime_error("slot.z_max_offset must be greater than slot.z_min_offset");
    }

    std::vector<double> centers;
    get_parameter("slots.centers", centers);
    if (centers.size() != kSlotCount * 3) {
      std::ostringstream error;
      error << "slots.centers must contain " << kSlotCount * 3
            << " numbers, got " << centers.size();
      throw std::runtime_error(error.str());
    }

    slots_.clear();
    slots_.reserve(kSlotCount);
    slots_configured_ = false;
    for (int i = 0; i < kSlotCount; ++i) {
      SlotRoi roi;
      roi.x = centers[i * 3 + 0];
      roi.y = centers[i * 3 + 1];
      roi.z = centers[i * 3 + 2];
      slots_.push_back(roi);
      if (std::abs(roi.x) > 1e-6 || std::abs(roi.y) > 1e-6 ||
          std::abs(roi.z) > 1e-6) {
        slots_configured_ = true;
      }
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!slots_configured_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "KFS ROI coordinates are still placeholders; skip detection.");
      publishEmptyResult(msg->header.stamp);
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
          map_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for TF %s <- %s: %s", map_frame_.c_str(),
                           msg->header.frame_id.c_str(), ex.what());
      return;
    }

    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*msg, *cloud);
    const Matrix4f map_from_cloud = transformToMatrix(tf_msg);
    const float radius_squared = static_cast<float>(background_radius_ *
                                                    background_radius_);
    std::vector<SlotStats> stats(kSlotCount);
    Cloud::Ptr debug_cloud(new Cloud);
    debug_cloud->reserve(cloud->size());

    for (const auto &raw_point : cloud->points) {
      if (!pcl::isFinite(raw_point)) {
        continue;
      }
      const Eigen::Vector4f transformed =
          map_from_cloud * Eigen::Vector4f(raw_point.x, raw_point.y,
                                           raw_point.z, 1.0F);
      Point point;
      point.x = transformed.x();
      point.y = transformed.y();
      point.z = transformed.z();
      point.intensity = raw_point.intensity;

      if (map_manager_->hasNeighborWithinSquaredRadius(point, radius_squared)) {
        continue;
      }

      bool inside_any_slot = false;
      for (std::size_t index = 0; index < slots_.size(); ++index) {
        if (!contains(slots_[index], point)) {
          continue;
        }
        inside_any_slot = true;
        ++stats[index].point_count;
        stats[index].voxels.insert(voxelKey(point, slot_voxel_size_));
      }
      if (inside_any_slot) {
        debug_cloud->push_back(point);
      }
    }

    for (auto &stat : stats) {
      stat.voxel_count = static_cast<int>(stat.voxels.size());
      stat.score = static_cast<float>(stat.voxel_count);
    }
    chooseOccupiedSlots(stats);
    publishResult(stats, debug_cloud, msg->header.stamp);
  }

  bool contains(const SlotRoi &slot, const Point &point) const {
    const double half = slot_size_xy_ * 0.5;
    return std::abs(point.x - slot.x) <= half &&
           std::abs(point.y - slot.y) <= half &&
           point.z >= slot.z + slot_z_min_offset_ &&
           point.z <= slot.z + slot_z_max_offset_;
  }

  void chooseOccupiedSlots(std::vector<SlotStats> &stats) const {
    std::vector<int> indices(stats.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&stats](int lhs, int rhs) {
      if (stats[lhs].score == stats[rhs].score) {
        return lhs < rhs;
      }
      return stats[lhs].score > stats[rhs].score;
    });

    int selected = 0;
    for (const int index : indices) {
      const bool passes_threshold =
          stats[index].point_count >= min_new_points_ &&
          stats[index].voxel_count >= min_occupied_voxels_;
      if (!passes_threshold || selected >= occupied_count_) {
        stats[index].occupied = false;
        continue;
      }
      stats[index].occupied = true;
      ++selected;
    }
  }

  void publishEmptyResult(const rclcpp::Time &stamp) {
    std::vector<SlotStats> stats(kSlotCount);
    Cloud::Ptr empty_cloud(new Cloud);
    publishResult(stats, empty_cloud, stamp);
  }

  void publishResult(const std::vector<SlotStats> &stats,
                     const Cloud::Ptr &debug_cloud,
                     const rclcpp::Time &stamp) {
    std_msgs::msg::Int32MultiArray slots_msg;
    slots_msg.data.reserve(stats.size());
    std_msgs::msg::Float32MultiArray scores_msg;
    scores_msg.data.reserve(stats.size());
    for (const auto &stat : stats) {
      slots_msg.data.push_back(stat.occupied ? 1 : 0);
      scores_msg.data.push_back(stat.score);
    }
    slots_pub_->publish(slots_msg);
    scores_pub_->publish(scores_msg);

    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*debug_cloud, debug_msg);
    debug_msg.header.stamp = stamp;
    debug_msg.header.frame_id = map_frame_;
    debug_cloud_pub_->publish(debug_msg);
    publishMarkers(stats, stamp);
  }

  void publishMarkers(const std::vector<SlotStats> &stats,
                      const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray array;
    for (std::size_t index = 0; index < slots_.size(); ++index) {
      const auto &slot = slots_[index];
      const auto &stat = stats[index];
      visualization_msgs::msg::Marker box;
      box.header.frame_id = map_frame_;
      box.header.stamp = stamp;
      box.ns = "kfs_roi";
      box.id = static_cast<int>(index);
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = slot.x;
      box.pose.position.y = slot.y;
      box.pose.position.z = slot.z + (slot_z_min_offset_ + slot_z_max_offset_) * 0.5;
      box.pose.orientation.w = 1.0;
      box.scale.x = slot_size_xy_;
      box.scale.y = slot_size_xy_;
      box.scale.z = slot_z_max_offset_ - slot_z_min_offset_;
      box.color.a = stat.occupied ? 0.35F : 0.12F;
      box.color.r = stat.occupied ? 0.0F : 1.0F;
      box.color.g = stat.occupied ? 1.0F : 0.0F;
      box.color.b = 0.0F;
      array.markers.push_back(box);

      visualization_msgs::msg::Marker text;
      text.header = box.header;
      text.ns = "kfs_roi_label";
      text.id = static_cast<int>(index);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = slot.x;
      text.pose.position.y = slot.y;
      text.pose.position.z = slot.z + slot_z_max_offset_ + 0.10;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.18;
      text.color.a = 1.0F;
      text.color.r = 1.0F;
      text.color.g = 1.0F;
      text.color.b = 1.0F;
      std::ostringstream label;
      label << index << ": " << (stat.occupied ? "KFS" : "empty")
            << " p=" << stat.point_count << " v=" << stat.voxel_count;
      text.text = label.str();
      array.markers.push_back(text);
    }
    marker_pub_->publish(array);
  }

  static constexpr int kSlotCount = 12;

  std::string map_path_;
  std::string map_frame_;
  std::string input_cloud_topic_;
  double map_voxel_size_{0.10};
  double background_radius_{0.08};
  double slot_size_xy_{0.35};
  double slot_z_min_offset_{0.02};
  double slot_z_max_offset_{0.55};
  double slot_voxel_size_{0.05};
  int min_new_points_{20};
  int min_occupied_voxels_{6};
  int occupied_count_{8};
  bool slots_configured_{false};
  std::vector<SlotRoi> slots_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<r2_localizer::MapManager> map_manager_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr slots_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr scores_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<R2KfsDetectorNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("r2_kfs_detector"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
