#include "NDTAligner.hpp"

#include <cmath>
#include <cstdint>

#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>

namespace r2_localizer {

// MapManager 通过组合注入；NDT target 保持为下采样后的全局地图。
NDTAligner::NDTAligner(Config config, const MapManager &map_manager)
    : config_(config), map_manager_(map_manager) {
  ndt_.setTransformationEpsilon(config_.transformation_epsilon);
  ndt_.setStepSize(config_.step_size);
  ndt_.setResolution(config_.resolution);
  ndt_.setMaximumIterations(config_.maximum_iterations);
  ndt_.setInputTarget(map_manager_.mapCloud());
}

Cloud::Ptr NDTAligner::voxelDownsample(const Cloud::Ptr &cloud,
                                       double leaf_size) const {
  if (leaf_size <= 0.0) {
    return Cloud::Ptr(new Cloud(*cloud));
  }
  pcl::VoxelGrid<Point> voxel;
  Cloud::Ptr output(new Cloud);
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(static_cast<float>(leaf_size),
                    static_cast<float>(leaf_size),
                    static_cast<float>(leaf_size));
  voxel.filter(*output);
  return output;
}

// 先按距离和高度去除无效点，再体素滤波，控制每帧 NDT 的计算量。
Cloud::Ptr NDTAligner::filterSource(const Cloud::Ptr &cloud) const {
  Cloud::Ptr output(new Cloud);
  output->reserve(cloud->size());
  const double min_sq = config_.min_range * config_.min_range;
  const double max_sq = config_.max_range * config_.max_range;
  for (const auto &point : cloud->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const double range_sq =
        point.x * point.x + point.y * point.y + point.z * point.z;
    if (range_sq < min_sq || range_sq > max_sq || point.z < config_.min_z ||
        point.z > config_.max_z) {
      continue;
    }
    output->push_back(point);
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  return voxelDownsample(output, config_.source_voxel_size);
}

// 使用初值将扫描点投影到 map，保留地图附近的点，减少稀疏地图中的误匹配。
Cloud::Ptr NDTAligner::cropSourceToMapOverlap(const Cloud::Ptr &source,
                                              const Matrix4f &guess) const {
  Cloud::Ptr output(new Cloud);
  output->reserve(source->size());
  const Eigen::Vector3f padding = Eigen::Vector3f::Constant(
      static_cast<float>(config_.map_overlap_padding));
  const Eigen::Vector3f lower = map_manager_.mapMin() - padding;
  const Eigen::Vector3f upper = map_manager_.mapMax() + padding;
  const float radius_squared = static_cast<float>(
      config_.map_match_radius * config_.map_match_radius);
  for (const auto &point : source->points) {
    const Eigen::Vector4f predicted =
        guess * Eigen::Vector4f(point.x, point.y, point.z, 1.0f);
    const Eigen::Vector3f xyz = predicted.head<3>();
    if (!((xyz.array() >= lower.array()).all() &&
          (xyz.array() <= upper.array()).all())) {
      continue;
    }
    Point predicted_point;
    predicted_point.x = xyz.x();
    predicted_point.y = xyz.y();
    predicted_point.z = xyz.z();
    if (map_manager_.hasNeighborWithinSquaredRadius(predicted_point,
                                                    radius_squared)) {
      output->push_back(point);
    }
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  return output;
}

// 执行单次 NDT，并原样返回收敛状态、fitness、修正量和配准后的扫描点云。
NDTAligner::Result NDTAligner::align(const Cloud::Ptr &source,
                                     const Matrix4f &guess,
                                     std::size_t min_source_points) {
  Result result;
  result.input_points = source->size();
  if (source->size() < min_source_points) {
    return result;
  }

  const Cloud::Ptr overlap_source = cropSourceToMapOverlap(source, guess);
  result.overlap_points = overlap_source->size();
  if (overlap_source->size() < min_source_points) {
    result.status = Status::kInsufficientOverlapPoints;
    return result;
  }

  result.status = Status::kReady;
  result.overlap_ratio = static_cast<double>(overlap_source->size()) /
                         static_cast<double>(source->size());
  ndt_.setInputSource(overlap_source);
  ndt_.align(*result.registered, guess);
  result.converged = ndt_.hasConverged();
  result.fitness = ndt_.getFitnessScore();
  result.candidate = ndt_.getFinalTransformation();
  transformDelta(result.candidate, guess, result.translation_delta,
                 result.yaw_delta, result.roll_pitch_delta);
  return result;
}

double NDTAligner::yawOf(const Matrix4f &matrix) {
  return std::atan2(matrix(1, 0), matrix(0, 0));
}

double NDTAligner::normalizedAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

void NDTAligner::transformDelta(const Matrix4f &candidate,
                                const Matrix4f &reference,
                                double &translation, double &yaw,
                                double &roll_pitch) {
  const Matrix4f delta = reference.inverse() * candidate;
  translation = delta.block<3, 1>(0, 3).norm();
  yaw = std::abs(normalizedAngle(yawOf(delta)));
  const auto rotation = delta.block<3, 3>(0, 0);
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch =
      std::atan2(-rotation(2, 0),
                 std::hypot(rotation(2, 1), rotation(2, 2)));
  roll_pitch = std::hypot(roll, pitch);
}

}  // namespace r2_localizer
