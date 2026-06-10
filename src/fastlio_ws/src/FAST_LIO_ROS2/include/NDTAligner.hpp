#pragma once

#include <cstddef>

#include <Eigen/Dense>
#include <pcl/registration/ndt.h>

#include "MapManager.hpp"

namespace r2_localizer {

// 封装扫描点云预处理和 PCL NDT。是否接受匹配结果仍由顶层节点决定。
class NDTAligner {
public:
  struct Config {
    double source_voxel_size{0.20};
    double map_overlap_padding{0.50};
    double map_match_radius{0.50};
    double min_range{0.0};
    double max_range{100.0};
    double min_z{-100.0};
    double max_z{100.0};
    double transformation_epsilon{0.01};
    double step_size{0.10};
    double resolution{0.50};
    int maximum_iterations{35};
  };

  // 区分“尚未执行 NDT”和“已经完成 NDT”，便于顶层节点保持原有日志行为。
  enum class Status {
    kReady,
    kInsufficientSourcePoints,
    kInsufficientOverlapPoints,
  };

  // 返回 NDT 原始结果和质量指标，不在算法层隐藏策略判断。
  struct Result {
    Status status{Status::kInsufficientSourcePoints};
    std::size_t input_points{0};
    std::size_t overlap_points{0};
    double overlap_ratio{0.0};
    bool converged{false};
    double fitness{0.0};
    Matrix4f candidate{Matrix4f::Identity()};
    double translation_delta{0.0};
    double yaw_delta{0.0};
    double roll_pitch_delta{0.0};
    Cloud::Ptr registered{new Cloud};
  };

  NDTAligner(Config config, const MapManager &map_manager);

  Cloud::Ptr voxelDownsample(const Cloud::Ptr &cloud, double leaf_size) const;
  Cloud::Ptr filterSource(const Cloud::Ptr &cloud) const;
  Cloud::Ptr cropSourceToMapOverlap(const Cloud::Ptr &source,
                                    const Matrix4f &guess) const;
  Result align(const Cloud::Ptr &source, const Matrix4f &guess,
               std::size_t min_source_points);

private:
  static double yawOf(const Matrix4f &matrix);
  static double normalizedAngle(double angle);
  static void transformDelta(const Matrix4f &candidate,
                             const Matrix4f &reference, double &translation,
                             double &yaw, double &roll_pitch);

  Config config_;
  const MapManager &map_manager_;
  pcl::NormalDistributionsTransform<Point, Point> ndt_;
};

}  // namespace r2_localizer
