#pragma once

#include <string>

#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace r2_localizer {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Matrix4f = Eigen::Matrix4f;

// 管理预制 PCD 地图及其空间索引。该类不关心 ROS2 消息，也不执行 NDT。
class MapManager {
public:
  MapManager(std::string map_path, double voxel_size);

  const Cloud::Ptr &mapCloud() const noexcept;
  const Eigen::Vector3f &mapMin() const noexcept;
  const Eigen::Vector3f &mapMax() const noexcept;
  const std::string &mapPath() const noexcept;

  // 判断预测点附近是否存在地图点，用于剔除明显不可能参与配准的扫描点。
  bool hasNeighborWithinSquaredRadius(const Point &point,
                                      float radius_squared) const;
  // 预留的局部地图接口。首期仍使用全局地图作为 NDT target，以保持原有行为。
  Cloud::Ptr cropLocalMap(const Matrix4f &map_to_body, double radius) const;

private:
  static Cloud::Ptr voxelDownsample(const Cloud::Ptr &cloud, double leaf_size);
  void loadMap();

  std::string map_path_;
  double voxel_size_;
  Cloud::Ptr map_cloud_{new Cloud};
  Eigen::Vector3f map_min_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f map_max_{Eigen::Vector3f::Zero()};
  mutable pcl::KdTreeFLANN<Point> map_kdtree_;
};

}  // namespace r2_localizer
