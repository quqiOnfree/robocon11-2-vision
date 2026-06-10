#include "MapManager.hpp"

#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace r2_localizer {

MapManager::MapManager(std::string map_path, double voxel_size)
    : map_path_(std::move(map_path)), voxel_size_(voxel_size) {
  loadMap();
}

const Cloud::Ptr &MapManager::mapCloud() const noexcept { return map_cloud_; }

const Eigen::Vector3f &MapManager::mapMin() const noexcept { return map_min_; }

const Eigen::Vector3f &MapManager::mapMax() const noexcept { return map_max_; }

const std::string &MapManager::mapPath() const noexcept { return map_path_; }

bool MapManager::hasNeighborWithinSquaredRadius(
    const Point &point, float radius_squared) const {
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  return map_kdtree_.nearestKSearch(point, 1, nearest_index,
                                    nearest_squared_distance) > 0 &&
         nearest_squared_distance.front() <= radius_squared;
}

// 以车体当前位置为球心裁剪局部地图，后续可用于降低大地图场景中的 NDT 开销。
Cloud::Ptr MapManager::cropLocalMap(const Matrix4f &map_to_body,
                                    double radius) const {
  Point center;
  center.x = map_to_body(0, 3);
  center.y = map_to_body(1, 3);
  center.z = map_to_body(2, 3);
  std::vector<int> indices;
  std::vector<float> squared_distances;
  map_kdtree_.radiusSearch(center, radius, indices, squared_distances);

  Cloud::Ptr local_map(new Cloud);
  local_map->reserve(indices.size());
  for (const int index : indices) {
    local_map->push_back(map_cloud_->points[index]);
  }
  local_map->width = static_cast<std::uint32_t>(local_map->size());
  local_map->height = 1;
  return local_map;
}

Cloud::Ptr MapManager::voxelDownsample(const Cloud::Ptr &cloud,
                                       double leaf_size) {
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

void MapManager::loadMap() {
  // 地图只在启动时加载一次；下采样结果同时用于发布、KDTree 查询和 NDT target。
  Cloud::Ptr raw_map(new Cloud);
  if (pcl::io::loadPCDFile<Point>(map_path_, *raw_map) < 0 ||
      raw_map->empty()) {
    throw std::runtime_error("Unable to load PCD map: " + map_path_);
  }
  map_cloud_ = voxelDownsample(raw_map, voxel_size_);
  if (map_cloud_->empty()) {
    throw std::runtime_error("PCD map is empty after filtering: " + map_path_);
  }

  // bbox 可快速拒绝落在地图范围之外的预测扫描点。
  map_min_ = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  map_max_ = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());
  for (const auto &point : map_cloud_->points) {
    const Eigen::Vector3f xyz(point.x, point.y, point.z);
    map_min_ = map_min_.cwiseMin(xyz);
    map_max_ = map_max_.cwiseMax(xyz);
  }
  map_kdtree_.setInputCloud(map_cloud_);
}

}  // namespace r2_localizer
