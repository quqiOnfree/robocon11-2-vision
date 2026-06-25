#pragma once

#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <Scancontext.h>
#include <nano_gicp/nano_gicp.hpp>
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <pcl/point_cloud.h>
#include <quatro/quatro_module.h>

#include "fast_lio_localization_sc_qn_ros2/pose_cloud.hpp"
#include "fast_lio_localization_sc_qn_ros2/utilities.hpp"

namespace fast_lio_localization_sc_qn_ros2 {

struct NanoGicpConfig {
  int thread_number = 0;
  int correspondences_number = 15;
  int max_iterations = 32;
  int ransac_iterations = 5;
  double max_correspondence_distance = 2.0;
  double fitness_score_threshold = 10.0;
  double transformation_epsilon = 0.01;
  double euclidean_fitness_epsilon = 0.01;
  double ransac_outlier_rejection_threshold = 1.0;
};

struct QuatroConfig {
  bool use_optimized_matching = true;
  bool estimate_scale = false;
  int max_correspondences = 500;
  int max_iterations = 50;
  double distance_threshold = 30.0;
  double fpfh_normal_radius = 0.30;
  double fpfh_radius = 0.50;
  double noise_bound = 0.30;
  double rotation_gnc_factor = 1.40;
  double rotation_cost_diff_threshold = 0.0001;
};

struct MapMatcherConfig {
  bool enable_quatro = true;
  bool enable_distance_gate = false;
  int num_submap_keyframes = 10;
  double voxel_resolution = 0.10;
  double scancontext_max_correspondence_distance = 30.0;
  NanoGicpConfig gicp;
  QuatroConfig quatro;
};

struct RegistrationOutput {
  bool valid = false;
  bool converged = false;
  double score = std::numeric_limits<double>::max();
  int candidate_index = -1;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
};

using CloudPair = std::tuple<Cloud, Cloud>;

class MapMatcher {
public:
  explicit MapMatcher(const MapMatcherConfig &config);

  void addMapScanContext(const Cloud &cloud);
  int fetchClosestKeyframeIndex(const PoseCloud &query,
                                const std::vector<MapKeyframe> &map_keyframes);
  RegistrationOutput perform(const PoseCloud &query,
                             const std::vector<MapKeyframe> &map_keyframes,
                             int candidate_index);

  const Cloud &sourceCloud() const { return source_cloud_; }
  const Cloud &targetCloud() const { return target_cloud_; }
  const Cloud &coarseAlignedCloud() const { return coarse_aligned_cloud_; }
  const Cloud &finalAlignedCloud() const { return final_aligned_cloud_; }

private:
  CloudPair makeSourceAndTarget(const PoseCloud &query,
                                const std::vector<MapKeyframe> &map_keyframes,
                                int candidate_index) const;
  RegistrationOutput icpAlign(const Cloud &source, const Cloud &target);
  RegistrationOutput coarseToFineAlign(const Cloud &source, const Cloud &target);

  MapMatcherConfig config_;
  SCManager scan_context_;
  nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
  std::shared_ptr<quatro<PointType>> quatro_;
  Cloud source_cloud_;
  Cloud target_cloud_;
  Cloud coarse_aligned_cloud_;
  Cloud final_aligned_cloud_;
};

}  // namespace fast_lio_localization_sc_qn_ros2
