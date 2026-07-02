#pragma once

#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>

#include <Scancontext.h>
#include <nano_gicp/nano_gicp.hpp>
#include <quatro/quatro_module.h>

#include "fast_lio_sam_sc_qn_ros2/pose_cloud.hpp"

namespace fast_lio_sam_sc_qn_ros2 {

struct NanoGicpConfig {
  int thread_number = 0;
  int correspondences_number = 15;
  int max_iterations = 32;
  int ransac_iterations = 5;
  double max_correspondence_distance = 35.0;
  double fitness_score_threshold = 1.5;
  double transformation_epsilon = 0.01;
  double euclidean_fitness_epsilon = 0.01;
  double ransac_outlier_rejection_threshold = 1.0;
};

struct QuatroConfig {
  bool use_optimized_matching = true;
  bool estimate_scale = false;
  int max_correspondences = 500;
  int max_iterations = 50;
  double distance_threshold = 35.0;
  double fpfh_normal_radius = 0.30;
  double fpfh_radius = 0.50;
  double noise_bound = 0.10;
  double rotation_gnc_factor = 1.40;
  double rotation_cost_diff_threshold = 0.0001;
};

struct LoopClosureConfig {
  bool enable = true;
  bool enable_quatro = true;
  bool enable_submap_matching = true;
  int num_submap_keyframes = 10;
  double voxel_resolution = 0.10;
  double scancontext_max_correspondence_distance = 35.0;
  NanoGicpConfig nano_gicp;
  QuatroConfig quatro;
};

struct RegistrationResult {
  bool valid = false;
  bool converged = false;
  double score = std::numeric_limits<double>::max();
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
};

class LoopClosure {
public:
  explicit LoopClosure(const LoopClosureConfig &config);

  void updateScanContext(const Cloud &cloud_local);
  int fetchCandidateKeyframeIndex(const PoseCloud &query,
                                  const std::vector<PoseCloud> &keyframes);
  RegistrationResult perform(const PoseCloud &query,
                             const std::vector<PoseCloud> &keyframes,
                             int candidate_index);

  Cloud sourceCloud() const { return source_cloud_; }
  Cloud targetCloud() const { return target_cloud_; }
  Cloud coarseAlignedCloud() const { return coarse_aligned_; }
  Cloud finalAlignedCloud() const { return final_aligned_; }

private:
  using CloudPair = std::tuple<Cloud, Cloud>;

  CloudPair makeSourceAndTargetCloud(const std::vector<PoseCloud> &keyframes,
                                     int source_index, int target_index) const;
  RegistrationResult alignWithNanoGicp(const Cloud &source, const Cloud &target);
  RegistrationResult alignCoarseToFine(const Cloud &source, const Cloud &target);

  LoopClosureConfig config_;
  SCManager scan_context_;
  nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
  std::shared_ptr<quatro<PointType>> quatro_;

  Cloud source_cloud_;
  Cloud target_cloud_;
  Cloud coarse_aligned_;
  Cloud final_aligned_;
};

}  // namespace fast_lio_sam_sc_qn_ros2
