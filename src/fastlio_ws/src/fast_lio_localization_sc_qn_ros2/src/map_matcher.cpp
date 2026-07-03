#include "fast_lio_localization_sc_qn_ros2/map_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>

namespace fast_lio_localization_sc_qn_ros2 {

MapMatcher::MapMatcher(const MapMatcherConfig &config) : config_(config) {
  const auto &g = config_.gicp;
  nano_gicp_.setNumThreads(g.thread_number);
  nano_gicp_.setCorrespondenceRandomness(g.correspondences_number);
  nano_gicp_.setMaximumIterations(g.max_iterations);
  nano_gicp_.setRANSACIterations(g.ransac_iterations);
  nano_gicp_.setMaxCorrespondenceDistance(g.max_correspondence_distance);
  nano_gicp_.setTransformationEpsilon(g.transformation_epsilon);
  nano_gicp_.setEuclideanFitnessEpsilon(g.euclidean_fitness_epsilon);
  nano_gicp_.setRANSACOutlierRejectionThreshold(g.ransac_outlier_rejection_threshold);

  const auto &q = config_.quatro;
  quatro_ = std::make_shared<quatro<PointType>>(
      q.fpfh_normal_radius,
      q.fpfh_radius,
      q.noise_bound,
      q.rotation_gnc_factor,
      q.rotation_cost_diff_threshold,
      q.max_iterations,
      q.estimate_scale,
      q.use_optimized_matching,
      q.distance_threshold,
      q.max_correspondences);
}

void MapMatcher::addMapScanContext(const Cloud &cloud) {
  scan_context_.makeAndSaveScancontextAndKeys(cloud);
}

int MapMatcher::fetchClosestKeyframeIndex(const PoseCloud &query,
                                          const std::vector<MapKeyframe> &map_keyframes) {
  // 预制地图与实时扫描属于两个独立 session，必须搜索全部地图描述子。
  // detectLoopClosureIDGivenScan() 是在线闭环接口，会排除最近 30 帧，短地图会永远无候选。
  Eigen::MatrixXd query_descriptor = scan_context_.makeScancontext(query.cloud_local);
  Eigen::MatrixXd query_ring_key =
      scan_context_.makeRingkeyFromScancontext(query_descriptor);
  std::vector<float> query_ring_key_vector = eig2stdvec(query_ring_key);
  const auto candidate = scan_context_.detectLoopClosureIDBetweenSession(
      query_ring_key_vector, query_descriptor);
  const int candidate_index = candidate.first;
  if (candidate_index < 0 || candidate_index >= static_cast<int>(map_keyframes.size())) {
    return -1;
  }

  if (config_.enable_distance_gate) {
    const double distance =
        (map_keyframes[candidate_index].pose.block<3, 1>(0, 3) -
         query.pose_corrected.block<3, 1>(0, 3)).norm();
    if (distance > config_.scancontext_max_correspondence_distance) {
      return -1;
    }
  }
  return candidate_index;
}

std::vector<RegistrationOutput> MapMatcher::fetchCandidateKeyframes(
    const PoseCloud &query,
    const std::vector<MapKeyframe> &map_keyframes) {
  Eigen::MatrixXd query_descriptor =
      scan_context_.makeScancontext(query.cloud_local);
  std::vector<RegistrationOutput> candidates;
  candidates.reserve(map_keyframes.size());

  const size_t count =
      std::min(map_keyframes.size(), scan_context_.polarcontexts_.size());
  for (size_t i = 0; i < count; ++i) {
    Eigen::MatrixXd map_descriptor = scan_context_.polarcontexts_[i];
    const auto [distance, yaw_shift] =
        scan_context_.distanceBtnScanContext(query_descriptor, map_descriptor);
    if (!std::isfinite(distance) ||
        distance > config_.scancontext_distance_threshold) {
      continue;
    }

    RegistrationOutput candidate;
    candidate.candidate_index = static_cast<int>(i);
    candidate.scancontext_distance = distance;
    // yaw_shift 仅用于 ScanContext 距离计算，完整 6DoF 由 GICP/Quatro 求解。
    (void)yaw_shift;
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const RegistrationOutput &a, const RegistrationOutput &b) {
              return a.scancontext_distance < b.scancontext_distance;
            });
  if (candidates.size() >
      static_cast<size_t>(std::max(1, config_.scancontext_num_candidates))) {
    candidates.resize(
        static_cast<size_t>(std::max(1, config_.scancontext_num_candidates)));
  }
  return candidates;
}

CloudPair MapMatcher::makeSourceAndTarget(const PoseCloud &query,
                                          const std::vector<MapKeyframe> &map_keyframes,
                                          int candidate_index) const {
  Cloud target_accum;
  Cloud source_world = transformCloud(query.cloud_local, query.pose_corrected);

  for (int i = candidate_index - config_.num_submap_keyframes;
       i <= candidate_index + config_.num_submap_keyframes; ++i) {
    if (i >= 0 && i < static_cast<int>(map_keyframes.size())) {
      target_accum += transformCloud(map_keyframes[i].cloud_local,
                                    map_keyframes[i].pose);
    }
  }
  return {*voxelizeCloud(source_world, config_.voxel_resolution),
          *voxelizeCloud(target_accum, config_.voxel_resolution)};
}

double MapMatcher::calculateOverlap(const Cloud &aligned,
                                    const Cloud &target) const {
  if (aligned.empty() || target.empty()) {
    return 0.0;
  }
  Cloud::ConstPtr target_ptr(new Cloud(target));
  pcl::KdTreeFLANN<PointType> tree;
  tree.setInputCloud(target_ptr);
  const double max_sq =
      config_.overlap_max_distance * config_.overlap_max_distance;
  size_t matched = 0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto &point : aligned) {
    if (tree.nearestKSearch(point, 1, indices, squared_distances) > 0 &&
        squared_distances[0] <= max_sq) {
      ++matched;
    }
  }
  return static_cast<double>(matched) /
         static_cast<double>(aligned.size());
}

RegistrationOutput MapMatcher::icpAlign(
    const Cloud &source, const Cloud &target,
    const Eigen::Matrix4d &initial_guess) {
  RegistrationOutput output;
  final_aligned_cloud_.clear();

  Cloud::Ptr source_ptr(new Cloud(source));
  Cloud::Ptr target_ptr(new Cloud(target));
  nano_gicp_.setInputSource(source_ptr);
  nano_gicp_.calculateSourceCovariances();
  nano_gicp_.setInputTarget(target_ptr);
  nano_gicp_.calculateTargetCovariances();
  nano_gicp_.align(final_aligned_cloud_, initial_guess.cast<float>());

  output.score = nano_gicp_.getFitnessScore();
  output.converged = nano_gicp_.hasConverged();
  output.overlap_ratio = calculateOverlap(final_aligned_cloud_, target);
  if (output.converged &&
      output.score < config_.gicp.fitness_score_threshold &&
      output.overlap_ratio >= config_.min_overlap_ratio) {
    output.valid = true;
    output.transform = nano_gicp_.getFinalTransformation().cast<double>();
  }
  return output;
}

RegistrationOutput MapMatcher::coarseToFineAlign(const Cloud &source, const Cloud &target) {
  RegistrationOutput coarse;
  coarse_aligned_cloud_.clear();

  coarse.transform = quatro_->align(source, target, coarse.converged);
  if (!coarse.converged) {
    return coarse;
  }

  coarse_aligned_cloud_ = transformCloud(source, coarse.transform);
  RegistrationOutput fine = icpAlign(coarse_aligned_cloud_, target);
  fine.transform = fine.transform * coarse.transform;
  return fine;
}

RegistrationOutput MapMatcher::perform(const PoseCloud &query,
                                       const std::vector<MapKeyframe> &map_keyframes,
                                       int candidate_index) {
  RegistrationOutput output;
  output.candidate_index = candidate_index;
  if (candidate_index < 0 || candidate_index >= static_cast<int>(map_keyframes.size())) {
    return output;
  }

  const auto [source, target] = makeSourceAndTarget(query, map_keyframes, candidate_index);
  source_cloud_ = source;
  target_cloud_ = target;

  // ScanContext 已给出候选关键帧。优先用候选地图位姿初始化 GICP，
  // 避免动态人员或稀疏单帧让全局特征配准随机跳到错误副本。
  const Eigen::Matrix4d initial_guess =
      map_keyframes[candidate_index].pose *
      query.pose_corrected.inverse();
  coarse_aligned_cloud_ = transformCloud(source, initial_guess);
  RegistrationOutput seeded = icpAlign(source, target, initial_guess);
  seeded.candidate_index = candidate_index;
  if (seeded.valid || !config_.enable_quatro) {
    return seeded;
  }

  RegistrationOutput global = coarseToFineAlign(source, target);
  global.candidate_index = candidate_index;
  if (global.valid ||
      (!seeded.converged && global.converged) ||
      global.score < seeded.score) {
    return global;
  }
  return seeded;
}

}  // namespace fast_lio_localization_sc_qn_ros2
