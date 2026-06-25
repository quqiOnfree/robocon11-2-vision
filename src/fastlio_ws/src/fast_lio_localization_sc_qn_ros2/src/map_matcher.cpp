#include "fast_lio_localization_sc_qn_ros2/map_matcher.hpp"

#include <iostream>

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
  const auto candidate = scan_context_.detectLoopClosureIDGivenScan(query.cloud_local);
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

CloudPair MapMatcher::makeSourceAndTarget(const PoseCloud &query,
                                          const std::vector<MapKeyframe> &map_keyframes,
                                          int candidate_index) const {
  Cloud target_accum;
  Cloud source_world = transformCloud(query.cloud_local, query.pose_corrected);

  if (config_.enable_quatro) {
    target_accum = transformCloud(map_keyframes[candidate_index].cloud_local,
                                  map_keyframes[candidate_index].pose);
  } else {
    for (int i = candidate_index - config_.num_submap_keyframes;
         i <= candidate_index + config_.num_submap_keyframes; ++i) {
      if (i >= 0 && i < static_cast<int>(map_keyframes.size())) {
        target_accum += transformCloud(map_keyframes[i].cloud_local, map_keyframes[i].pose);
      }
    }
  }

  return {*voxelizeCloud(source_world, config_.voxel_resolution),
          *voxelizeCloud(target_accum, config_.voxel_resolution)};
}

RegistrationOutput MapMatcher::icpAlign(const Cloud &source, const Cloud &target) {
  RegistrationOutput output;
  final_aligned_cloud_.clear();

  Cloud::Ptr source_ptr(new Cloud(source));
  Cloud::Ptr target_ptr(new Cloud(target));
  nano_gicp_.setInputSource(source_ptr);
  nano_gicp_.calculateSourceCovariances();
  nano_gicp_.setInputTarget(target_ptr);
  nano_gicp_.calculateTargetCovariances();
  nano_gicp_.align(final_aligned_cloud_);

  output.score = nano_gicp_.getFitnessScore();
  output.converged = nano_gicp_.hasConverged();
  if (output.converged && output.score < config_.gicp.fitness_score_threshold) {
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

  if (config_.enable_quatro) {
    return coarseToFineAlign(source, target);
  }
  return icpAlign(source, target);
}

}  // namespace fast_lio_localization_sc_qn_ros2
