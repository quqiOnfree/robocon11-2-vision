#include "fast_lio_sam_sc_qn_ros2/loop_closure.hpp"

#include <algorithm>
#include <iostream>

namespace fast_lio_sam_sc_qn_ros2 {

LoopClosure::LoopClosure(const LoopClosureConfig &config) : config_(config) {
  const auto &gc = config_.nano_gicp;
  nano_gicp_.setNumThreads(gc.thread_number);
  nano_gicp_.setCorrespondenceRandomness(gc.correspondences_number);
  nano_gicp_.setMaximumIterations(gc.max_iterations);
  nano_gicp_.setRANSACIterations(gc.ransac_iterations);
  nano_gicp_.setMaxCorrespondenceDistance(gc.max_correspondence_distance);
  nano_gicp_.setTransformationEpsilon(gc.transformation_epsilon);
  nano_gicp_.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon);
  nano_gicp_.setRANSACOutlierRejectionThreshold(
      gc.ransac_outlier_rejection_threshold);

  const auto &qc = config_.quatro;
  if (config_.enable_quatro) {
    quatro_ = std::make_shared<quatro<PointType>>(
        qc.fpfh_normal_radius, qc.fpfh_radius, qc.noise_bound,
        qc.rotation_gnc_factor, qc.rotation_cost_diff_threshold,
        qc.max_iterations, qc.estimate_scale, qc.use_optimized_matching,
        qc.distance_threshold, qc.max_correspondences);
  }
}

void LoopClosure::updateScanContext(const Cloud &cloud_local) {
  scan_context_.makeAndSaveScancontextAndKeys(cloud_local);
}

int LoopClosure::fetchCandidateKeyframeIndex(
    const PoseCloud &query, const std::vector<PoseCloud> &keyframes) {
  const auto detected =
      scan_context_.detectLoopClosureIDGivenScan(query.cloud_local);
  const int candidate = detected.first;
  if (candidate < 0 || candidate >= static_cast<int>(keyframes.size())) {
    return -1;
  }

  const double distance =
      (keyframes[candidate].pose_corrected.block<3, 1>(0, 3) -
       query.pose_corrected.block<3, 1>(0, 3))
          .norm();
  if (distance < config_.scancontext_max_correspondence_distance) {
    return candidate;
  }
  return -1;
}

LoopClosure::CloudPair LoopClosure::makeSourceAndTargetCloud(
    const std::vector<PoseCloud> &keyframes, int source_index,
    int target_index) const {
  Cloud source_accum;
  Cloud target_accum;
  const int range = std::max(0, config_.num_submap_keyframes);

  const auto append_keyframe = [&](Cloud &dst, int i) {
    if (i >= 0 && i < static_cast<int>(keyframes.size())) {
      dst += transformCloud(keyframes[i].cloud_local,
                            keyframes[i].pose_corrected);
    }
  };

  if (config_.enable_submap_matching) {
    for (int i = source_index - range; i <= source_index + range; ++i) {
      append_keyframe(source_accum, i);
    }
    for (int i = target_index - range; i <= target_index + range; ++i) {
      append_keyframe(target_accum, i);
    }
  } else {
    append_keyframe(source_accum, source_index);
    if (config_.enable_quatro) {
      append_keyframe(target_accum, target_index);
    } else {
      for (int i = target_index - range; i <= target_index + range; ++i) {
        append_keyframe(target_accum, i);
      }
    }
  }

  return {*voxelizeCloud(source_accum, config_.voxel_resolution),
          *voxelizeCloud(target_accum, config_.voxel_resolution)};
}

RegistrationResult LoopClosure::alignWithNanoGicp(const Cloud &source,
                                                  const Cloud &target) {
  RegistrationResult result;
  final_aligned_.clear();

  auto source_ptr = Cloud::Ptr(new Cloud(source));
  auto target_ptr = Cloud::Ptr(new Cloud(target));
  nano_gicp_.setInputSource(source_ptr);
  nano_gicp_.calculateSourceCovariances();
  nano_gicp_.setInputTarget(target_ptr);
  nano_gicp_.calculateTargetCovariances();
  nano_gicp_.align(final_aligned_);

  result.score = nano_gicp_.getFitnessScore();
  result.converged = nano_gicp_.hasConverged();
  if (result.converged &&
      result.score < config_.nano_gicp.fitness_score_threshold) {
    result.valid = true;
    result.transform = nano_gicp_.getFinalTransformation().cast<double>();
  }
  return result;
}

RegistrationResult LoopClosure::alignCoarseToFine(const Cloud &source,
                                                  const Cloud &target) {
  RegistrationResult result;
  if (!quatro_) {
    return alignWithNanoGicp(source, target);
  }

  coarse_aligned_.clear();
  result.transform = quatro_->align(source, target, result.converged);
  if (!result.converged) {
    return result;
  }

  coarse_aligned_ = transformCloud(source, result.transform);
  const Eigen::Matrix4d coarse_tf = result.transform;
  result = alignWithNanoGicp(coarse_aligned_, target);
  result.transform = result.transform * coarse_tf;
  return result;
}

RegistrationResult LoopClosure::perform(const PoseCloud &query,
                                         const std::vector<PoseCloud> &keyframes,
                                         int candidate_index) {
  if (candidate_index < 0) {
    return RegistrationResult{};
  }

  const auto [source, target] =
      makeSourceAndTargetCloud(keyframes, query.index, candidate_index);
  source_cloud_ = source;
  target_cloud_ = target;

  if (config_.enable_quatro) {
    return alignCoarseToFine(source, target);
  }
  return alignWithNanoGicp(source, target);
}

}  // namespace fast_lio_sam_sc_qn_ros2
