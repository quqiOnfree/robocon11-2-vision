#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "fast_lio_sam_sc_qn_ros2/loop_closure.hpp"
#include "fast_lio_sam_sc_qn_ros2/pose_cloud.hpp"

namespace fast_lio_sam_sc_qn_ros2 {

class FastLioSamScQnNode : public rclcpp::Node {
public:
  using OdomMsg = nav_msgs::msg::Odometry;
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<OdomMsg, CloudMsg>;

  FastLioSamScQnNode() : Node("fast_lio_sam_sc_qn") {
    declareParameters();
    readParameters();
    setupGraph();
    setupRos();

    RCLCPP_INFO(get_logger(),
                "FAST-LIO-SAM-SC-QN ROS2 backend ready: odom=%s cloud=%s output=%s",
                odom_topic_.c_str(), cloud_topic_.c_str(), global_odom_topic_.c_str());
  }

private:
  void declareParameters() {
    declare_parameter<std::string>("topics.odom", "/Odometry");
    declare_parameter<std::string>("topics.cloud", "/cloud_registered");
    declare_parameter<std::string>("topics.global_odom", "/r2/global_odometry");
    declare_parameter<std::string>("topics.current_pose", "/r2_current_pose");
    declare_parameter<std::string>("topics.corrected_path", "/r2/sam/corrected_path");
    declare_parameter<std::string>("topics.corrected_current_cloud", "/r2/sam/corrected_current_cloud");
    declare_parameter<std::string>("frames.map", "map");
    declare_parameter<std::string>("frames.child", "body");
    declare_parameter<int>("sync.queue_size", 30);
    declare_parameter<double>("sync.max_stamp_diff_sec", 0.03);
    declare_parameter<std::string>("sync.qos_reliability", "reliable");

    declare_parameter<double>("keyframe.distance_threshold", 1.5);
    declare_parameter<int>("keyframe.min_points", 80);
    declare_parameter<double>("graph.prior_rotation_noise", 1e-4);
    declare_parameter<double>("graph.prior_translation_noise", 1e-2);
    declare_parameter<double>("graph.odom_rotation_noise", 1e-4);
    declare_parameter<double>("graph.odom_translation_noise", 1e-2);
    declare_parameter<double>("graph.loop_rotation_noise", 1e-3);
    declare_parameter<double>("graph.loop_translation_noise", 1e-2);

    declare_parameter<bool>("loop.enable", true);
    declare_parameter<bool>("loop.enable_quatro", true);
    declare_parameter<bool>("loop.enable_submap_matching", true);
    declare_parameter<int>("loop.num_submap_keyframes", 10);
    declare_parameter<int>("loop.min_keyframes", 35);
    declare_parameter<double>("loop.voxel_resolution", 0.10);
    declare_parameter<double>("loop.scancontext_max_correspondence_distance", 35.0);

    declare_parameter<int>("nano_gicp.thread_number", 0);
    declare_parameter<int>("nano_gicp.correspondences_number", 15);
    declare_parameter<int>("nano_gicp.max_iterations", 32);
    declare_parameter<int>("nano_gicp.ransac_iterations", 5);
    declare_parameter<double>("nano_gicp.max_correspondence_distance", 35.0);
    declare_parameter<double>("nano_gicp.fitness_score_threshold", 1.5);
    declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
    declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
    declare_parameter<double>("nano_gicp.ransac_outlier_rejection_threshold", 1.0);

    declare_parameter<bool>("quatro.use_optimized_matching", true);
    declare_parameter<bool>("quatro.estimate_scale", false);
    declare_parameter<int>("quatro.max_correspondences", 500);
    declare_parameter<int>("quatro.max_iterations", 50);
    declare_parameter<double>("quatro.distance_threshold", 35.0);
    declare_parameter<double>("quatro.fpfh_normal_radius", 0.30);
    declare_parameter<double>("quatro.fpfh_radius", 0.50);
    declare_parameter<double>("quatro.noise_bound", 0.10);
    declare_parameter<double>("quatro.rotation_gnc_factor", 1.40);
    declare_parameter<double>("quatro.rotation_cost_diff_threshold", 0.0001);
  }

  void readParameters() {
    get_parameter("topics.odom", odom_topic_);
    get_parameter("topics.cloud", cloud_topic_);
    get_parameter("topics.global_odom", global_odom_topic_);
    get_parameter("topics.current_pose", current_pose_topic_);
    get_parameter("topics.corrected_path", corrected_path_topic_);
    get_parameter("topics.corrected_current_cloud", corrected_current_cloud_topic_);
    get_parameter("frames.map", map_frame_);
    get_parameter("frames.child", child_frame_);
    get_parameter("sync.queue_size", sync_queue_size_);
    get_parameter("sync.max_stamp_diff_sec", max_stamp_diff_sec_);
    get_parameter("sync.qos_reliability", sync_qos_reliability_);
    get_parameter("keyframe.distance_threshold", keyframe_distance_threshold_);
    get_parameter("keyframe.min_points", min_keyframe_points_);
    get_parameter("loop.min_keyframes", min_loop_keyframes_);

    get_parameter("graph.prior_rotation_noise", prior_rotation_noise_);
    get_parameter("graph.prior_translation_noise", prior_translation_noise_);
    get_parameter("graph.odom_rotation_noise", odom_rotation_noise_);
    get_parameter("graph.odom_translation_noise", odom_translation_noise_);
    get_parameter("graph.loop_rotation_noise", loop_rotation_noise_);
    get_parameter("graph.loop_translation_noise", loop_translation_noise_);

    LoopClosureConfig lc;
    get_parameter("loop.enable", lc.enable);
    get_parameter("loop.enable_quatro", lc.enable_quatro);
    get_parameter("loop.enable_submap_matching", lc.enable_submap_matching);
    get_parameter("loop.num_submap_keyframes", lc.num_submap_keyframes);
    get_parameter("loop.voxel_resolution", lc.voxel_resolution);
    get_parameter("loop.scancontext_max_correspondence_distance",
                  lc.scancontext_max_correspondence_distance);

    get_parameter("nano_gicp.thread_number", lc.nano_gicp.thread_number);
    get_parameter("nano_gicp.correspondences_number", lc.nano_gicp.correspondences_number);
    get_parameter("nano_gicp.max_iterations", lc.nano_gicp.max_iterations);
    get_parameter("nano_gicp.ransac_iterations", lc.nano_gicp.ransac_iterations);
    get_parameter("nano_gicp.max_correspondence_distance",
                  lc.nano_gicp.max_correspondence_distance);
    get_parameter("nano_gicp.fitness_score_threshold",
                  lc.nano_gicp.fitness_score_threshold);
    get_parameter("nano_gicp.transformation_epsilon",
                  lc.nano_gicp.transformation_epsilon);
    get_parameter("nano_gicp.euclidean_fitness_epsilon",
                  lc.nano_gicp.euclidean_fitness_epsilon);
    get_parameter("nano_gicp.ransac_outlier_rejection_threshold",
                  lc.nano_gicp.ransac_outlier_rejection_threshold);

    get_parameter("quatro.use_optimized_matching", lc.quatro.use_optimized_matching);
    get_parameter("quatro.estimate_scale", lc.quatro.estimate_scale);
    get_parameter("quatro.max_correspondences", lc.quatro.max_correspondences);
    get_parameter("quatro.max_iterations", lc.quatro.max_iterations);
    get_parameter("quatro.distance_threshold", lc.quatro.distance_threshold);
    get_parameter("quatro.fpfh_normal_radius", lc.quatro.fpfh_normal_radius);
    get_parameter("quatro.fpfh_radius", lc.quatro.fpfh_radius);
    get_parameter("quatro.noise_bound", lc.quatro.noise_bound);
    get_parameter("quatro.rotation_gnc_factor", lc.quatro.rotation_gnc_factor);
    get_parameter("quatro.rotation_cost_diff_threshold",
                  lc.quatro.rotation_cost_diff_threshold);

    loop_enabled_ = lc.enable;
    loop_closure_ = std::make_unique<LoopClosure>(lc);
  }

  void setupGraph() {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(params);

    prior_noise_ = makeDiagonalNoise(prior_rotation_noise_, prior_translation_noise_);
    odom_noise_ = makeDiagonalNoise(odom_rotation_noise_, odom_translation_noise_);
    loop_noise_ = makeDiagonalNoise(loop_rotation_noise_, loop_translation_noise_);
  }

  void setupRos() {
    global_odom_pub_ = create_publisher<OdomMsg>(global_odom_topic_, 20);
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(current_pose_topic_, 20);
    corrected_path_pub_ = create_publisher<nav_msgs::msg::Path>(corrected_path_topic_, 10);
    corrected_cloud_pub_ = create_publisher<CloudMsg>(corrected_current_cloud_topic_, 10);
    localized_pub_ = create_publisher<std_msgs::msg::Bool>("/r2/localized", 10);
    loop_score_pub_ = create_publisher<std_msgs::msg::Float64>("/r2/sam/loop_score", 10);

    rclcpp::QoS qos(static_cast<size_t>(sync_queue_size_));
    if (sync_qos_reliability_ == "best_effort" || sync_qos_reliability_ == "sensor_data") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    qos.durability_volatile();
    RCLCPP_INFO(get_logger(), "SAM sync QoS: reliability=%s queue=%d",
                sync_qos_reliability_.c_str(), sync_queue_size_);
    odom_sub_.subscribe(this, odom_topic_, qos.get_rmw_qos_profile());
    cloud_sub_.subscribe(this, cloud_topic_, qos.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(static_cast<uint32_t>(sync_queue_size_)), odom_sub_, cloud_sub_);
    sync_->registerCallback(std::bind(&FastLioSamScQnNode::odomCloudCallback,
                                      this, std::placeholders::_1, std::placeholders::_2));
  }

  static gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise(
      double rotation_var, double translation_var) {
    const gtsam::Vector variances =
        (gtsam::Vector(6) << rotation_var, rotation_var, rotation_var,
                             translation_var, translation_var, translation_var)
            .finished();
    return gtsam::noiseModel::Diagonal::Variances(variances);
  }

  void odomCloudCallback(const OdomMsg::ConstSharedPtr odom_msg,
                         const CloudMsg::ConstSharedPtr cloud_msg) {
    const double dt = std::abs(
        (rclcpp::Time(odom_msg->header.stamp) - rclcpp::Time(cloud_msg->header.stamp)).seconds());
    if (dt > max_stamp_diff_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "ApproximateTime matched a loose pair: dt=%.4f s", dt);
    }

    PoseCloud frame(*odom_msg, *cloud_msg, next_keyframe_index_);
    if (static_cast<int>(frame.cloud_local.size()) < min_keyframe_points_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skip SAM frame: too few points (%zu)", frame.cloud_local.size());
      return;
    }

    if (!initialized_) {
      initialize(frame, *odom_msg);
      return;
    }

    const Eigen::Matrix4d relative = last_raw_pose_.inverse() * frame.pose_raw;
    odom_delta_ = odom_delta_ * relative;
    frame.pose_corrected = last_corrected_pose_ * odom_delta_;
    current_frame_ = frame;
    last_raw_pose_ = frame.pose_raw;

    publishGlobalOdometry(frame.pose_corrected, *odom_msg);
    publishCorrectedCloud(frame);

    if (isKeyframe(frame, keyframes_.back())) {
      addKeyframe(frame, *odom_msg);
    }
  }

  void initialize(PoseCloud &frame, const OdomMsg &odom_msg) {
    initialized_ = true;
    last_raw_pose_ = frame.pose_raw;
    last_corrected_pose_ = frame.pose_corrected;
    odom_delta_ = Eigen::Matrix4d::Identity();
    current_frame_ = frame;

    keyframes_.push_back(frame);
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, matrixToGtsamPose(frame.pose_raw), prior_noise_));
    initial_values_.insert(0, matrixToGtsamPose(frame.pose_raw));
    isam_->update(graph_, initial_values_);
    isam_->update();
    graph_.resize(0);
    initial_values_.clear();
    corrected_estimate_ = isam_->calculateEstimate();

    loop_closure_->updateScanContext(frame.cloud_local);
    updatePath();
    publishGlobalOdometry(frame.pose_corrected, odom_msg);
    publishLocalized(true);
    ++next_keyframe_index_;

    RCLCPP_INFO(get_logger(), "SAM initialized with first keyframe: points=%zu",
                frame.cloud_local.size());
  }

  bool isKeyframe(const PoseCloud &frame, const PoseCloud &last_keyframe) const {
    const double distance =
        (frame.pose_corrected.block<3, 1>(0, 3) -
         last_keyframe.pose_corrected.block<3, 1>(0, 3))
            .norm();
    return distance >= keyframe_distance_threshold_;
  }

  void addKeyframe(PoseCloud frame, const OdomMsg &odom_msg) {
    frame.index = next_keyframe_index_;
    keyframes_.push_back(frame);

    const int from = next_keyframe_index_ - 1;
    const int to = next_keyframe_index_;
    const gtsam::Pose3 pose_from = matrixToGtsamPose(keyframes_[from].pose_corrected);
    const gtsam::Pose3 pose_to = matrixToGtsamPose(frame.pose_corrected);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from, to,
                                                  pose_from.between(pose_to),
                                                  odom_noise_));
    initial_values_.insert(to, pose_to);

    loop_closure_->updateScanContext(frame.cloud_local);
    bool loop_added = false;
    if (loop_enabled_ && static_cast<int>(keyframes_.size()) >= min_loop_keyframes_) {
      loop_added = tryAddLoopFactor(frame);
    }

    isam_->update(graph_, initial_values_);
    if (loop_added) {
      // 闭环因子会明显改变历史状态，多做几次 update 让 iSAM2 收敛得更稳。
      isam_->update();
      isam_->update();
      isam_->update();
    }
    graph_.resize(0);
    initial_values_.clear();

    corrected_estimate_ = isam_->calculateEstimate();
    updateCorrectedKeyframes();
    last_corrected_pose_ = keyframes_.back().pose_corrected;
    odom_delta_ = Eigen::Matrix4d::Identity();
    current_frame_ = keyframes_.back();
    updatePath();

    publishGlobalOdometry(last_corrected_pose_, odom_msg);
    publishCorrectedCloud(current_frame_);
    ++next_keyframe_index_;

    RCLCPP_INFO(get_logger(),
                "SAM keyframe %d accepted: total=%zu loop_added=%d",
                to, keyframes_.size(), loop_added ? 1 : 0);
  }

  bool tryAddLoopFactor(const PoseCloud &frame) {
    const int candidate = loop_closure_->fetchCandidateKeyframeIndex(frame, keyframes_);
    if (candidate < 0) {
      return false;
    }

    const RegistrationResult result = loop_closure_->perform(frame, keyframes_, candidate);
    std_msgs::msg::Float64 score_msg;
    score_msg.data = result.score;
    loop_score_pub_->publish(score_msg);

    if (!result.valid) {
      RCLCPP_WARN(get_logger(),
                  "Loop candidate rejected: query=%d candidate=%d converged=%d score=%.4f",
                  frame.index, candidate, result.converged ? 1 : 0, result.score);
      return false;
    }

    const gtsam::Pose3 pose_from = matrixToGtsamPose(result.transform * frame.pose_corrected);
    const gtsam::Pose3 pose_to = matrixToGtsamPose(keyframes_[candidate].pose_corrected);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(frame.index, candidate,
                                                  pose_from.between(pose_to),
                                                  loop_noise_));
    RCLCPP_INFO(get_logger(),
                "Loop factor added: query=%d candidate=%d score=%.4f",
                frame.index, candidate, result.score);
    return true;
  }

  void updateCorrectedKeyframes() {
    for (size_t i = 0; i < keyframes_.size(); ++i) {
      if (corrected_estimate_.exists(static_cast<gtsam::Key>(i))) {
        keyframes_[i].pose_corrected =
            gtsamPoseToMatrix(corrected_estimate_.at<gtsam::Pose3>(i));
      }
    }
  }

  void updatePath() {
    corrected_path_.header.frame_id = map_frame_;
    corrected_path_.header.stamp = now();
    corrected_path_.poses.clear();
    corrected_path_.poses.reserve(keyframes_.size());
    for (const auto &kf : keyframes_) {
      corrected_path_.poses.push_back(
          matrixToPoseStamped(kf.pose_corrected, map_frame_, kf.stamp));
    }
    corrected_path_pub_->publish(corrected_path_);
  }

  void publishGlobalOdometry(const Eigen::Matrix4d &pose, const OdomMsg &source_odom) {
    OdomMsg odom = source_odom;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose = matrixToPoseMsg(pose);
    global_odom_pub_->publish(odom);

    auto pose_msg = matrixToPoseStamped(pose, map_frame_, source_odom.header.stamp);
    current_pose_pub_->publish(pose_msg);
  }

  void publishCorrectedCloud(const PoseCloud &frame) {
    if (corrected_cloud_pub_->get_subscription_count() == 0) {
      return;
    }
    const Cloud corrected_cloud = transformCloud(frame.cloud_local, frame.pose_corrected);
    CloudMsg msg;
    pcl::toROSMsg(corrected_cloud, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = frame.stamp;
    corrected_cloud_pub_->publish(msg);
  }

  void publishLocalized(bool localized) {
    std_msgs::msg::Bool msg;
    msg.data = localized;
    localized_pub_->publish(msg);
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string global_odom_topic_;
  std::string current_pose_topic_;
  std::string corrected_path_topic_;
  std::string corrected_current_cloud_topic_;
  std::string map_frame_;
  std::string child_frame_;
  int sync_queue_size_ = 30;
  double max_stamp_diff_sec_ = 0.03;
  std::string sync_qos_reliability_ = "reliable";
  double keyframe_distance_threshold_ = 1.5;
  int min_keyframe_points_ = 80;
  int min_loop_keyframes_ = 35;
  bool loop_enabled_ = true;

  double prior_rotation_noise_ = 1e-4;
  double prior_translation_noise_ = 1e-2;
  double odom_rotation_noise_ = 1e-4;
  double odom_translation_noise_ = 1e-2;
  double loop_rotation_noise_ = 1e-3;
  double loop_translation_noise_ = 1e-2;

  message_filters::Subscriber<OdomMsg> odom_sub_;
  message_filters::Subscriber<CloudMsg> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<OdomMsg>::SharedPtr global_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr corrected_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localized_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr loop_score_pub_;

  std::unique_ptr<LoopClosure> loop_closure_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;
  gtsam::Values corrected_estimate_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr loop_noise_;

  bool initialized_ = false;
  int next_keyframe_index_ = 0;
  Eigen::Matrix4d last_raw_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
  PoseCloud current_frame_;
  std::vector<PoseCloud> keyframes_;
  nav_msgs::msg::Path corrected_path_;
};

}  // namespace fast_lio_sam_sc_qn_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fast_lio_sam_sc_qn_ros2::FastLioSamScQnNode>());
  rclcpp::shutdown();
  return 0;
}
