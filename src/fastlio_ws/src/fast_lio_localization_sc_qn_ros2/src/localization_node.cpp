#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

#include <pcl/io/pcd_io.h>

#include "fast_lio_localization_sc_qn_ros2/map_matcher.hpp"
#include "fast_lio_localization_sc_qn_ros2/pose_cloud.hpp"
#include "fast_lio_localization_sc_qn_ros2/utilities.hpp"

namespace fast_lio_localization_sc_qn_ros2 {

class FastLioLocalizationScQnNode : public rclcpp::Node {
public:
  using OdomMsg = nav_msgs::msg::Odometry;
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<OdomMsg, CloudMsg>;

  FastLioLocalizationScQnNode() : Node("fast_lio_localization_sc_qn") {
    declareParameters();
    readParameters();
    map_matcher_ = std::make_unique<MapMatcher>(matcher_config_);
    loadMap(map_directory_);
    setupRos();

    RCLCPP_INFO(get_logger(),
                "FAST-LIO Localization SC-QN ready: odom=%s cloud=%s map=%s output=%s",
                odom_topic_.c_str(), cloud_topic_.c_str(), map_directory_.c_str(),
                global_odom_topic_.c_str());
  }

private:
  void declareParameters() {
    declare_parameter<std::string>("topics.odom", "/Odometry");
    declare_parameter<std::string>("topics.cloud", "/cloud_registered");
    declare_parameter<std::string>("topics.global_odom", "/r2/global_odometry");
    declare_parameter<std::string>("topics.current_pose", "/r2_current_pose");
    declare_parameter<std::string>("topics.saved_map", "/r2/localization/saved_map");
    declare_parameter<std::string>("topics.corrected_cloud", "/r2/localization/corrected_cloud");
    declare_parameter<std::string>("topics.debug_source", "/r2/localization/src");
    declare_parameter<std::string>("topics.debug_target", "/r2/localization/dst");
    declare_parameter<std::string>("topics.debug_coarse", "/r2/localization/coarse_aligned");
    declare_parameter<std::string>("topics.debug_final", "/r2/localization/final_aligned");
    declare_parameter<std::string>("topics.match_marker", "/r2/localization/matches");
    declare_parameter<std::string>("frames.map", "map");
    declare_parameter<std::string>("frames.child", "body");
    declare_parameter<std::string>("map.directory", "");
    declare_parameter<double>("map.visualize_voxel_size", 1.0);

    declare_parameter<int>("sync.queue_size", 30);
    declare_parameter<double>("sync.max_stamp_diff_sec", 0.03);
    declare_parameter<std::string>("sync.qos_reliability", "reliable");
    declare_parameter<double>("match.timer_hz", 1.0);
    declare_parameter<int>("cold_start.accumulation_frames", 10);
    declare_parameter<int>("cold_start.max_attempts", 3);
    declare_parameter<double>("keyframe.distance_threshold", 1.0);
    declare_parameter<int>("keyframe.min_points", 80);

    declare_parameter<bool>("match.enable_quatro", true);
    declare_parameter<bool>("match.enable_distance_gate", false);
    declare_parameter<int>("match.num_submap_keyframes", 10);
    declare_parameter<double>("match.voxel_resolution", 0.10);
    declare_parameter<double>("match.scancontext_max_correspondence_distance", 30.0);

    declare_parameter<int>("nano_gicp.thread_number", 0);
    declare_parameter<int>("nano_gicp.correspondences_number", 15);
    declare_parameter<int>("nano_gicp.max_iterations", 32);
    declare_parameter<int>("nano_gicp.ransac_iterations", 5);
    declare_parameter<double>("nano_gicp.max_correspondence_distance", 2.0);
    declare_parameter<double>("nano_gicp.fitness_score_threshold", 1.0);
    declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
    declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
    declare_parameter<double>("nano_gicp.ransac_outlier_rejection_threshold", 1.0);

    declare_parameter<bool>("quatro.use_optimized_matching", true);
    declare_parameter<bool>("quatro.estimate_scale", false);
    declare_parameter<int>("quatro.max_correspondences", 500);
    declare_parameter<int>("quatro.max_iterations", 50);
    declare_parameter<double>("quatro.distance_threshold", 30.0);
    declare_parameter<double>("quatro.fpfh_normal_radius", 0.30);
    declare_parameter<double>("quatro.fpfh_radius", 0.50);
    declare_parameter<double>("quatro.noise_bound", 0.30);
    declare_parameter<double>("quatro.rotation_gnc_factor", 1.40);
    declare_parameter<double>("quatro.rotation_cost_diff_threshold", 0.0001);
  }

  void readParameters() {
    get_parameter("topics.odom", odom_topic_);
    get_parameter("topics.cloud", cloud_topic_);
    get_parameter("topics.global_odom", global_odom_topic_);
    get_parameter("topics.current_pose", current_pose_topic_);
    get_parameter("topics.saved_map", saved_map_topic_);
    get_parameter("topics.corrected_cloud", corrected_cloud_topic_);
    get_parameter("topics.debug_source", debug_source_topic_);
    get_parameter("topics.debug_target", debug_target_topic_);
    get_parameter("topics.debug_coarse", debug_coarse_topic_);
    get_parameter("topics.debug_final", debug_final_topic_);
    get_parameter("topics.match_marker", match_marker_topic_);
    get_parameter("frames.map", map_frame_);
    get_parameter("frames.child", child_frame_);
    get_parameter("map.directory", map_directory_);
    get_parameter("map.visualize_voxel_size", map_visualize_voxel_size_);
    get_parameter("sync.queue_size", sync_queue_size_);
    get_parameter("sync.max_stamp_diff_sec", max_stamp_diff_sec_);
    get_parameter("sync.qos_reliability", sync_qos_reliability_);
    get_parameter("match.timer_hz", match_timer_hz_);
    get_parameter("cold_start.accumulation_frames", cold_start_accumulation_frames_);
    get_parameter("cold_start.max_attempts", cold_start_max_attempts_);
    cold_start_accumulation_frames_ = std::max(1, cold_start_accumulation_frames_);
    cold_start_max_attempts_ = std::max(1, cold_start_max_attempts_);
    get_parameter("keyframe.distance_threshold", keyframe_distance_threshold_);
    get_parameter("keyframe.min_points", min_keyframe_points_);

    get_parameter("match.enable_quatro", matcher_config_.enable_quatro);
    get_parameter("match.enable_distance_gate", matcher_config_.enable_distance_gate);
    get_parameter("match.num_submap_keyframes", matcher_config_.num_submap_keyframes);
    get_parameter("match.voxel_resolution", matcher_config_.voxel_resolution);
    get_parameter("match.scancontext_max_correspondence_distance",
                  matcher_config_.scancontext_max_correspondence_distance);

    get_parameter("nano_gicp.thread_number", matcher_config_.gicp.thread_number);
    get_parameter("nano_gicp.correspondences_number", matcher_config_.gicp.correspondences_number);
    get_parameter("nano_gicp.max_iterations", matcher_config_.gicp.max_iterations);
    get_parameter("nano_gicp.ransac_iterations", matcher_config_.gicp.ransac_iterations);
    get_parameter("nano_gicp.max_correspondence_distance",
                  matcher_config_.gicp.max_correspondence_distance);
    get_parameter("nano_gicp.fitness_score_threshold",
                  matcher_config_.gicp.fitness_score_threshold);
    get_parameter("nano_gicp.transformation_epsilon",
                  matcher_config_.gicp.transformation_epsilon);
    get_parameter("nano_gicp.euclidean_fitness_epsilon",
                  matcher_config_.gicp.euclidean_fitness_epsilon);
    get_parameter("nano_gicp.ransac_outlier_rejection_threshold",
                  matcher_config_.gicp.ransac_outlier_rejection_threshold);

    get_parameter("quatro.use_optimized_matching", matcher_config_.quatro.use_optimized_matching);
    get_parameter("quatro.estimate_scale", matcher_config_.quatro.estimate_scale);
    get_parameter("quatro.max_correspondences", matcher_config_.quatro.max_correspondences);
    get_parameter("quatro.max_iterations", matcher_config_.quatro.max_iterations);
    get_parameter("quatro.distance_threshold", matcher_config_.quatro.distance_threshold);
    get_parameter("quatro.fpfh_normal_radius", matcher_config_.quatro.fpfh_normal_radius);
    get_parameter("quatro.fpfh_radius", matcher_config_.quatro.fpfh_radius);
    get_parameter("quatro.noise_bound", matcher_config_.quatro.noise_bound);
    get_parameter("quatro.rotation_gnc_factor", matcher_config_.quatro.rotation_gnc_factor);
    get_parameter("quatro.rotation_cost_diff_threshold",
                  matcher_config_.quatro.rotation_cost_diff_threshold);
  }

  void setupRos() {
    global_odom_pub_ = create_publisher<OdomMsg>(global_odom_topic_, 20);
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(current_pose_topic_, 20);
    saved_map_pub_ = create_publisher<CloudMsg>(saved_map_topic_, rclcpp::QoS(1).transient_local().reliable());
    corrected_cloud_pub_ = create_publisher<CloudMsg>(corrected_cloud_topic_, 10);
    debug_source_pub_ = create_publisher<CloudMsg>(debug_source_topic_, 10);
    debug_target_pub_ = create_publisher<CloudMsg>(debug_target_topic_, 10);
    debug_coarse_pub_ = create_publisher<CloudMsg>(debug_coarse_topic_, 10);
    debug_final_pub_ = create_publisher<CloudMsg>(debug_final_topic_, 10);
    match_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(match_marker_topic_, 10);
    localized_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/r2/localized", rclcpp::QoS(1).transient_local().reliable());
    fitness_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/r2/fitness_score", rclcpp::QoS(1).transient_local().reliable());

    rclcpp::QoS qos(static_cast<size_t>(sync_queue_size_));
    if (sync_qos_reliability_ == "best_effort" || sync_qos_reliability_ == "sensor_data") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    qos.durability_volatile();
    odom_sub_.subscribe(this, odom_topic_, qos.get_rmw_qos_profile());
    cloud_sub_.subscribe(this, cloud_topic_, qos.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(static_cast<uint32_t>(sync_queue_size_)), odom_sub_, cloud_sub_);
    sync_->registerCallback(std::bind(&FastLioLocalizationScQnNode::odomCloudCallback,
                                      this, std::placeholders::_1, std::placeholders::_2));

    const double hz = std::max(0.1, match_timer_hz_);
    match_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz)),
        std::bind(&FastLioLocalizationScQnNode::matchingTimerCallback, this));

    publishSavedMap();
  }

  static std::vector<std::string> splitCsvLine(const std::string &line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
      fields.push_back(item);
    }
    return fields;
  }

  void loadMap(const std::string &directory) {
    if (directory.empty()) {
      throw std::runtime_error("map.directory is empty; set it to an r2_sam_keyframe_map_v1 directory");
    }

    const std::filesystem::path root(directory);
    const std::filesystem::path poses_path = root / "poses.csv";
    const std::filesystem::path keyframes_dir = root / "keyframes";
    if (!std::filesystem::exists(poses_path) || !std::filesystem::is_directory(keyframes_dir)) {
      throw std::runtime_error(
          "Map directory must contain poses.csv and keyframes/: " + root.string());
    }

    std::ifstream poses(poses_path);
    if (!poses.is_open()) {
      throw std::runtime_error("failed to open " + poses_path.string());
    }

    map_keyframes_.clear();
    saved_map_cloud_.clear();

    std::string line;
    bool first_line = true;
    while (std::getline(poses, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      if (first_line) {
        first_line = false;
        if (line.find("index,") == 0) {
          continue;
        }
      }

      const auto fields = splitCsvLine(line);
      if (fields.size() != 20) {
        throw std::runtime_error("invalid poses.csv row, expected 20 fields: " + line);
      }

      MapKeyframe keyframe;
      keyframe.index = std::stoi(fields[0]);
      const std::filesystem::path pcd_path = keyframes_dir / fields[3];
      if (pcl::io::loadPCDFile<PointType>(pcd_path.string(), keyframe.cloud_local) != 0) {
        throw std::runtime_error("failed to load keyframe pcd: " + pcd_path.string());
      }

      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      size_t k = 4;
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          pose(r, c) = std::stod(fields[k++]);
        }
      }
      keyframe.pose = pose;

      saved_map_cloud_ += transformCloud(keyframe.cloud_local, keyframe.pose);
      map_matcher_->addMapScanContext(keyframe.cloud_local);
      map_keyframes_.push_back(std::move(keyframe));
    }

    if (map_keyframes_.empty()) {
      throw std::runtime_error("map directory contains no keyframes: " + root.string());
    }
    saved_map_cloud_ = *voxelizeCloud(saved_map_cloud_, map_visualize_voxel_size_);
    RCLCPP_INFO(get_logger(), "Loaded localization keyframe map: directory=%s keyframes=%zu map_points=%zu",
                root.string().c_str(), map_keyframes_.size(), saved_map_cloud_.size());
  }

  void odomCloudCallback(const OdomMsg::ConstSharedPtr odom_msg,
                         const CloudMsg::ConstSharedPtr cloud_msg) {
    const double dt = std::abs(
        (rclcpp::Time(odom_msg->header.stamp) - rclcpp::Time(cloud_msg->header.stamp)).seconds());
    if (dt > max_stamp_diff_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "ApproximateTime matched a loose pair: dt=%.4f s", dt);
    }

    PoseCloud current(*odom_msg, *cloud_msg, current_keyframe_index_);
    if (static_cast<int>(current.cloud_local.size()) < min_keyframe_points_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skip localization frame: too few points (%zu)", current.cloud_local.size());
      return;
    }

    current_frame_ = current;
    if (!is_localized_) {
      accumulateColdStartFrame(current);
      publishLocalized(false);
      return;
    }

    // 首次全局对齐成功后，固定使用 map<-odom 修正每一帧 FAST-LIO 位姿。
    current.pose_corrected = map_from_odom_ * current.pose_raw;
    current_frame_ = current;
    publishGlobalOdometry(current.pose_corrected, *odom_msg);
    publishCorrectedCloud(current);

    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    if (isKeyframe(current, last_keyframe_)) {
      current.index = current_keyframe_index_++;
      current.processed = false;
      last_keyframe_ = current;
    }
  }

  void accumulateColdStartFrame(const PoseCloud &current) {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    if (cold_start_attempts_ >= cold_start_max_attempts_ ||
        cold_start_waiting_for_match_) {
      return;
    }

    // Quatro 要求 scan-to-scan 点云规模相近。等待一个静态窗口稳定前端，
    // 但不叠加重复扫描；选择窗口内点数最丰富的一帧作为冷启动查询。
    if (cold_start_frame_count_ == 0 ||
        current.cloud_local.size() > cold_start_accumulator_.cloud_local.size()) {
      cold_start_accumulator_ = current;
    }
    ++cold_start_frame_count_;

    if (cold_start_frame_count_ < cold_start_accumulation_frames_) {
      return;
    }

    cold_start_accumulator_.index = current_keyframe_index_++;
    cold_start_accumulator_.processed = false;
    cold_start_accumulator_.pose_corrected = cold_start_accumulator_.pose_raw;
    pending_cold_start_match_ = cold_start_accumulator_;
    cold_start_match_ready_ = true;
    cold_start_waiting_for_match_ = true;
    RCLCPP_INFO(get_logger(),
                "Cold-start accumulation ready: attempt=%d/%d frames=%d points=%zu",
                cold_start_attempts_ + 1, cold_start_max_attempts_,
                cold_start_frame_count_, pending_cold_start_match_.cloud_local.size());
  }

  bool isKeyframe(const PoseCloud &frame, const PoseCloud &last) const {
    return keyframe_distance_threshold_ <
           (last.pose_corrected.block<3, 1>(0, 3) -
            frame.pose_corrected.block<3, 1>(0, 3)).norm();
  }

  void matchingTimerCallback() {
    PoseCloud keyframe;
    bool cold_start_match = false;
    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      if (!is_localized_) {
        if (!cold_start_match_ready_) {
          return;
        }
        keyframe = pending_cold_start_match_;
        cold_start_match_ready_ = false;
        cold_start_match = true;
      } else {
        keyframe = last_keyframe_;
        if (keyframe.processed) {
          return;
        }
        last_keyframe_.processed = true;
      }
    }

    runGlobalMatch(keyframe, cold_start_match);
  }

  void runGlobalMatch(PoseCloud keyframe, bool cold_start_match) {
    const int candidate = map_matcher_->fetchClosestKeyframeIndex(keyframe, map_keyframes_);
    if (candidate < 0) {
      RCLCPP_WARN(get_logger(),
                  "%s: no ScanContext map candidate for keyframe %d",
                  cold_start_match ? "Cold-start match rejected" : "Map match rejected",
                  keyframe.index);
      publishLocalized(false);
      if (cold_start_match) {
        finishColdStartFailure();
      }
      return;
    }

    const RegistrationOutput result = map_matcher_->perform(keyframe, map_keyframes_, candidate);
    std_msgs::msg::Float64 score_msg;
    score_msg.data = result.score;
    fitness_pub_->publish(score_msg);
    publishDebugClouds(keyframe.stamp);

    if (!result.valid) {
      RCLCPP_WARN(get_logger(),
                  "%s: keyframe=%d candidate=%d converged=%d score=%.4f threshold=%.4f",
                  cold_start_match ? "Cold-start match rejected" : "Map match rejected",
                  keyframe.index, candidate, result.converged ? 1 : 0, result.score,
                  matcher_config_.gicp.fitness_score_threshold);
      publishLocalized(false);
      if (cold_start_match) {
        finishColdStartFailure();
      }
      return;
    }

    // result.transform 把当前修正系中的查询点云变换到 map；与已有 map<-odom 左乘组合。
    map_from_odom_ = result.transform * map_from_odom_;
    keyframe.pose_corrected = map_from_odom_ * keyframe.pose_raw;
    keyframe.processed = true;
    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      last_keyframe_ = keyframe;
      is_localized_ = true;
      resetColdStartAccumulatorLocked();
    }

    publishGlobalOdometry(keyframe.pose_corrected, keyframe.stamp);
    publishLocalized(true);
    publishMatchMarker(keyframe.pose_corrected, keyframe.pose_raw, keyframe.stamp);

    RCLCPP_INFO(get_logger(),
                "%s: keyframe=%d candidate=%d score=%.4f; map<-odom locked",
                cold_start_match ? "Cold-start match accepted" : "Map match accepted",
                keyframe.index, candidate, result.score);
  }

  void finishColdStartFailure() {
    int attempts = 0;
    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      ++cold_start_attempts_;
      attempts = cold_start_attempts_;
      resetColdStartAccumulatorLocked();
    }

    if (attempts >= cold_start_max_attempts_) {
      RCLCPP_ERROR(get_logger(),
                   "Cold-start localization failed after %d attempts; no global odometry will be published",
                   attempts);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Cold-start localization will retry with a fresh %d-frame accumulation window (%d/%d)",
                  cold_start_accumulation_frames_, attempts, cold_start_max_attempts_);
    }
  }

  void resetColdStartAccumulatorLocked() {
    cold_start_accumulator_ = PoseCloud{};
    pending_cold_start_match_ = PoseCloud{};
    cold_start_frame_count_ = 0;
    cold_start_match_ready_ = false;
    cold_start_waiting_for_match_ = false;
  }

  void publishGlobalOdometry(const Eigen::Matrix4d &pose, const OdomMsg &source_odom) {
    OdomMsg odom = source_odom;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose = matrixToPoseMsg(pose);
    global_odom_pub_->publish(odom);
    current_pose_pub_->publish(matrixToPoseStamped(pose, map_frame_, source_odom.header.stamp));
  }

  void publishGlobalOdometry(const Eigen::Matrix4d &pose, const rclcpp::Time &stamp) {
    OdomMsg odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose = matrixToPoseMsg(pose);
    global_odom_pub_->publish(odom);
    current_pose_pub_->publish(matrixToPoseStamped(pose, map_frame_, stamp));
  }

  void publishCorrectedCloud(const PoseCloud &frame) {
    if (corrected_cloud_pub_->get_subscription_count() == 0) {
      return;
    }
    corrected_cloud_pub_->publish(
        cloudToRosMsg(transformCloud(frame.cloud_local, frame.pose_corrected), map_frame_, frame.stamp));
  }

  void publishSavedMap() {
    saved_map_pub_->publish(cloudToRosMsg(saved_map_cloud_, map_frame_, now()));
  }

  void publishDebugClouds(const rclcpp::Time &stamp) {
    debug_source_pub_->publish(cloudToRosMsg(map_matcher_->sourceCloud(), map_frame_, stamp));
    debug_target_pub_->publish(cloudToRosMsg(map_matcher_->targetCloud(), map_frame_, stamp));
    debug_coarse_pub_->publish(cloudToRosMsg(map_matcher_->coarseAlignedCloud(), map_frame_, stamp));
    debug_final_pub_->publish(cloudToRosMsg(map_matcher_->finalAlignedCloud(), map_frame_, stamp));
  }

  void publishLocalized(bool localized) {
    std_msgs::msg::Bool msg;
    msg.data = localized;
    localized_pub_->publish(msg);
  }

  void publishMatchMarker(const Eigen::Matrix4d &corrected_pose,
                          const Eigen::Matrix4d &raw_pose,
                          const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = stamp;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    geometry_msgs::msg::Point a;
    a.x = corrected_pose(0, 3);
    a.y = corrected_pose(1, 3);
    a.z = corrected_pose(2, 3);
    geometry_msgs::msg::Point b;
    b.x = raw_pose(0, 3);
    b.y = raw_pose(1, 3);
    b.z = raw_pose(2, 3);
    marker.points.push_back(a);
    marker.points.push_back(b);
    match_marker_pub_->publish(marker);
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string global_odom_topic_;
  std::string current_pose_topic_;
  std::string saved_map_topic_;
  std::string corrected_cloud_topic_;
  std::string debug_source_topic_;
  std::string debug_target_topic_;
  std::string debug_coarse_topic_;
  std::string debug_final_topic_;
  std::string match_marker_topic_;
  std::string map_frame_;
  std::string child_frame_;
  std::string map_directory_;
  double map_visualize_voxel_size_ = 1.0;
  int sync_queue_size_ = 30;
  double max_stamp_diff_sec_ = 0.03;
  std::string sync_qos_reliability_ = "reliable";
  double match_timer_hz_ = 1.0;
  int cold_start_accumulation_frames_ = 10;
  int cold_start_max_attempts_ = 3;
  double keyframe_distance_threshold_ = 1.0;
  int min_keyframe_points_ = 80;

  MapMatcherConfig matcher_config_;
  std::unique_ptr<MapMatcher> map_matcher_;
  std::vector<MapKeyframe> map_keyframes_;
  Cloud saved_map_cloud_;

  bool is_localized_ = false;
  int current_keyframe_index_ = 0;
  Eigen::Matrix4d map_from_odom_ = Eigen::Matrix4d::Identity();
  PoseCloud last_keyframe_;
  PoseCloud current_frame_;
  PoseCloud cold_start_accumulator_;
  PoseCloud pending_cold_start_match_;
  int cold_start_frame_count_ = 0;
  int cold_start_attempts_ = 0;
  bool cold_start_match_ready_ = false;
  bool cold_start_waiting_for_match_ = false;
  std::mutex keyframe_mutex_;

  message_filters::Subscriber<OdomMsg> odom_sub_;
  message_filters::Subscriber<CloudMsg> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::TimerBase::SharedPtr match_timer_;

  rclcpp::Publisher<OdomMsg>::SharedPtr global_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr saved_map_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr corrected_cloud_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr debug_source_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr debug_target_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr debug_coarse_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr debug_final_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr match_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localized_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_pub_;
};

}  // namespace fast_lio_localization_sc_qn_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fast_lio_localization_sc_qn_ros2::FastLioLocalizationScQnNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("fast_lio_localization_sc_qn"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
