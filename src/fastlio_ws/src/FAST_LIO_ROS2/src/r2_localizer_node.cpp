#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "FieldMapTransform.hpp"
#include "MapManager.hpp"
#include "NDTAligner.hpp"
#include "OdomSynchronizer.hpp"

// ROS2 顶层编排节点：处理参数、话题、TF 和冷启动状态，不承担底层点云算法。
class R2LocalizerNode : public rclcpp::Node {
public:
  using Cloud = r2_localizer::Cloud;
  using Matrix4f = r2_localizer::Matrix4f;
  using NDTAligner = r2_localizer::NDTAligner;
  using OdomSample = r2_localizer::OdomSample;

  R2LocalizerNode() : Node("r2_localizer"), initial_cloud_(new Cloud) {
    declareParameters();
    readParameters();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/r2_current_pose", 10);
    odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>("/r2/global_odometry", 10);
    registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/registered_scan", 10);
    localized_pub_ =
        create_publisher<std_msgs::msg::Bool>("/r2/localized", 10);
    fitness_pub_ =
        create_publisher<std_msgs::msg::Float64>("/r2/fitness_score", 10);
    pose_report_srv_ = create_service<std_srvs::srv::Trigger>(
        "/r2/report_pose",
        std::bind(&R2LocalizerNode::reportPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", map_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 三个核心对象通过组合注入，顶层节点只负责编排调用顺序。
    map_manager_ =
        std::make_unique<r2_localizer::MapManager>(map_path_, map_voxel_size_);
    odom_synchronizer_ = std::make_unique<r2_localizer::OdomSynchronizer>(
        odom_sync_tolerance_sec_);
    ndt_aligner_ =
        std::make_unique<NDTAligner>(makeAlignerConfig(), *map_manager_);
    openMetricsLog();
    initial_map_body_guess_ = makeInitialPose();

    // 点云配准较耗时；与里程计回调分组后，缓存更新不会被 NDT 长时间阻塞。
    cloud_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;
    initial_pose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_, rclcpp::SystemDefaultsQoS(),
            std::bind(&R2LocalizerNode::initialPoseCallback, this,
                      std::placeholders::_1),
            cloud_options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(100),
        std::bind(&R2LocalizerNode::odomCallback, this, std::placeholders::_1),
        odom_options);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&R2LocalizerNode::cloudCallback, this,
                  std::placeholders::_1),
        cloud_options);

    map_timer_ = create_wall_timer(
        std::chrono::duration<double>(map_publish_period_sec_),
        std::bind(&R2LocalizerNode::publishGlobalMap, this));
    publishGlobalMap();
    publishLocalized(false);

    RCLCPP_INFO(
        get_logger(),
        "R2 localizer ready: map=%s points=%zu, input=%s + %s, initial accumulation=%d frames",
        map_path_.c_str(), map_manager_->mapCloud()->size(), cloud_topic_.c_str(),
        odom_topic_.c_str(), initial_accumulation_frames_);
  }

private:
  void declareParameters() {
    declare_parameter<std::string>(
        "map_path", "/root/fastlio_ws/MapPCD/R2_Field_Map_v3.pcd");
    declare_parameter<std::string>("metrics_csv_path",
                                   "/tmp/r2_localizer_metrics.csv");
    declare_parameter<std::string>("topics.cloud", "/cloud_registered_body");
    declare_parameter<std::string>("topics.odom", "/Odometry");
    declare_parameter<std::string>("topics.initial_pose", "/initialpose");
    declare_parameter<std::string>("frames.map", "map");
    declare_parameter<std::string>("frames.field", "field");
    declare_parameter<std::string>("frames.odom", "camera_init");
    declare_parameter<std::string>("frames.body", "body");
    declare_parameter<std::vector<double>>("frames.field_to_map",
                                           std::vector<double>{});
    declare_parameter<bool>("outputs.use_field_frame", false);
    declare_parameter<double>("initial_pose.x", 0.0);
    declare_parameter<double>("initial_pose.y", 0.0);
    declare_parameter<double>("initial_pose.z", 0.0);
    declare_parameter<double>("initial_pose.roll", 0.0);
    declare_parameter<double>("initial_pose.pitch", 0.0);
    declare_parameter<double>("initial_pose.yaw", 0.0);
    declare_parameter<int>("initial_accumulation_frames", 30);
    declare_parameter<int>("initial_max_attempts", 1);
    declare_parameter<int>("initial_registration_interval_frames", 10);
    declare_parameter<int>("registration_interval_frames", 1);
    declare_parameter<int>("min_source_points", 100);
    declare_parameter<double>("odom_sync_tolerance_sec", 0.03);
    declare_parameter<double>("filters.source_voxel_size", 0.20);
    declare_parameter<double>("filters.map_voxel_size", 0.20);
    declare_parameter<double>("filters.map_overlap_padding", 0.50);
    declare_parameter<double>("filters.map_match_radius", 0.50);
    declare_parameter<double>("filters.min_range", 0.0);
    declare_parameter<double>("filters.max_range", 100.0);
    declare_parameter<double>("filters.min_z", -100.0);
    declare_parameter<double>("filters.max_z", 100.0);
    declare_parameter<double>("ndt.transformation_epsilon", 0.01);
    declare_parameter<double>("ndt.step_size", 0.10);
    declare_parameter<double>("ndt.resolution", 0.50);
    declare_parameter<int>("ndt.maximum_iterations", 35);
    declare_parameter<double>("acceptance.max_fitness_score", 0.15);
    declare_parameter<double>("acceptance.max_initial_fitness_score", 0.08);
    declare_parameter<double>("acceptance.min_overlap_ratio", 0.08);
    declare_parameter<double>("acceptance.min_initial_overlap_ratio", 0.10);
    declare_parameter<double>("acceptance.max_initial_translation", 3.0);
    declare_parameter<double>("acceptance.max_initial_yaw", 1.57);
    declare_parameter<double>("acceptance.max_initial_roll_pitch", 0.52);
    declare_parameter<double>("acceptance.max_correction_translation", 0.50);
    declare_parameter<double>("acceptance.max_correction_yaw", 0.35);
    declare_parameter<double>("acceptance.max_correction_roll_pitch", 0.10);
    declare_parameter<double>("map_publish_period_sec", 2.0);
    declare_parameter<double>("pose_report.window_sec", 3.0);
    declare_parameter<int>("pose_report.min_samples", 5);
  }

  void readParameters() {
    get_parameter("map_path", map_path_);
    get_parameter("metrics_csv_path", metrics_csv_path_);
    get_parameter("topics.cloud", cloud_topic_);
    get_parameter("topics.odom", odom_topic_);
    get_parameter("topics.initial_pose", initial_pose_topic_);
    get_parameter("frames.map", map_frame_);
    get_parameter("frames.field", field_frame_);
    get_parameter("frames.odom", odom_frame_);
    get_parameter("frames.body", body_frame_);
    std::vector<double> field_to_map_values;
    get_parameter("frames.field_to_map", field_to_map_values);
    field_map_transform_.setFieldToMap(field_to_map_values);
    get_parameter("outputs.use_field_frame", use_field_frame_);
    get_parameter("initial_accumulation_frames", initial_accumulation_frames_);
    get_parameter("initial_max_attempts", initial_max_attempts_);
    get_parameter("initial_registration_interval_frames",
                  initial_registration_interval_frames_);
    get_parameter("registration_interval_frames", registration_interval_frames_);
    get_parameter("min_source_points", min_source_points_);
    get_parameter("odom_sync_tolerance_sec", odom_sync_tolerance_sec_);
    get_parameter("filters.source_voxel_size", source_voxel_size_);
    get_parameter("filters.map_voxel_size", map_voxel_size_);
    get_parameter("filters.map_overlap_padding", map_overlap_padding_);
    get_parameter("filters.map_match_radius", map_match_radius_);
    get_parameter("filters.min_range", min_range_);
    get_parameter("filters.max_range", max_range_);
    get_parameter("filters.min_z", min_z_);
    get_parameter("filters.max_z", max_z_);
    get_parameter("ndt.transformation_epsilon", ndt_epsilon_);
    get_parameter("ndt.step_size", ndt_step_size_);
    get_parameter("ndt.resolution", ndt_resolution_);
    get_parameter("ndt.maximum_iterations", ndt_max_iterations_);
    get_parameter("acceptance.max_fitness_score", max_fitness_score_);
    get_parameter("acceptance.max_initial_fitness_score",
                  max_initial_fitness_score_);
    get_parameter("acceptance.min_overlap_ratio", min_overlap_ratio_);
    get_parameter("acceptance.min_initial_overlap_ratio",
                  min_initial_overlap_ratio_);
    get_parameter("acceptance.max_initial_translation", max_initial_translation_);
    get_parameter("acceptance.max_initial_yaw", max_initial_yaw_);
    get_parameter("acceptance.max_initial_roll_pitch",
                  max_initial_roll_pitch_);
    get_parameter("acceptance.max_correction_translation",
                  max_correction_translation_);
    get_parameter("acceptance.max_correction_yaw", max_correction_yaw_);
    get_parameter("acceptance.max_correction_roll_pitch",
                  max_correction_roll_pitch_);
    get_parameter("map_publish_period_sec", map_publish_period_sec_);
    get_parameter("pose_report.window_sec", pose_report_window_sec_);
    get_parameter("pose_report.min_samples", pose_report_min_samples_);
    pose_report_window_sec_ = std::max(0.1, pose_report_window_sec_);
    pose_report_min_samples_ = std::max(1, pose_report_min_samples_);
    initial_accumulation_frames_ = std::max(1, initial_accumulation_frames_);
    initial_max_attempts_ = std::max(1, initial_max_attempts_);
    initial_registration_interval_frames_ =
        std::max(1, initial_registration_interval_frames_);
    registration_interval_frames_ = std::max(1, registration_interval_frames_);
  }

  NDTAligner::Config makeAlignerConfig() const {
    NDTAligner::Config config;
    config.source_voxel_size = source_voxel_size_;
    config.map_overlap_padding = map_overlap_padding_;
    config.map_match_radius = map_match_radius_;
    config.min_range = min_range_;
    config.max_range = max_range_;
    config.min_z = min_z_;
    config.max_z = max_z_;
    config.transformation_epsilon = ndt_epsilon_;
    config.step_size = ndt_step_size_;
    config.resolution = ndt_resolution_;
    config.maximum_iterations = ndt_max_iterations_;
    return config;
  }

  void openMetricsLog() {
    if (metrics_csv_path_.empty()) {
      return;
    }
    metrics_stream_.open(metrics_csv_path_, std::ios::out | std::ios::trunc);
    if (!metrics_stream_) {
      RCLCPP_WARN(get_logger(), "Cannot open metrics CSV: %s",
                  metrics_csv_path_.c_str());
      return;
    }
    metrics_stream_ << "stamp_sec,input_points,overlap_points,overlap_ratio,converged,accepted,fitness,translation_delta,yaw_delta,roll_pitch_delta,initial\n";
  }

  Matrix4f makeInitialPose() const {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    get_parameter("initial_pose.x", x);
    get_parameter("initial_pose.y", y);
    get_parameter("initial_pose.z", z);
    get_parameter("initial_pose.roll", roll);
    get_parameter("initial_pose.pitch", pitch);
    get_parameter("initial_pose.yaw", yaw);
    Matrix4f pose = Matrix4f::Identity();
    pose.block<3, 3>(0, 0) =
        (Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(static_cast<float>(roll), Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    pose(0, 3) = static_cast<float>(x);
    pose(1, 3) = static_cast<float>(y);
    pose(2, 3) = static_cast<float>(z);
    return pose;
  }

  static Matrix4f poseToMatrix(const geometry_msgs::msg::Pose &pose) {
    Matrix4f matrix = Matrix4f::Identity();
    Eigen::Quaternionf q(static_cast<float>(pose.orientation.w),
                         static_cast<float>(pose.orientation.x),
                         static_cast<float>(pose.orientation.y),
                         static_cast<float>(pose.orientation.z));
    if (q.norm() < 1e-6f) {
      q = Eigen::Quaternionf::Identity();
    } else {
      q.normalize();
    }
    matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
    matrix(0, 3) = static_cast<float>(pose.position.x);
    matrix(1, 3) = static_cast<float>(pose.position.y);
    matrix(2, 3) = static_cast<float>(pose.position.z);
    return matrix;
  }

  static geometry_msgs::msg::Pose matrixToPose(const Matrix4f &matrix) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
    q.normalize();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  // RViz 的 /initialpose 仅作为调试覆盖入口：清空旧校正并重新执行静止累计。
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    initial_map_body_guess_ = poseToMatrix(msg->pose.pose);
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      have_correction_ = false;
    }
    match_valid_ = false;
    initial_cloud_->clear();
    initial_frame_count_ = 0;
    initial_attempt_count_ = 0;
    registration_frame_count_ = 0;
    publishLocalized(false);
    RCLCPP_INFO(get_logger(),
                "Initial pose override received; restarting static accumulation.");
  }

  // FAST-LIO 局部里程计持续进入缓存；已有全局校正时同步发布全局 body 位姿。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    OdomSample sample{rclcpp::Time(msg->header.stamp),
                      poseToMatrix(msg->pose.pose), *msg};
    odom_synchronizer_->push(sample);
    if (hasCorrection()) {
      publishGlobalPose(sample);
    }
  }

  // /cloud_registered_body 已由 FAST-LIO 去畸变，配准前先寻找同一时刻的局部里程计。
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    OdomSample sample;
    if (!odom_synchronizer_->findMatchingOdom(rclcpp::Time(msg->header.stamp),
                                               sample)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for synchronized FAST-LIO odometry for body cloud.");
      return;
    }

    Cloud::Ptr body_cloud(new Cloud);
    pcl::fromROSMsg(*msg, *body_cloud);
    Cloud::Ptr filtered_body = ndt_aligner_->filterSource(body_cloud);
    if (filtered_body->size() < static_cast<std::size_t>(min_source_points_)) {
      return;
    }

    Cloud::Ptr odom_cloud(new Cloud);
    pcl::transformPointCloud(*filtered_body, *odom_cloud,
                             sample.camera_init_to_body);

    // 冷启动阶段累计多帧静止扫描，提高矮围栏和稀疏结构的可匹配点数。
    if (!hasCorrection()) {
      if (initial_attempt_count_ >= initial_max_attempts_) {
        return;
      }
      *initial_cloud_ += *odom_cloud;
      ++initial_frame_count_;
      const bool try_initial_match =
          initial_frame_count_ == initial_accumulation_frames_ ||
          (initial_frame_count_ > initial_accumulation_frames_ &&
           (initial_frame_count_ - initial_accumulation_frames_) %
                   initial_registration_interval_frames_ ==
               0);
      if (try_initial_match) {
        ++initial_attempt_count_;
        Cloud::Ptr accumulated = ndt_aligner_->voxelDownsample(
            initial_cloud_, source_voxel_size_);
        alignAndUpdate(accumulated, sample, true);
        if (!hasCorrection() &&
            initial_frame_count_ >= 2 * initial_accumulation_frames_) {
          initial_cloud_->clear();
          initial_frame_count_ = 0;
          RCLCPP_WARN(
              get_logger(),
              "Initial matching rejected; starting a fresh static accumulation window.");
        }
      }
      return;
    }

    // 首次定位成功后，连续位姿以 FAST-LIO 为主，NDT 只更新 map -> camera_init 校正。
    ++registration_frame_count_;
    if (registration_frame_count_ % registration_interval_frames_ == 0) {
      alignAndUpdate(odom_cloud, sample, false);
    }
  }

  void alignAndUpdate(const Cloud::Ptr &source, const OdomSample &sample,
                      bool initial) {
    if (source->size() < static_cast<std::size_t>(min_source_points_)) {
      return;
    }

    const Matrix4f guess =
        initial ? initial_map_body_guess_ * sample.camera_init_to_body.inverse()
                : map_to_camera_init_;
    const NDTAligner::Result result = ndt_aligner_->align(
        source, guess, static_cast<std::size_t>(min_source_points_));
    if (result.status == NDTAligner::Status::kInsufficientOverlapPoints) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Insufficient source points within predicted map overlap: %zu",
          result.overlap_points);
      publishLocalized(false);
      return;
    }
    if (result.status != NDTAligner::Status::kReady) {
      return;
    }

    // 接受门限放在顶层策略层：算法层只返回结果，不决定是否信任本次校正。
    const double translation_limit =
        initial ? max_initial_translation_ : max_correction_translation_;
    const double yaw_limit = initial ? max_initial_yaw_ : max_correction_yaw_;
    const double roll_pitch_limit =
        initial ? max_initial_roll_pitch_ : max_correction_roll_pitch_;
    const double fitness_limit =
        initial ? max_initial_fitness_score_ : max_fitness_score_;
    const double overlap_limit =
        initial ? min_initial_overlap_ratio_ : min_overlap_ratio_;
    const bool accepted =
        result.converged && std::isfinite(result.fitness) &&
        result.fitness <= fitness_limit &&
        result.overlap_ratio >= overlap_limit &&
        result.translation_delta <= translation_limit &&
        result.yaw_delta <= yaw_limit &&
        result.roll_pitch_delta <= roll_pitch_limit;

    publishFitness(result.fitness);
    publishLocalized(accepted);
    writeMetric(sample.stamp, result.input_points, result.overlap_points,
                result.overlap_ratio, result.converged, accepted,
                result.fitness, result.translation_delta, result.yaw_delta,
                result.roll_pitch_delta, initial);

    if (!accepted) {
      match_valid_ = false;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NDT rejected: converged=%d fitness=%.4f overlap=%.3f correction=%.3f m / yaw %.3f rad / roll-pitch %.3f rad",
          result.converged, result.fitness, result.overlap_ratio,
          result.translation_delta, result.yaw_delta, result.roll_pitch_delta);
      if (initial && initial_attempt_count_ >= initial_max_attempts_) {
        RCLCPP_ERROR(
            get_logger(),
            "Initial localization failed after %d attempt(s); waiting for /initialpose or restart.",
            initial_attempt_count_);
      }
      return;
    }

    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      map_to_camera_init_ = result.candidate;
      have_correction_ = true;
    }
    match_valid_ = true;
    initial_cloud_->clear();
    initial_frame_count_ = 0;
    publishRegisteredCloud(result.registered, sample.stamp);
    publishGlobalPose(sample);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NDT accepted: fitness=%.4f overlap=%.3f correction=%.3f m / yaw %.3f rad / roll-pitch %.3f rad",
        result.fitness, result.overlap_ratio, result.translation_delta,
        result.yaw_delta, result.roll_pitch_delta);
  }

  bool hasCorrection() const {
    std::lock_guard<std::mutex> lock(correction_mutex_);
    return have_correction_;
  }

  // NDT 和 TF 校正仍在 map 中运行；对外可选择发布水平 field 坐标。
  void publishGlobalPose(const OdomSample &sample) {
    Matrix4f correction;
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      if (!have_correction_) {
        return;
      }
      correction = map_to_camera_init_;
    }
    const Matrix4f map_to_body = correction * sample.camera_init_to_body;
    const Matrix4f output_to_body =
        use_field_frame_ ? field_map_transform_.mapPoseToField(map_to_body)
                         : map_to_body;
    const std::string &output_frame =
        use_field_frame_ ? field_frame_ : map_frame_;
    const auto pose = matrixToPose(output_to_body);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = sample.odom.header.stamp;
    pose_msg.header.frame_id = output_frame;
    pose_msg.pose = pose;
    pose_pub_->publish(pose_msg);

    nav_msgs::msg::Odometry global_odom;
    global_odom.header = pose_msg.header;
    global_odom.child_frame_id = body_frame_;
    global_odom.pose.pose = pose;
    global_odom.pose.covariance = sample.odom.pose.covariance;
    global_odom.twist = sample.odom.twist;
    odom_pub_->publish(global_odom);
    rememberPose(pose_msg);

    if (use_field_frame_) {
      geometry_msgs::msg::TransformStamped field_tf;
      field_tf.header.stamp = pose_msg.header.stamp;
      field_tf.header.frame_id = field_frame_;
      field_tf.child_frame_id = map_frame_;
      field_tf.transform = field_map_transform_.fieldToMapTransformMsg();
      tf_broadcaster_->sendTransform(field_tf);
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = pose_msg.header.stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.translation.x = correction(0, 3);
    tf_msg.transform.translation.y = correction(1, 3);
    tf_msg.transform.translation.z = correction(2, 3);
    const auto correction_pose = matrixToPose(correction);
    tf_msg.transform.rotation = correction_pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void rememberPose(const geometry_msgs::msg::PoseStamped &pose_msg) {
    std::lock_guard<std::mutex> lock(pose_history_mutex_);
    pose_history_.push_back(pose_msg);
    const rclcpp::Time newest(pose_msg.header.stamp);
    const double keep_sec = std::max(10.0, pose_report_window_sec_ * 3.0);
    while (pose_history_.size() > 1) {
      const rclcpp::Time oldest(pose_history_.front().header.stamp);
      if ((newest - oldest).seconds() <= keep_sec) {
        break;
      }
      pose_history_.pop_front();
    }
    while (pose_history_.size() > 1000) {
      pose_history_.pop_front();
    }
  }

  static double yawFromPose(const geometry_msgs::msg::Pose &pose) {
    const Eigen::Quaternionf q(static_cast<float>(pose.orientation.w),
                               static_cast<float>(pose.orientation.x),
                               static_cast<float>(pose.orientation.y),
                               static_cast<float>(pose.orientation.z));
    const auto rotation = q.normalized().toRotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  }

  void reportPoseCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    std::vector<geometry_msgs::msg::PoseStamped> samples;
    {
      std::lock_guard<std::mutex> lock(pose_history_mutex_);
      if (pose_history_.empty()) {
        response->success = false;
        response->message =
            "No localized pose yet. Wait for /r2/localized=true first.";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }

      const rclcpp::Time newest(pose_history_.back().header.stamp);
      for (auto iter = pose_history_.rbegin(); iter != pose_history_.rend();
           ++iter) {
        const rclcpp::Time stamp(iter->header.stamp);
        if ((newest - stamp).seconds() > pose_report_window_sec_) {
          break;
        }
        samples.push_back(*iter);
      }
    }

    if (samples.size() < static_cast<std::size_t>(pose_report_min_samples_)) {
      std::ostringstream warning;
      warning << "Not enough pose samples: got " << samples.size()
              << ", need " << pose_report_min_samples_
              << ". Keep the radar still and retry.";
      response->success = false;
      response->message = warning.str();
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw_sin = 0.0;
    double yaw_cos = 0.0;
    for (const auto &sample : samples) {
      x += sample.pose.position.x;
      y += sample.pose.position.y;
      z += sample.pose.position.z;
      const double yaw = yawFromPose(sample.pose);
      yaw_sin += std::sin(yaw);
      yaw_cos += std::cos(yaw);
    }
    const double count = static_cast<double>(samples.size());
    x /= count;
    y /= count;
    z /= count;
    const double yaw_rad = std::atan2(yaw_sin / count, yaw_cos / count);
    const double yaw_deg = yaw_rad * 180.0 / M_PI;

    std::ostringstream message;
    const std::string frame = samples.empty() ? map_frame_ : samples.front().header.frame_id;
    message << std::fixed << std::setprecision(3)
            << frame << " averaged pose over " << samples.size()
            << " samples / " << pose_report_window_sec_ << " s: x=" << x
            << " m, y=" << y << " m, z=" << z << " m, yaw=" << yaw_deg
            << " deg";
    response->success = true;
    response->message = message.str();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void publishGlobalMap() {
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_manager_->mapCloud(), map_msg);
    map_msg.header.stamp = now();
    map_msg.header.frame_id = map_frame_;
    map_pub_->publish(map_msg);
  }

  void publishRegisteredCloud(const Cloud::Ptr &cloud,
                              const rclcpp::Time &stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    registered_pub_->publish(msg);
  }

  void publishLocalized(bool localized) {
    std_msgs::msg::Bool msg;
    msg.data = localized;
    localized_pub_->publish(msg);
  }

  void publishFitness(double fitness) {
    std_msgs::msg::Float64 msg;
    msg.data = fitness;
    fitness_pub_->publish(msg);
  }

  void writeMetric(const rclcpp::Time &stamp, std::size_t input_points,
                   std::size_t overlap_points, double overlap_ratio,
                   bool converged, bool accepted, double fitness,
                   double translation_delta, double yaw_delta,
                   double roll_pitch_delta, bool initial) {
    if (!metrics_stream_) {
      return;
    }
    metrics_stream_ << stamp.seconds() << ',' << input_points << ','
                    << overlap_points << ',' << overlap_ratio << ','
                    << converged << ',' << accepted << ',' << fitness << ','
                    << translation_delta << ',' << yaw_delta << ','
                    << roll_pitch_delta << ',' << initial << '\n';
    metrics_stream_.flush();
  }

  std::string map_path_;
  std::string metrics_csv_path_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string map_frame_;
  std::string field_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  int initial_accumulation_frames_{30};
  int initial_max_attempts_{1};
  int initial_registration_interval_frames_{10};
  int registration_interval_frames_{1};
  int min_source_points_{100};
  double odom_sync_tolerance_sec_{0.03};
  double source_voxel_size_{0.20};
  double map_voxel_size_{0.20};
  double map_overlap_padding_{0.50};
  double map_match_radius_{0.50};
  double min_range_{0.0};
  double max_range_{100.0};
  double min_z_{-100.0};
  double max_z_{100.0};
  double ndt_epsilon_{0.01};
  double ndt_step_size_{0.10};
  double ndt_resolution_{0.50};
  int ndt_max_iterations_{35};
  double max_fitness_score_{0.15};
  double max_initial_fitness_score_{0.08};
  double min_overlap_ratio_{0.08};
  double min_initial_overlap_ratio_{0.10};
  double max_initial_translation_{3.0};
  double max_initial_yaw_{1.57};
  double max_initial_roll_pitch_{0.52};
  double max_correction_translation_{0.50};
  double max_correction_yaw_{0.35};
  double max_correction_roll_pitch_{0.10};
  double map_publish_period_sec_{2.0};
  double pose_report_window_sec_{3.0};
  int pose_report_min_samples_{5};
  bool use_field_frame_{false};

  bool have_correction_{false};
  bool match_valid_{false};
  int initial_frame_count_{0};
  int initial_attempt_count_{0};
  int registration_frame_count_{0};
  Matrix4f initial_map_body_guess_{Matrix4f::Identity()};
  Matrix4f map_to_camera_init_{Matrix4f::Identity()};
  Cloud::Ptr initial_cloud_;
  mutable std::mutex correction_mutex_;
  mutable std::mutex pose_history_mutex_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_history_;
  std::ofstream metrics_stream_;
  r2_localizer::FieldMapTransform field_map_transform_;

  std::unique_ptr<r2_localizer::MapManager> map_manager_;
  std::unique_ptr<r2_localizer::OdomSynchronizer> odom_synchronizer_;
  std::unique_ptr<NDTAligner> ndt_aligner_;

  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localized_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pose_report_srv_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<R2LocalizerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                       2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("r2_localizer"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
