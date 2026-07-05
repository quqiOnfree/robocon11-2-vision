#pragma once

#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

namespace fast_lio_sam_sc_qn_ros2 {

using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;

inline Cloud::Ptr voxelizeCloud(const Cloud &cloud, const float leaf_size) {
  auto in = Cloud::Ptr(new Cloud(cloud));
  auto out = Cloud::Ptr(new Cloud);
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(in);
  voxel.filter(*out);
  return out;
}

inline Cloud transformCloud(const Cloud &cloud, const Eigen::Matrix4d &tf) {
  Cloud out;
  if (cloud.empty()) {
    return out;
  }
  pcl::transformPointCloud(cloud, out, tf);
  return out;
}

inline Eigen::Matrix4d odomToMatrix(const nav_msgs::msg::Odometry &odom) {
  const auto &p = odom.pose.pose.position;
  const auto &q = odom.pose.pose.orientation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  quat.normalize();

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();
  matrix(0, 3) = p.x;
  matrix(1, 3) = p.y;
  matrix(2, 3) = p.z;
  return matrix;
}

inline geometry_msgs::msg::Pose matrixToPoseMsg(const Eigen::Matrix4d &matrix) {
  Eigen::Quaterniond quat(matrix.block<3, 3>(0, 0));
  quat.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

inline geometry_msgs::msg::PoseStamped matrixToPoseStamped(
    const Eigen::Matrix4d &matrix, const std::string &frame_id,
    const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose = matrixToPoseMsg(matrix);
  return pose;
}

inline gtsam::Pose3 matrixToGtsamPose(const Eigen::Matrix4d &matrix) {
  const Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0);
  return gtsam::Pose3(
      gtsam::Rot3(rot),
      gtsam::Point3(matrix(0, 3), matrix(1, 3), matrix(2, 3)));
}

inline Eigen::Matrix4d gtsamPoseToMatrix(const gtsam::Pose3 &pose) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = pose.rotation().matrix();
  matrix(0, 3) = pose.translation().x();
  matrix(1, 3) = pose.translation().y();
  matrix(2, 3) = pose.translation().z();
  return matrix;
}

struct PoseCloud {
  Cloud cloud_local;
  Eigen::Matrix4d pose_raw = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected = Eigen::Matrix4d::Identity();
  rclcpp::Time stamp;
  int index = 0;
  bool loop_processed = false;

  PoseCloud() = default;

  PoseCloud(const nav_msgs::msg::Odometry &odom,
            const sensor_msgs::msg::PointCloud2 &cloud_msg,
            const int keyframe_index)
      : pose_raw(odomToMatrix(odom)), pose_corrected(pose_raw),
        stamp(odom.header.stamp), index(keyframe_index) {
    Cloud world_cloud;
    pcl::fromROSMsg(cloud_msg, world_cloud);
    // FAST-LIO 的 /cloud_registered 是 camera_init/world 系点云。
    // 后端保存关键帧时转回局部坐标，后续用优化后的 pose 再拼回全局图。
    cloud_local = transformCloud(world_cloud, pose_raw.inverse());
  }
};

}  // namespace fast_lio_sam_sc_qn_ros2
