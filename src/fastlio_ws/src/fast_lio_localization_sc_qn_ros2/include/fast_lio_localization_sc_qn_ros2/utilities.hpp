#pragma once

#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fast_lio_localization_sc_qn_ros2 {

using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;

inline Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::Pose &pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  q.normalize();
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  mat.block<3, 3>(0, 0) = q.toRotationMatrix();
  mat(0, 3) = pose.position.x;
  mat(1, 3) = pose.position.y;
  mat(2, 3) = pose.position.z;
  return mat;
}

inline geometry_msgs::msg::Pose matrixToPoseMsg(const Eigen::Matrix4d &pose) {
  Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  q.normalize();
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose(0, 3);
  msg.position.y = pose(1, 3);
  msg.position.z = pose(2, 3);
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

inline geometry_msgs::msg::PoseStamped matrixToPoseStamped(
    const Eigen::Matrix4d &pose, const std::string &frame_id,
    const rclcpp::Time &stamp = rclcpp::Time(0, 0, RCL_ROS_TIME)) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose = matrixToPoseMsg(pose);
  return msg;
}

template <typename PointT>
inline pcl::PointCloud<PointT> transformCloud(const pcl::PointCloud<PointT> &cloud,
                                              const Eigen::Matrix4d &tf) {
  if (cloud.empty()) {
    return cloud;
  }
  pcl::PointCloud<PointT> out;
  pcl::transformPointCloud(cloud, out, tf.cast<float>());
  return out;
}

inline Cloud::Ptr voxelizeCloud(const Cloud &cloud, double voxel_size) {
  Cloud::Ptr input(new Cloud(cloud));
  Cloud::Ptr output(new Cloud);
  if (voxel_size <= 0.0) {
    *output = cloud;
    return output;
  }
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size),
                    static_cast<float>(voxel_size));
  voxel.setInputCloud(input);
  voxel.filter(*output);
  return output;
}

inline sensor_msgs::msg::PointCloud2 cloudToRosMsg(const Cloud &cloud,
                                                   const std::string &frame_id,
                                                   const rclcpp::Time &stamp = rclcpp::Time(0, 0, RCL_ROS_TIME)) {
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return msg;
}

}  // namespace fast_lio_localization_sc_qn_ros2
