#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_localization_sc_qn_ros2/utilities.hpp"

namespace fast_lio_localization_sc_qn_ros2 {

struct PoseCloud {
  Cloud cloud_local;
  Eigen::Matrix4d pose_raw = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected = Eigen::Matrix4d::Identity();
  rclcpp::Time stamp;
  int index = 0;
  bool processed = false;

  PoseCloud() = default;

  PoseCloud(const nav_msgs::msg::Odometry &odom,
            const sensor_msgs::msg::PointCloud2 &cloud_msg,
            int frame_index)
      : pose_raw(poseMsgToMatrix(odom.pose.pose)),
        pose_corrected(pose_raw),
        stamp(odom.header.stamp),
        index(frame_index) {
    Cloud cloud_world;
    pcl::fromROSMsg(cloud_msg, cloud_world);
    // FAST-LIO 的 /cloud_registered 是世界系点云；匹配时保存为当前雷达局部系，沿用师兄 ROS1 逻辑。
    cloud_local = transformCloud(cloud_world, pose_raw.inverse());
  }
};

struct MapKeyframe {
  Cloud cloud_local;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  int index = 0;

  MapKeyframe() = default;

  MapKeyframe(const geometry_msgs::msg::PoseStamped &pose_msg,
              const sensor_msgs::msg::PointCloud2 &cloud_msg,
              int frame_index)
      : pose(poseMsgToMatrix(pose_msg.pose)), index(frame_index) {
    pcl::fromROSMsg(cloud_msg, cloud_local);
  }
};

}  // namespace fast_lio_localization_sc_qn_ros2
