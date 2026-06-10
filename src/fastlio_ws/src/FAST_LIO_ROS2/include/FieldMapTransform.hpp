#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace r2_localizer {

using Matrix4f = Eigen::Matrix4f;

// 维护真实场地水平坐标 field 与预制 PCD 坐标 map 之间的固定刚体变换。
class FieldMapTransform {
public:
  FieldMapTransform() = default;
  explicit FieldMapTransform(const std::vector<double> &field_to_map);

  void setFieldToMap(const std::vector<double> &field_to_map);

  const Matrix4f &fieldToMapMatrix() const noexcept;
  const Matrix4f &mapToFieldMatrix() const noexcept;

  Matrix4f fieldPoseToMap(const Matrix4f &field_pose) const;
  Matrix4f mapPoseToField(const Matrix4f &map_pose) const;
  Eigen::Vector3f fieldPointToMap(const Eigen::Vector3f &field_point) const;
  Eigen::Vector3f mapPointToField(const Eigen::Vector3f &map_point) const;

  geometry_msgs::msg::Pose mapPoseToFieldMsg(
      const geometry_msgs::msg::Pose &map_pose) const;
  geometry_msgs::msg::Transform fieldToMapTransformMsg() const;

private:
  static Matrix4f parseMatrix(const std::vector<double> &values);
  static geometry_msgs::msg::Pose matrixToPose(const Matrix4f &matrix);
  static Matrix4f poseToMatrix(const geometry_msgs::msg::Pose &pose);
  static geometry_msgs::msg::Transform matrixToTransform(const Matrix4f &matrix);

  Matrix4f field_to_map_{Matrix4f::Identity()};
  Matrix4f map_to_field_{Matrix4f::Identity()};
};

}  // namespace r2_localizer
