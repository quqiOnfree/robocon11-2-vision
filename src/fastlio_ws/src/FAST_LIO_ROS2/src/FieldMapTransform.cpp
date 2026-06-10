#include "FieldMapTransform.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace r2_localizer {

FieldMapTransform::FieldMapTransform(const std::vector<double> &field_to_map) {
  setFieldToMap(field_to_map);
}

void FieldMapTransform::setFieldToMap(
    const std::vector<double> &field_to_map) {
  field_to_map_ = parseMatrix(field_to_map);
  map_to_field_ = field_to_map_.inverse();
}

const Matrix4f &FieldMapTransform::fieldToMapMatrix() const noexcept {
  return field_to_map_;
}

const Matrix4f &FieldMapTransform::mapToFieldMatrix() const noexcept {
  return map_to_field_;
}

Matrix4f FieldMapTransform::fieldPoseToMap(const Matrix4f &field_pose) const {
  return field_to_map_ * field_pose;
}

Matrix4f FieldMapTransform::mapPoseToField(const Matrix4f &map_pose) const {
  return map_to_field_ * map_pose;
}

Eigen::Vector3f FieldMapTransform::fieldPointToMap(
    const Eigen::Vector3f &field_point) const {
  const Eigen::Vector4f homogeneous(field_point.x(), field_point.y(),
                                    field_point.z(), 1.0F);
  return (field_to_map_ * homogeneous).head<3>();
}

Eigen::Vector3f FieldMapTransform::mapPointToField(
    const Eigen::Vector3f &map_point) const {
  const Eigen::Vector4f homogeneous(map_point.x(), map_point.y(),
                                    map_point.z(), 1.0F);
  return (map_to_field_ * homogeneous).head<3>();
}

geometry_msgs::msg::Pose FieldMapTransform::mapPoseToFieldMsg(
    const geometry_msgs::msg::Pose &map_pose) const {
  return matrixToPose(mapPoseToField(poseToMatrix(map_pose)));
}

geometry_msgs::msg::Transform FieldMapTransform::fieldToMapTransformMsg()
    const {
  return matrixToTransform(field_to_map_);
}

Matrix4f FieldMapTransform::parseMatrix(const std::vector<double> &values) {
  if (values.empty()) {
    return Matrix4f::Identity();
  }
  if (values.size() != 16) {
    throw std::runtime_error(
        "frames.field_to_map must contain 16 numbers in row-major order");
  }

  Matrix4f matrix;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = static_cast<float>(values[row * 4 + col]);
    }
  }
  return matrix;
}

geometry_msgs::msg::Pose FieldMapTransform::matrixToPose(
    const Matrix4f &matrix) {
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

Matrix4f FieldMapTransform::poseToMatrix(
    const geometry_msgs::msg::Pose &pose) {
  Matrix4f matrix = Matrix4f::Identity();
  Eigen::Quaternionf q(static_cast<float>(pose.orientation.w),
                       static_cast<float>(pose.orientation.x),
                       static_cast<float>(pose.orientation.y),
                       static_cast<float>(pose.orientation.z));
  if (q.norm() < 1e-6F) {
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

geometry_msgs::msg::Transform FieldMapTransform::matrixToTransform(
    const Matrix4f &matrix) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = matrix(0, 3);
  transform.translation.y = matrix(1, 3);
  transform.translation.z = matrix(2, 3);
  Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  q.normalize();
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  transform.rotation.w = q.w();
  return transform;
}

}  // namespace r2_localizer
