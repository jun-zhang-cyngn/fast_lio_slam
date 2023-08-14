/*
 * ***********************************************************************
 * Copyright (c) 2021, by Cyngn. All rights reserved.
 *
 * Proprietary and confidential.
 *
 * ***********************************************************************
 */

#pragma once

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <array>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace cyngn::localization {

enum class LidarID : uint8_t
{
  PANDAR_40 = 0,
  PANDAR_QT = 1,
  OUSTER_128 = 2,
};

std::ostream& operator<<(std::ostream& os, const LidarID& obj);

class LidarInfo {
 public:
  uint32_t GetLidarLines(const LidarID& lidar_id) const;

 private:
  static const std::unordered_map<LidarID, uint32_t> lidar_lines_map_;
};

enum class Status
{
  SUCCESS,
  FAILURE,
  WARNING
};

struct Double2d {
  int id;
  double value;
  Double2d(int id_, double value_) : id(id_), value(value_){};
};

using CloudCurvature = std::vector<Double2d>;

inline void LogError(const std::string& msg) {
  LOG(ERROR) << ros::Time::now() << " | "
             << "lidar localizer"
             << " | " << msg;
}

inline void LogErrorBridge(const std::string& msg) {
  LOG(ERROR) << ros::Time::now() << " | "
             << "lidar localizer bridge"
             << " | " << msg;
}

inline void LogInfo(const std::string& msg) {
  LOG(INFO) << ros::Time::now() << " | "
            << "lidar localizer"
            << " | " << msg;
}

inline void LogWarn(const std::string& msg) {
  LOG(WARNING) << ros::Time::now() << " | "
               << "lidar localizer"
               << " | " << msg;
}

inline Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch, const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

inline void QuatToRollPitchYaw(const Eigen::Quaterniond& quat, double& roll, double& pitch, double& yaw) {
  auto rot = quat.toRotationMatrix();
  roll = atan2(rot(2, 1), rot(2, 2));
  pitch = atan2(-rot(2, 0), std::pow(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2), 0.5));
  yaw = atan2(rot(1, 0), rot(0, 0));
}

inline gtsam::Pose3 convertIsometry3dToPose3(const Eigen::Isometry3d& isometry) {
  // Extract rotation and translation from Isometry3d
  Eigen::Matrix3d eigen_rotation = isometry.rotation();
  Eigen::Vector3d eigen_translation = isometry.translation();

  // Convert rotation to gtsam::Rot3
  gtsam::Matrix3 gtsam_rotation;
  gtsam_rotation << eigen_rotation;
  gtsam::Rot3 rotation(gtsam_rotation);

  // Convert translation to gtsam::Point3
  gtsam::Point3 translation(eigen_translation.x(), eigen_translation.y(), eigen_translation.z());

  // Construct gtsam::Pose3
  return gtsam::Pose3(rotation, translation);
}

inline gtsam::Pose3 convertMatrix4dToPose3(const Eigen::Matrix4d& matrix) {
  Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);
  Eigen::Vector3d translation_vector = matrix.block<3, 1>(0, 3);
  gtsam::Rot3 rotation = gtsam::Rot3(rotation_matrix);
  gtsam::Point3 translation = gtsam::Point3(translation_vector);
  return gtsam::Pose3(rotation, translation);
}

inline Eigen::Isometry3d convertPose3ToIsometry3d(const gtsam::Pose3& pose) {
  // Extract rotation and translation from Pose3
  gtsam::Rot3 rotation = pose.rotation();
  gtsam::Point3 translation = pose.translation();

  // Convert rotation to Eigen::Matrix3d
  Eigen::Matrix3d eigen_rotation;
  eigen_rotation << rotation.matrix();

  // Convert translation to Eigen::Vector3d
  Eigen::Vector3d eigen_translation(translation.x(), translation.y(), translation.z());

  // Construct Eigen::Isometry3d
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.rotate(eigen_rotation);
  isometry.pretranslate(eigen_translation);

  return isometry;
}

}  // namespace cyngn::localization
