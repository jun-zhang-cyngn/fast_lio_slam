// ***********************************************************************
// Copyright (c) 2021, by Cyngn. All rights reserved.
//
// Proprietary and confidential.
//
// ***********************************************************************

#include "features.h"
#include "mypcl.hpp"

#include <math.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/console.h>

#include "params.h"

namespace cyngn::localization {

FeatureExtraction::FeatureExtraction(FeatureParams feature_params, LidarID lidar_id)
    : feature_params_(feature_params), lidar_id_(lidar_id) {}

Status FeatureExtraction::ExtractFeatures(const pcl::PointCloud<PointXYZITR>::Ptr &pc_in, Feature &edge_features,
                                          Feature &surf_features) {
  LidarInfo lidar_info;
  const uint32_t kNScans = lidar_info.GetLidarLines(lidar_id_);
  if (kNScans == 0) {
    LogError("Incorrect number of scan lines");
    return Status::FAILURE;
  }
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laser_cloud_scans;
  for (size_t i = 0; i < kNScans; i++) {
    laser_cloud_scans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
  }

  const size_t pt_size = pc_in->points.size();
  for (size_t i = 0; i < pt_size; i++) {
    const auto &pt = pc_in->points[i];
    const uint32_t scan_id = pt.ring;
    if (scan_id >= kNScans) {
      LogError("wrong scan number " + std::to_string(kNScans) + " or wrong lidar model " +
               std::to_string(static_cast<int>(lidar_id_)));
      continue;
    }

    pcl::PointXYZI pt2;
    pt2.x = pt.x;
    pt2.y = pt.y;
    pt2.z = pt.z;
    pt2.intensity = pt.intensity;
    laser_cloud_scans[scan_id]->push_back(pt2);
  }

  for (const auto &laser_cloud_scan : laser_cloud_scans) {
    if (laser_cloud_scan->points.size() < feature_params_.min_num_ring_points) {
      continue;
    }

    std::vector<Double2d> cloud_curvature;
    int range_len = (feature_params_.curvature_calc_range_left[1] - feature_params_.curvature_calc_range_left[0] + 1) +
                    (feature_params_.curvature_calc_range_right[1] - feature_params_.curvature_calc_range_right[0] + 1);
    int total_points = laser_cloud_scan->points.size() - range_len;

    if (total_points <= 0) {
      LogError("total points cannot be <= 0");
      return Status::FAILURE;
    }
    for (size_t j = feature_params_.curvature_calc_range_left[1];
         j < laser_cloud_scan->points.size() - feature_params_.curvature_calc_range_right[1]; ++j) {
      Eigen::Vector3d diff_xyz = Eigen::Vector3d::Zero();

      for (size_t k = feature_params_.curvature_calc_range_left[0]; k <= feature_params_.curvature_calc_range_left[1];
           ++k) {
        diff_xyz(0) += laser_cloud_scan->points[j - k].x;
        diff_xyz(1) += laser_cloud_scan->points[j - k].y;
        diff_xyz(2) += laser_cloud_scan->points[j - k].z;
      }

      for (size_t k = feature_params_.curvature_calc_range_right[0]; k <= feature_params_.curvature_calc_range_right[1];
           ++k) {
        diff_xyz(0) += laser_cloud_scan->points[j + k].x;
        diff_xyz(1) += laser_cloud_scan->points[j + k].y;
        diff_xyz(2) += laser_cloud_scan->points[j + k].z;
      }

      diff_xyz(0) -= (static_cast<float>(range_len) * laser_cloud_scan->points[j].x);
      diff_xyz(1) -= (static_cast<float>(range_len) * laser_cloud_scan->points[j].y);
      diff_xyz(2) -= (static_cast<float>(range_len) * laser_cloud_scan->points[j].z);

      cloud_curvature.push_back({static_cast<int32_t>(j), diff_xyz.squaredNorm()});
    }

    uint32_t sector_length = static_cast<uint32_t>(total_points / feature_params_.num_sectors);
    if (sector_length == 0) {
      LogError("sector length cannot be  0");
    }
    for (size_t j = 0; j < feature_params_.num_sectors; ++j) {
      uint32_t sector_start = (sector_length * j);
      uint32_t sector_end = (sector_length * (j + 1)) - 1;
      if (j == (feature_params_.num_sectors - 1)) {
        sector_end = total_points - 1;
      }

      if (sector_end >= cloud_curvature.size()) {
        LogError("sector end is out of bounds");
        return Status::FAILURE;
      }

      std::vector<Double2d> sub_cloud_curvature(cloud_curvature.begin() + sector_start,
                                                cloud_curvature.begin() + sector_end);
      ExtractFeaturesFromSector(laser_cloud_scan, sub_cloud_curvature, edge_features, surf_features);
    }
  }

  return Status::SUCCESS;
}

Status FeatureExtraction::ExtractFeaturesFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                                                    CloudCurvature &cloud_curvature, Feature &edge_features,
                                                    Feature &surf_features) {
  std::sort(cloud_curvature.begin(), cloud_curvature.end(),
            [](const Double2d &a, const Double2d &b) { return a.value < b.value; });

  int largest_picked_num = 0;
  std::vector<int> picked_points;
  for (auto curvature_it = cloud_curvature.rbegin(); curvature_it != cloud_curvature.rend(); ++curvature_it) {
    int ind = curvature_it->id;
    if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
      if (curvature_it->value <= feature_params_.feature_curv_threshold) {
        break;
      }

      largest_picked_num++;
      picked_points.push_back(ind);

      if (largest_picked_num <= feature_params_.largest_picked_num_threshold) {
        edge_features->push_back(pc_in->points[ind]);
      } else {
        break;
      }

      for (size_t k = feature_params_.curvature_calc_range_right[0]; k <= feature_params_.curvature_calc_range_right[1];
           k++) {
        Eigen::Vector3d diff_xyz;
        diff_xyz(0) = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
        diff_xyz(1) = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
        diff_xyz(2) = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;

        if (diff_xyz.squaredNorm() > feature_params_.partial_curvature_threshold_right) {
          break;
        }
        picked_points.push_back(ind + k);
      }

      for (size_t k = feature_params_.curvature_calc_range_left[0]; k >= feature_params_.curvature_calc_range_left[1];
           k--) {
        Eigen::Vector3d diff_xyz;
        diff_xyz(0) = pc_in->points[ind - k].x - pc_in->points[ind - k + 1].x;
        diff_xyz(1) = pc_in->points[ind - k].y - pc_in->points[ind - k + 1].y;
        diff_xyz(2) = pc_in->points[ind - k].z - pc_in->points[ind - k + 1].z;
        if (diff_xyz.squaredNorm() > feature_params_.partial_curvature_threshold_left) {
          break;
        }
        picked_points.push_back(ind + k);
      }
    }
  }

  for (const auto &curvature : cloud_curvature) {
    const int ind = curvature.id;
    if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
      surf_features->push_back(pc_in->points[ind]);
    }
  }

  return Status::SUCCESS;
}

}  // namespace cyngn::localization

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cyngn_feature_map");
  ros::NodeHandle nh;

  std::string file_path = "/home/ubuntu/github/catkin_fastlio/data/2023-07-25-21-05-17_0814_surf/feat/";
  double downsample_size = 0.1;
  int pcd_name_fill_num = 6;;

  // nh.getParam("file_path", file_path);
  // nh.getParam("downsample_size", downsample_size);
  // nh.getParam("pcd_name_fill_num", pcd_name_fill_num);
  
  std::cout << "file path " << file_path << "\n";
  std::vector<mypcl::pose> pose_vec;
  std::cout << "reading pose file.." << file_path << std::endl;
  pose_vec = mypcl::read_pose(file_path + "/pose.json");
  size_t pose_size = pose_vec.size();
  std::cout<<"pose size "<< pose_size << std::endl;

  cyngn::localization::FeatureParams feature_params;

  feature_params.feature_curv_threshold = 0.1;
  feature_params.largest_picked_num_threshold = 20;
  feature_params.partial_curvature_threshold_right = 0.05;
  feature_params.partial_curvature_threshold_left = 0.05;
  
  feature_params.min_num_ring_points = 131;
  feature_params.num_sectors = 6;

  const size_t curvature_calc_range_right_min = 1;
  const size_t curvature_calc_range_right_max = 5;
  const size_t curvature_calc_range_left_min = 1;
  const size_t curvature_calc_range_left_max = 5;

  feature_params.curvature_calc_range_right = {curvature_calc_range_right_min, curvature_calc_range_right_max};
  feature_params.curvature_calc_range_left = {curvature_calc_range_left_min, curvature_calc_range_left_max};
  

  auto fe = std::make_unique<cyngn::localization::FeatureExtraction>(feature_params, cyngn::localization::LidarID::OUSTER_128);

  // read the poses and PCD 
  pcl::PointCloud<cyngn::localization::PointXYZITR>::Ptr pc_surf(new pcl::PointCloud<cyngn::localization::PointXYZITR>);

  cyngn::localization::Feature surf_features = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cyngn::localization::Feature edge_features = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  for(size_t i = 0; i < pose_size; i++)
  {
    std::cout << "extracting features for frame " << i << "\n";
    mypcl::loadPCD(file_path + "pcd/", pcd_name_fill_num, pc_surf, i);

    pcl::PointCloud<cyngn::localization::PointXYZITR>::Ptr pc_filtered(new pcl::PointCloud<cyngn::localization::PointXYZITR>);
    
    pc_filtered->resize(pc_surf->points.size());
    int cnt = 0;
    for(size_t j = 0; j < pc_surf->points.size(); j++)
    {
      pc_filtered->points[cnt] = pc_surf->points[j];
      cnt++;
    }
    pc_filtered->resize(cnt);
    mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);

    fe->ExtractFeatures(pc_filtered, edge_features, surf_features);
  }

  pcl::VoxelGrid<pcl::PointXYZI> surf_down_sample_filter_;
  pcl::VoxelGrid<pcl::PointXYZI> edge_down_sample_filter_;

  surf_down_sample_filter_.setLeafSize(0.4, 0.4, 0.4);
  surf_down_sample_filter_.setInputCloud(surf_features);
  surf_down_sample_filter_.filter(*surf_features);
  
  edge_down_sample_filter_.setLeafSize(0.4, 0.4, 0.4);
  edge_down_sample_filter_.setInputCloud(edge_features);
  edge_down_sample_filter_.filter(*edge_features);

  pcl::io::savePCDFileBinary(file_path + "/surf_map.pcd", *surf_features);  
  pcl::io::savePCDFileBinary(file_path + "/edge_map.pcd", *edge_features); 

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
} 
