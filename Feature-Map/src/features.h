/*
 * ***********************************************************************
 * Copyright (c) 2021, by Cyngn. All rights reserved.
 *
 * Proprietary and confidential.
 *
 * ***********************************************************************
 */

#pragma once

#include "pcl_custom_point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common.h"
#include "params.h"

namespace cyngn::localization {

using Feature = pcl::PointCloud<pcl::PointXYZI>::Ptr;

class FeatureExtraction {
 public:
  FeatureExtraction(FeatureParams feature_params, LidarID lidar_id);
  Status ExtractFeatures(const pcl::PointCloud<PointXYZITR>::Ptr &pc_in, Feature &edge_features,
                         Feature &surf_features);
  Status ExtractFeaturesFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, CloudCurvature &cloud_curvature,
                                   Feature &edge_features, Feature &surf_features);

 private:
  FeatureParams feature_params_;
  LidarID lidar_id_;
};


}  // namespace cyngn::localization
