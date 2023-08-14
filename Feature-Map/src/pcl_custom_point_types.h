/*
 * ***********************************************************************
 * Copyright (c) 2021, by Cyngn. All rights reserved.
 *
 * Proprietary and confidential.
 *
 * ***********************************************************************
 */

#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Note: include this file before any pcl file to avoid linking errors

namespace cyngn::localization {
struct PointXYZITR {
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace cyngn::localization

POINT_CLOUD_REGISTER_POINT_STRUCT(cyngn::localization::PointXYZITR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(double, timestamp,
                                                                                     timestamp)(uint16_t, ring, ring));
