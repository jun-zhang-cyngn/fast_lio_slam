// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cmath>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl {
struct PointXYZITR {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t timestamp;
  uint16_t ring;  /// laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
}


POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZITR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring));


typedef pcl::PointXYZI PointType;
// typedef pcl::PointXYZITR PointType;

pcl::PointXYZITR convert_pointxyzi(pcl::PointXYZI &pt) {
  pcl::PointXYZITR pt2;
  pt2.x = pt.x;
  pt2.y = pt.y;
  pt2.z = pt.z;
  uint16_t ring_id = (static_cast<uint32_t>(pt.intensity) & 0xFF000000) >> 24; 
  pt2.intensity = (static_cast<uint32_t>(pt.intensity) & 0x00FFFFFF);
  pt2.ring = ring_id;
  pt2.timestamp = 0;
  if(ring_id >= 128) {
    std::cout << "error ring id is incorrect " << ring_id << "\n";
    assert(ring_id < 128);
  }
  // also fix the intensity value of the original point
  pt.intensity = pt2.intensity;
  return pt2;
}
void convert_pointxyzi_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,  
  pcl::PointCloud<pcl::PointXYZITR>::Ptr &pc_out) {
  pc_out->points.clear();
  for(auto &pt : pc_in->points) {
    pc_out->points.push_back(convert_pointxyzi(pt));
  }
  pc_out->width = 1;
  pc_out->height = pc_out->points.size();
}


inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};