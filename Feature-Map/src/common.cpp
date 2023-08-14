// ***********************************************************************
// Copyright (c) 2021, by Cyngn. All rights reserved.
//
// Proprietary and confidential.
//
// ***********************************************************************

#include "params.h"
#include "common.h"

namespace cyngn::localization {

std::ostream& operator<<(std::ostream& os, const LidarID& obj) {
  os << static_cast<std::underlying_type<LidarID>::type>(obj);
  return os;
}

const std::unordered_map<LidarID, uint32_t> LidarInfo::lidar_lines_map_ = {
    {LidarID::PANDAR_40, 40}, {LidarID::PANDAR_QT, 64}, {LidarID::OUSTER_128, 128}};

uint32_t LidarInfo::GetLidarLines(const LidarID& lidar_id) const {
  return lidar_lines_map_.at(lidar_id);
}

}  // namespace cyngn::localization
