#ifndef POINTCLOUD_UNDISTORTION_UNDISTORTION_H_
#define POINTCLOUD_UNDISTORTION_UNDISTORTION_H_

#include <resources-common/point-cloud.h>
#include <vi-map/vi-map.h>

namespace pointcloud_undistortion {

bool undistortPointCloud(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const aslam::SensorId& sensor_id, const int64_t& timestamp_ns,
    resources::PointCloud* cloud);

}  //  namespace pointcloud_undistortion

#endif  //  POINTCLOUD_UNDISTORTION_UNDISTORTION_H_
