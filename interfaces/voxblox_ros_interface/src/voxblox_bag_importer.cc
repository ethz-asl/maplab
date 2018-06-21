#include <memory>

#include <vi-map/vi-map-serialization.h>

#include "voxblox_ros_interface/voxblox_bag_importer.h"

namespace voxblox {

VoxbloxBagImporter::VoxbloxBagImporter(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), tsdf_server_(nh, nh_private) {}

bool VoxbloxBagImporter::setupRosbag(const std::string& filename,
                                     const std::string& pointcloud_topic) {
  rosbag_source_.reset(
      new SimpleRosbagSource(filename, pointcloud_topic, "", "", ""));

  rosbag_source_->setNonConstPointcloudCallback(std::bind(
      &VoxbloxBagImporter::pointcloudCallback, this, std::placeholders::_1));
}

bool VoxbloxBagImporter::setupMap(const std::string& map_path) {
  const bool result =
      vi_map::serialization::loadMapFromFolder(map_path, &vi_map_);
  if (result != true) {
    return false;
  }

  // Figure out the mission ID. We always just take the first mission in the
  // thing.
  vi_map::MissionIdList mission_ids;
  vi_map_.getAllMissionIds(&mission_ids);
  if (mission_ids.size() < 1) {
    return false;
  }
  mission_id_ = mission_ids[0];

  // Fill in min and max timestamps for future pose lookups.
  landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
  pose_interpolator_.getVertexToTimeStampMap(
      vi_map_, mission_id_, &vertex_to_time_map, &min_timestamp_ns_,
      &max_timestamp_ns_);
  if (vertex_to_time_map.empty()) {
    return false;
  }
  return true;
}

bool VoxbloxBagImporter::setupSensor(const std::string& calibration_file_path) {
  // Load camera calibration.
  aslam::NCamera::Ptr ncamera =
      aslam::NCamera::loadFromYaml(calibration_file_path);
  if (!ncamera) {
    return false;
  }

  const bool has_one_camera = ncamera->getNumCameras() == 1u;
  if (!has_one_camera) {
    return false;
  }
  T_I_C_ = ncamera->get_T_C_B(0).inverse().cast<FloatingPoint>();
  return true;
}

bool VoxbloxBagImporter::lookupTransformInMap(int64_t timestamp_ns,
                                              voxblox::Transformation* T_G_I) {
  if (timestamp_ns < min_timestamp_ns_ || timestamp_ns > max_timestamp_ns_) {
    return false;
  }

  // Interpolate poses at resource timestamp.
  // You have to feed a vector in, no way to just look up one pose. :(

  aslam::TransformationVector poses_M_I;
  const landmark_triangulation::PoseInterpolator pose_interpolator;

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(1);
  resource_timestamps << timestamp_ns;
  size_t index = 0;

  pose_interpolator.getPosesAtTime(vi_map_, mission_id_, resource_timestamps,
                                   &poses_M_I);

  const aslam::Transformation& T_G_M =
      vi_map_.getMissionBaseFrameForMission(mission_id_).get_T_G_M();
  const aslam::Transformation& T_M_I = poses_M_I[index];

  *T_G_I = (T_G_M * T_M_I).cast<FloatingPoint>() * T_I_C_;

  return true;
}

void VoxbloxBagImporter::pointcloudCallback(
    sensor_msgs::PointCloud2Ptr pointcloud) {
  voxblox::Transformation T_G_I;
  const int64_t timestamp_ns = pointcloud->header.stamp.toNSec();
  if (!lookupTransformInMap(timestamp_ns, &T_G_I)) {
    return;
  }

  voxblox::Transformation T_G_C = T_G_I * T_I_C_;
  const bool is_freespace_pointcloud = false;

  tsdf_server_.processPointCloudMessageAndInsert(pointcloud, T_G_C,
                                                 is_freespace_pointcloud);
}

void VoxbloxBagImporter::run() { rosbag_source_->readRosbag(); }

}  // namespace voxblox
