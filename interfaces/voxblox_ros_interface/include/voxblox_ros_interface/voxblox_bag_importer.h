#ifndef VOXBLOX_ROS_INTERFACE_VOXBLOX_BAG_IMPORTER_H_
#define VOXBLOX_ROS_INTERFACE_VOXBLOX_BAG_IMPORTER_H_

#include <landmark-triangulation/pose-interpolator.h>
#include <map-manager/map-manager.h>
#include <resource-importer/simple-rosbag-reader.h>
#include <vi-map/vi-map.h>
#include <voxblox_ros/tsdf_server.h>

namespace voxblox {

class VoxbloxBagImporter {
 public:
  VoxbloxBagImporter(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  void setSubsampling(int integrate_every_nth_message);

  bool setupRosbag(const std::string& filename,
                   const std::string& pointcloud_topic);

  bool setupMap(const std::string& map_path);
  bool setupSensor(const std::string& calibration_file_path);

  bool lookupTransformInMap(int64_t timestamp_ns,
                            voxblox::Transformation* T_G_I);

  void pointcloudCallback(sensor_msgs::PointCloud2Ptr pointcloud);

  void run();

  // Generates and publishes the mesh.
  void visualize();

  void save(const std::string& output_path);
  size_t numMessages() const;
  int getSubsampling() const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Some more settings...
  int integrate_every_nth_message_;
  // Timestamp limits.
  int64_t min_timestamp_ns_;
  int64_t max_timestamp_ns_;
  // Calibration
  voxblox::Transformation T_I_C_;

  // Voxblox TSDF server.
  TsdfServer tsdf_server_;
  // The rosbag to load from.
  std::unique_ptr<SimpleRosbagSource> rosbag_source_;
  // The map!
  vi_map::VIMap vi_map_;
  // Interpolate poses.
  landmark_triangulation::PoseInterpolator pose_interpolator_;

  // Temporary state.
  // We only support one mission.
  vi_map::MissionId mission_id_;
  size_t message_index_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_INTERFACE_VOXBLOX_BAG_IMPORTER_H_
