#ifndef VOXBLOX_ROS_INTERFACE_VOXBLOX_BAG_IMPORTER_H_
#define VOXBLOX_ROS_INTERFACE_VOXBLOX_BAG_IMPORTER_H_

#include <landmark-triangulation/pose-interpolator.h>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>
#include <voxblox_ros/tsdf_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <image_undistort/depth.h>
#include <image_undistort/stereo_undistort.h>

namespace voxblox {

class VoxbloxBagImporter {
 public:
  VoxbloxBagImporter(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  void setSubsampling(int integrate_every_nth_message);

  bool setupRosbag(const std::string& filename);
  bool setupMap(const std::string& map_path);

  bool setupPointcloudSensor(const std::string& pointcloud_topic,
                             const std::string& camchain_namespace);
  bool setupStereoSensor(const std::string& cam0_topic,
                         const std::string& cam1_topic,
                         const std::string& camchain_namespace);

  bool lookupTransformInMap(int64_t timestamp_ns,
                            voxblox::Transformation* T_G_I);

  void pointcloudCallback(sensor_msgs::PointCloud2Ptr pointcloud);

  void cameraCallback(sensor_msgs::ImageConstPtr image, bool left);
  void stereoPointcloudCallback(sensor_msgs::PointCloud2Ptr pointcloud);

  // Actually put the pointcloud into the map.
  void integratePointcloud(const Transformation& T_I_S,
                           sensor_msgs::PointCloud2Ptr pointcloud);

  void run();

  // Generates and publishes the mesh.
  void visualize();

  void save(const std::string& output_path);
  size_t numMessages() const;
  int getSubsampling() const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Topics to listen to.
  std::string pointcloud_topic_;
  std::string cam0_topic_;
  std::string cam1_topic_;

  // Calibration
  voxblox::Transformation T_I_P_;
  voxblox::Transformation T_I_C0_;
  voxblox::Transformation T_I_C1_;

  // Some more settings...
  int integrate_every_nth_message_;
  // Timestamp limits.
  int64_t min_timestamp_ns_;
  int64_t max_timestamp_ns_;

  // Bag stuff.
  rosbag::Bag bag_;
  std::unique_ptr<rosbag::View> bag_view_;

  // Stereo stuff!
  // TODO: make this an actually usable library. ;)
  image_undistort::StereoUndistort stereo_undistort_;
  image_undistort::Depth depth_;
  // ROS subscriber to get the data back out.
  ros::Subscriber stereo_ptcloud_sub_;

  // Storage for unpaired images.
  sensor_msgs::ImageConstPtr left_image_;
  sensor_msgs::ImageConstPtr right_image_;

  // Voxblox TSDF server.
  TsdfServer tsdf_server_;
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
