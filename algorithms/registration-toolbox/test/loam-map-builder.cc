#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "registration-toolbox/common/base-controller.h"
#include "registration-toolbox/common/supported.h"
#include "registration-toolbox/loam-controller.h"
#include "registration-toolbox/loam-feature-detector.h"

namespace regbox {

int counter = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
aslam::Transformation T_map_source;
auto aligner = regbox::BaseController::make("regbox::LoamController", "Loam");
auto feature_detector = regbox::LoamFeatureDetector();
ros::Publisher map_pub;
void registerCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud) {
  CHECK_NOTNULL(aligner);
  const regbox::RegistrationResult result =
      aligner->align(map_cloud, source_cloud, T_map_source);

  pcl::PointCloud<pcl::PointXYZI>::Ptr source_features_registered =
      result.getRegisteredCloud();

  *map_cloud += *source_features_registered;

  pcl::PointCloud<pcl::PointXYZI>::Ptr surface_map(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_map(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (pcl::PointXYZI map_point : *map_cloud) {
    if (map_point.intensity == 0) {
      surface_map->push_back(map_point);
    } else {
      edge_map->push_back(map_point);
    }
  }

  pcl::PointCloud<pcl::PointXYZI> edge_map_down_sampled;
  pcl::VoxelGrid<pcl::PointXYZI> edge_filter;
  edge_filter.setInputCloud(edge_map);
  edge_filter.setLeafSize(0.2, 0.2, 0.2);
  edge_filter.filter(edge_map_down_sampled);

  pcl::PointCloud<pcl::PointXYZI> surface_map_down_sampled;
  pcl::VoxelGrid<pcl::PointXYZI> surface_filter;
  surface_filter.setInputCloud(surface_map);
  surface_filter.setLeafSize(0.4, 0.4, 0.4);
  surface_filter.filter(surface_map_down_sampled);

  *map_cloud = surface_map_down_sampled + edge_map_down_sampled;

  T_map_source = result.get_T_target_source();
  std::cout << "Registration result: \n" << T_map_source << std::endl;

  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*map_cloud, map_msg);
  map_msg.header.frame_id = "/loam_map";
  map_pub.publish(map_msg);
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (counter > 0) {
    counter = 0;
    return;
  }
  pcl::PointCloud<pcl::PointXYZL> cloud_label;
  pcl::fromROSMsg(*msg, cloud_label);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (pcl::PointXYZL point : cloud_label) {
    pcl::PointXYZI point_intensity;
    point_intensity.getVector3fMap() = point.getVector3fMap();
    point_intensity.intensity = point.label;
    cloud_intensity->push_back(point_intensity);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_features(
      new pcl::PointCloud<pcl::PointXYZI>);

  feature_detector.extractLoamFeaturesFromPointCloud(
      cloud_intensity, cloud_features);

  registerCloud(cloud_features);
  counter++;
}

}  // namespace regbox

int main(int argc, char** argv) {
  ros::init(argc, argv, "registration_toolbox_driver");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  regbox::map_cloud =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  regbox::T_map_source.setIdentity();
  ros::NodeHandle nh;
  // ros::Subscriber pointcloud_sub = nh.subscribe("/os_cloud_node/points", 10,
  //   regbox::pointCloudCallback);
  ros::Subscriber pointcloud_sub =
      nh.subscribe("/points_undistorted", 100, regbox::pointCloudCallback);
  regbox::map_pub = nh.advertise<sensor_msgs::PointCloud2>("loam_map", 10);
  ros::spin();

  return 0;
}
