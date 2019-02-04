#ifndef RESOURCE_IMPORTER_SIMPLE_ROSBAG_READER_H_
#define RESOURCE_IMPORTER_SIMPLE_ROSBAG_READER_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Transform.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

class SimpleRosbagSource {
 public:
  SimpleRosbagSource(
      const std::string& rosbag_filename, const std::string& rostopic,
      const std::string& rostopic_camera_info, const std::string& imu_frame,
      const std::string& camera_frame);

  void readRosbag();

  void setImageCallback(
      const std::function<void(sensor_msgs::ImageConstPtr)>& callback);

  void setPointcloudCallback(
      const std::function<void(sensor_msgs::PointCloud2ConstPtr)>& callback);

  void setCameraInfoCallback(
      const std::function<void(sensor_msgs::CameraInfoConstPtr)>& callback);

  void setCameraExtrinsicsCallback(
      const std::function<void(geometry_msgs::Transform)>& callback);

 private:
  void openRosbag();

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> bag_view_;

  const std::string rosbag_filename_;
  const std::string resource_topic_;
  const std::string camera_info_topic_;

  const std::string rosbag_imu_frame_;
  const std::string rosbag_camera_frame_;

  // We fill this transformer with all tf messages until we can successfully
  // retrieve the camera extrinsics we are interested in.
  tf::Transformer tf_transformer_;

  std::function<void(sensor_msgs::ImageConstPtr)> image_lambda_;
  std::function<void(sensor_msgs::PointCloud2ConstPtr)> pointcloud_lambda_;
  std::function<void(sensor_msgs::CameraInfoConstPtr)> camera_info_lambda_;
  std::function<void(geometry_msgs::Transform)> camera_extrinsics_lambda_;
};

#endif  // RESOURCE_IMPORTER_SIMPLE_ROSBAG_READER_H_
