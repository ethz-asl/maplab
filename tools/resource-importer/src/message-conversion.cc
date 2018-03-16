#include "resource-importer/message-conversion.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/distortion.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <resources-common/point-cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <vi-map/vi-map-serialization.h>

void convertPointCloudMessage(
    sensor_msgs::PointCloud2ConstPtr point_cloud_msg,
    resources::PointCloud* maplab_pointcloud) {
  CHECK(point_cloud_msg);
  CHECK_NOTNULL(maplab_pointcloud);

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
  pcl::fromROSMsg(*point_cloud_msg, pointcloud_pcl);

  size_t flattened_size = 3u * pointcloud_pcl.size();
  maplab_pointcloud->xyz.reserve(flattened_size);
  // TODO(mfehr): Make sure there are colors in the point cloud.
  maplab_pointcloud->colors.reserve(flattened_size);
  // TODO(mfehr): Check if there are normals in the point cloud and use it
  // if there are.
  maplab_pointcloud->normals.resize(flattened_size, 0.0f);

  for (size_t i = 0u; i < pointcloud_pcl.points.size(); ++i) {
    if (!std::isfinite(pointcloud_pcl.points[i].x) ||
        !std::isfinite(pointcloud_pcl.points[i].y) ||
        !std::isfinite(pointcloud_pcl.points[i].z)) {
      continue;
    }

    maplab_pointcloud->xyz.push_back(pointcloud_pcl.points[i].x);
    maplab_pointcloud->xyz.push_back(pointcloud_pcl.points[i].y);
    maplab_pointcloud->xyz.push_back(pointcloud_pcl.points[i].z);

    maplab_pointcloud->colors.push_back(pointcloud_pcl.points[i].r);
    maplab_pointcloud->colors.push_back(pointcloud_pcl.points[i].g);
    maplab_pointcloud->colors.push_back(pointcloud_pcl.points[i].b);
  }
}

void convertDepthImageMessage(
    sensor_msgs::ImageConstPtr image_message, cv::Mat* image) {
  CHECK(image_message);
  CHECK_NOTNULL(image);
  CHECK_EQ(image_message->encoding, sensor_msgs::image_encodings::TYPE_16UC1);

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(
        image_message, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);

  *image = cv_ptr->image.clone();
}

void convertColorImageMessage(
    sensor_msgs::ImageConstPtr image_message, cv::Mat* image) {
  CHECK(image_message);
  CHECK_NOTNULL(image);
  CHECK_EQ(image_message->encoding, sensor_msgs::image_encodings::TYPE_8UC3);

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(
        image_message, sensor_msgs::image_encodings::TYPE_8UC3);
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);

  *image = cv_ptr->image.clone();
}

void convertCameraInfo(
    sensor_msgs::CameraInfoConstPtr camera_info_msg,
    const geometry_msgs::Transform& T_C_I_msg,
    std::pair<aslam::Camera*, aslam::Transformation>* camera_with_extrinsics) {
  CHECK(camera_info_msg);
  CHECK_NOTNULL(camera_with_extrinsics);

  // Convert from geometry_msgs to Eigen to minkindr via transformation matrix
  // to make sure we don't skrew up any converntions.
  Eigen::Affine3d T_C_I_eigen;
  tf::transformMsgToEigen(T_C_I_msg, T_C_I_eigen);
  Eigen::Matrix4d T_C_I_mat = T_C_I_eigen.matrix();
  aslam::Transformation T_C_I(T_C_I_mat);

  LOG(INFO) << "Camera extrinsics - T_C_I: \n" << T_C_I;

  const size_t width = camera_info_msg->width;
  const size_t height = camera_info_msg->height;

  Eigen::Vector4d intrinsics;
  intrinsics[0] = camera_info_msg->K[0];
  intrinsics[1] = camera_info_msg->K[4];
  intrinsics[2] = camera_info_msg->K[2];
  intrinsics[3] = camera_info_msg->K[5];

  LOG(INFO) << "Camera intrinsics: " << intrinsics.transpose();

  aslam::Distortion::UniquePtr distortion;
  Eigen::VectorXd distortion_params;
  if (camera_info_msg->distortion_model ==
      sensor_msgs::distortion_models::PLUMB_BOB) {
    CHECK_GE(camera_info_msg->D.size(), 4u);
    if (camera_info_msg->D.size() > 4u) {
      LOG(WARNING)
          << "The camera info message has distortion type PLUMB_BOB with "
          << "more than 4 parameters. The aslam_cv distortion model only "
          << "supports 4 parameters and will use the first 4 values!";
    }

    distortion_params.resize(4u);
    distortion_params[0] = camera_info_msg->D[0];
    distortion_params[1] = camera_info_msg->D[1];
    distortion_params[2] = camera_info_msg->D[2];
    distortion_params[3] = camera_info_msg->D[3];

    LOG(INFO) << "Camera distortion params: " << distortion_params.transpose();

    const bool no_distortion = distortion_params.isZero(1e-10);
    if (no_distortion) {
      distortion.reset(new aslam::NullDistortion());
    } else {
      distortion.reset(new aslam::RadTanDistortion(distortion_params));
    }
  } else {
    LOG(FATAL) << "This distortion model is not supported by aslam_cv: "
               << camera_info_msg->distortion_model;
  }
  CHECK(distortion)
      << "Unable to convert camera info distortion to aslam distortion!";

  aslam::Camera::UniquePtr camera_ptr(
      new aslam::PinholeCamera(intrinsics, width, height, distortion));

  camera_with_extrinsics->first = camera_ptr->clone();
  camera_with_extrinsics->second = T_C_I;
}
