#ifndef RESOURCE_IMPORTER_MESSAGE_CONVERSION_H_
#define RESOURCE_IMPORTER_MESSAGE_CONVERSION_H_

#include <utility>

#include <aslam/cameras/camera.h>
#include <aslam/common/pose-types.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp>
#include <resources-common/point-cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

void convertDepthImageMessage(
    sensor_msgs::ImageConstPtr image_message, cv::Mat* image);

void convertFloatDepthImageMessage(
    sensor_msgs::ImageConstPtr image_message, cv::Mat* image);

void convertColorImageMessage(
    sensor_msgs::ImageConstPtr image_message, cv::Mat* image);

void convertCameraInfo(
    sensor_msgs::CameraInfoConstPtr camera_info_msg,
    const geometry_msgs::Transform& T_C_I_stamped,
    std::pair<aslam::Camera*, aslam::Transformation>* camera_with_extrinsics);

#endif  // RESOURCE_IMPORTER_MESSAGE_CONVERSION_H_
