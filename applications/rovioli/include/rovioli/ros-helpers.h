#ifndef ROVIOLI_ROS_HELPERS_H_
#define ROVIOLI_ROS_HELPERS_H_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop

#include <vio-common/vio-types.h>

namespace rovioli {

constexpr int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
  return aslam::time::seconds(static_cast<int64_t>(rostime.sec)) +
         static_cast<int64_t>(rostime.nsec);
}

vio::ImuMeasurement::Ptr convertRosImuToMaplabImu(
    const sensor_msgs::ImuConstPtr& imu_msg);

void applyHistogramEqualization(
    const cv::Mat& input_image, cv::Mat* output_image);

vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx);

vio::OdometryMeasurement::Ptr convertRosOdometryToOdometry(
    const nav_msgs::OdometryConstPtr& odometry_msg);

void odometryCovarianceToEigenMatrix(
    geometry_msgs::PoseWithCovariance::_covariance_type&
        odometry_msg_covariance,
    Eigen::Matrix<double, 6, 6>& covariance);  // NOLINT

void eigenMatrixToOdometryCovariance(
    const Eigen::Matrix<double, 6, 6>& covariance,
    double* odometry_msg_covariance_data);

}  // namespace rovioli

#endif  // ROVIOLI_ROS_HELPERS_H_
