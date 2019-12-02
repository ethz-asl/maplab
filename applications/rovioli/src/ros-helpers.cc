#include "rovioli/ros-helpers.h"
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop

#include <vio-common/vio-types.h>

DEFINE_bool(
    rovioli_image_apply_clahe_histogram_equalization, false,
    "Apply CLAHE histogram equalization to image.");
DEFINE_int32(
    rovioli_image_clahe_clip_limit, 2,
    "CLAHE histogram equalization parameter: clip limit.");
DEFINE_int32(
    rovioli_image_clahe_grid_size, 8,
    "CLAHE histogram equalization parameter: grid size.");

DEFINE_double(
    rovioli_image_16_bit_to_8_bit_scale_factor, 1,
    "Scale factor applied to 16bit images when converting them to 8bit "
    "images.");
DEFINE_double(
    rovioli_image_16_bit_to_8_bit_shift, 0,
    "Shift applied to the scaled values when converting 16bit images to 8bit "
    "images.");

namespace rovioli {
vio::ImuMeasurement::Ptr convertRosImuToMaplabImu(
    const sensor_msgs::ImuConstPtr& imu_msg) {
  CHECK(imu_msg);
  vio::ImuMeasurement::Ptr imu_measurement(new vio::ImuMeasurement);
  imu_measurement->timestamp = rosTimeToNanoseconds(imu_msg->header.stamp);
  imu_measurement->imu_data << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
      imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
      imu_msg->angular_velocity.z;
  return imu_measurement;
}

void applyHistogramEqualization(
    const cv::Mat& input_image, cv::Mat* output_image) {
  CHECK_NOTNULL(output_image);
  CHECK(!input_image.empty());

  static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
      FLAGS_rovioli_image_clahe_clip_limit,
      cv::Size(
          FLAGS_rovioli_image_clahe_grid_size,
          FLAGS_rovioli_image_clahe_grid_size));
  clahe->apply(input_image, *output_image);
}

vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx) {
  CHECK(image_message);
  cv_bridge::CvImageConstPtr cv_ptr;
  vio::ImageMeasurement::Ptr image_measurement(new vio::ImageMeasurement);
  try {
    // 16bit images are treated differently, we will apply histogram
    // equalization first and then convert them to MONO8;
    if (image_message->encoding == sensor_msgs::image_encodings::MONO16) {
      cv_ptr = cv_bridge::toCvShare(
          image_message, sensor_msgs::image_encodings::MONO16);
      CHECK(cv_ptr);

      cv::Mat processed_image;
      if (FLAGS_rovioli_image_apply_clahe_histogram_equalization) {
        applyHistogramEqualization(cv_ptr->image, &processed_image);
      } else {
        processed_image = cv_ptr->image;
      }
      processed_image.convertTo(
          image_measurement->image, CV_8U,
          FLAGS_rovioli_image_16_bit_to_8_bit_scale_factor,
          FLAGS_rovioli_image_16_bit_to_8_bit_shift);
    } else {
      if (image_message->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
        // NOTE: we assume all 8UC1 type images are monochrome images.
        cv_ptr = cv_bridge::toCvShare(
            image_message, sensor_msgs::image_encodings::TYPE_8UC1);
      } else {
        cv_ptr = cv_bridge::toCvShare(
            image_message, sensor_msgs::image_encodings::MONO8);
      }
      CHECK(cv_ptr);

      if (FLAGS_rovioli_image_apply_clahe_histogram_equalization) {
        cv::Mat processed_image;
        applyHistogramEqualization(cv_ptr->image, &processed_image);
        image_measurement->image = processed_image;
      } else {
        image_measurement->image = cv_ptr->image.clone();
      }
    }
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);

  image_measurement->timestamp =
      rosTimeToNanoseconds(image_message->header.stamp);
  image_measurement->camera_index = camera_idx;
  return image_measurement;
}

vio::OdometryMeasurement::Ptr convertRosOdometryToOdometry(
    const nav_msgs::OdometryConstPtr& odometry_msg) {
  CHECK(odometry_msg);
  vio::OdometryMeasurement::Ptr odometry_measurement(
      new vio::OdometryMeasurement);
  odometry_measurement->timestamp =
      rosTimeToNanoseconds(odometry_msg->header.stamp);

  const geometry_msgs::Twist& twist = odometry_msg->twist.twist;
  odometry_measurement->velocity_linear_O << twist.linear.x, twist.linear.y,
      twist.linear.z;
  odometry_measurement->velocity_angular_O << twist.angular.x, twist.angular.y,
      twist.angular.z;

  return odometry_measurement;
}

void odometryCovarianceToEigenMatrix(
    geometry_msgs::PoseWithCovariance::_covariance_type&
        odometry_msg_covariance,
    Eigen::Matrix<double, 6, 6>& covariance) {  // NOLINT
  covariance =
      Eigen::Map<Eigen::MatrixXd>(odometry_msg_covariance.data(), 6, 6);
}

void eigenMatrixToOdometryCovariance(
    const Eigen::Matrix<double, 6, 6>& covariance,
    double* odometry_msg_covariance_data) {
  Eigen::Map<Eigen::MatrixXd>(odometry_msg_covariance_data, 6, 6) = covariance;
}

}  // namespace rovioli
