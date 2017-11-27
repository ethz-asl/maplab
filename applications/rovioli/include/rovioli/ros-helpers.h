#ifndef ROVIOLI_ROS_HELPERS_H_
#define ROVIOLI_ROS_HELPERS_H_

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

namespace rovioli {

inline int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
  return aslam::time::from_seconds(static_cast<int64_t>(rostime.sec)) +
         static_cast<int64_t>(rostime.nsec);
}

inline vio::ImuMeasurement::Ptr convertRosImuToMaplabImu(
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

inline vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx) {
  CHECK(image_message);
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // Convert the image to MONO8 if necessary.
    cv_ptr = cv_bridge::toCvShare(
        image_message, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);

  vio::ImageMeasurement::Ptr image_measurement(new vio::ImageMeasurement);
  // Unfortunately the clone is necessary as ROS stores the data in an
  // uint8_t array inside CvImage and not the cv::Mat itself. Keeping the
  // data alive without holding on to the CvImage is not possible without
  // letting all the code depend on ROS and boost.
  image_measurement->image = cv_ptr->image.clone();
  image_measurement->timestamp =
      rosTimeToNanoseconds(image_message->header.stamp);
  image_measurement->camera_index = camera_idx;
  return image_measurement;
}

}  // namespace rovioli

#endif  // ROVIOLI_ROS_HELPERS_H_
