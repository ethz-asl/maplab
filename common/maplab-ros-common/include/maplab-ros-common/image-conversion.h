#ifndef MAPLAB_ROS_COMMON_IMAGE_CONVERSION_H_
#define MAPLAB_ROS_COMMON_IMAGE_CONVERSION_H_

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>

namespace ros_common {

inline void imageMsgToCvMat(
    const sensor_msgs::Image& image_msg, const std::string image_encoding,
    cv::Mat* cv_mat) {
  CHECK(!image_encoding.empty());
  CHECK_NOTNULL(cv_mat);
  cv_bridge::CvImageConstPtr cv_bridge_image;
  try {
    cv_bridge_image = cv_bridge::toCvCopy(image_msg, image_encoding);
  } catch (const cv_bridge::Exception& e) {
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_bridge_image);
  *cv_mat = cv_bridge_image->image;
}

inline void imageMsgToCvMat(
    const sensor_msgs::Image& image_msg, cv::Mat* cv_mat) {
  CHECK_NOTNULL(cv_mat);
  const std::string& image_encoding = image_msg.encoding;
  imageMsgToCvMat(image_msg, image_encoding, cv_mat);
}

}  // namespace ros_common

#endif  // MAPLAB_ROS_COMMON_IMAGE_CONVERSION_H_
