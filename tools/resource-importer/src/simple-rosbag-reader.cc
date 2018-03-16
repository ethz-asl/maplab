#include "resource-importer/simple-rosbag-reader.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
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
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

SimpleRosbagSource::SimpleRosbagSource(
    const std::string& rosbag_filename, const std::string& rostopic,
    const std::string& rostopic_camera_info, const std::string& imu_frame,
    const std::string& camera_frame)
    : bag_(new rosbag::Bag),
      bag_view_(nullptr),
      rosbag_filename_(rosbag_filename),
      resource_topic_(rostopic),
      camera_info_topic_(rostopic_camera_info),
      rosbag_imu_frame_(imu_frame),
      rosbag_camera_frame_(camera_frame) {
  openRosbag();
}

void SimpleRosbagSource::openRosbag() {
  CHECK(bag_);
  try {
    bag_->open(rosbag_filename_, rosbag::bagmode::Read);
  } catch (const std::exception& ex) {  // NOLINT
    LOG(FATAL) << "Could not open the rosbag " << rosbag_filename_ << ": "
               << ex.what();
  }

  std::vector<std::string> topics;

  CHECK(!resource_topic_.empty());
  topics.push_back(resource_topic_);

  // If we didn't get a camera info topic we are most likely loading the camera
  // calibration from an NCamera yaml, so no need to listen to any of these
  // topics.
  if (!camera_info_topic_.empty()) {
    topics.push_back(camera_info_topic_);
    topics.push_back("/tf");
    topics.push_back("/tf_static");
  }

  try {
    CHECK(bag_);
    bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(topics)));
  } catch (const std::exception& ex) {  // NOLINT
    LOG(FATAL) << "Could not open a rosbag view: " << ex.what();
  }
  CHECK(bag_view_);
}

void SimpleRosbagSource::readRosbag() {
  CHECK(bag_view_);

  bool extracted_camera_info = false;
  bool extracted_camera_extrinsics = false;

  rosbag::View::iterator it_message = bag_view_->begin();
  while (it_message != bag_view_->end()) {
    const rosbag::MessageInstance& message = *it_message;
    const std::string& topic = message.getTopic();
    CHECK(!topic.empty());

    if (topic == "/tf" || topic == "/tf_static") {
      LOG(INFO) << "Found tf message";

      boost::shared_ptr<tf::tfMessage> tf_messages =
          message.instantiate<tf::tfMessage>();
      if (tf_messages && !extracted_camera_extrinsics) {
        ros::Time timestamp_ros;

        LOG(INFO) << "Inserting TF messages into transformer...";

        for (const geometry_msgs::TransformStamped& transform_stamped :
             tf_messages->transforms) {
          timestamp_ros = transform_stamped.header.stamp;
          tf::StampedTransform T_C_I_msg_tf;
          tf::transformStampedMsgToTF(transform_stamped, T_C_I_msg_tf);

          LOG(INFO) << "Inserting TF messages from "
                    << transform_stamped.child_frame_id << " to "
                    << transform_stamped.header.frame_id << " at "
                    << timestamp_ros;

          tf_transformer_.setTransform(T_C_I_msg_tf);
        }

        LOG(INFO) << "Checking if camera extrinsics are present...";

        // NOTE: we don't care about the timestamp at all, because we assume
        // the camera extrinsics are a static transform.
        if (tf_transformer_.canTransform(
                rosbag_camera_frame_, rosbag_imu_frame_, timestamp_ros)) {
          tf::StampedTransform T_C_I_msg_tf;
          tf_transformer_.lookupTransform(
              rosbag_imu_frame_, rosbag_camera_frame_, timestamp_ros,
              T_C_I_msg_tf);

          geometry_msgs::TransformStamped T_C_I_msg;
          tf::transformStampedTFToMsg(T_C_I_msg_tf, T_C_I_msg);

          CHECK(camera_extrinsics_lambda_);
          camera_extrinsics_lambda_(T_C_I_msg.transform);
          extracted_camera_extrinsics = true;
        } else {
          LOG(INFO) << "Camera extrinsics not found!";
        }
        ++it_message;
        continue;
      } else {
        // If we already extracted the camera extrinsics once, there is no need
        // to do it again. Skipping this message.
        ++it_message;
        continue;
      }
      LOG(FATAL) << "[camera extrinsics topic] Unsupported ROS message type.";
    } else if (topic == resource_topic_) {
      sensor_msgs::ImageConstPtr image_message =
          message.instantiate<sensor_msgs::Image>();
      if (image_message) {
        CHECK(image_lambda_);
        image_lambda_(image_message);
        ++it_message;
        continue;
      }

      sensor_msgs::PointCloud2ConstPtr pointcloud_message =
          message.instantiate<sensor_msgs::PointCloud2>();
      if (pointcloud_message) {
        CHECK(pointcloud_lambda_);
        pointcloud_lambda_(pointcloud_message);
        ++it_message;
        continue;
      }

      LOG(FATAL) << "[resource ros topic] Unsupported ROS message type.";

    } else if (topic == camera_info_topic_) {
      sensor_msgs::CameraInfoConstPtr camera_info_msg =
          message.instantiate<sensor_msgs::CameraInfo>();
      if (camera_info_msg && !extracted_camera_info) {
        CHECK(camera_info_lambda_);
        camera_info_lambda_(camera_info_msg);
        extracted_camera_info = true;
        ++it_message;
        continue;
      } else if (extracted_camera_info) {
        // If we already extracted the camera info once, there is no need to
        // do it again. Skipping this message.
        ++it_message;
        continue;
      }

      LOG(FATAL) << "[camera info ros topic] Unsupported ROS message type.";

    } else {
      LOG(FATAL) << "Unknown ROS topic: " << topic;
    }
  }
}

void SimpleRosbagSource::setImageCallback(
    const std::function<void(sensor_msgs::ImageConstPtr)>& callback) {
  image_lambda_ = callback;
}

void SimpleRosbagSource::setPointcloudCallback(
    const std::function<void(sensor_msgs::PointCloud2ConstPtr)>& callback) {
  pointcloud_lambda_ = callback;
}

void SimpleRosbagSource::setCameraInfoCallback(
    const std::function<void(sensor_msgs::CameraInfoConstPtr)>& callback) {
  camera_info_lambda_ = callback;
}

void SimpleRosbagSource::setCameraExtrinsicsCallback(
    const std::function<void(geometry_msgs::Transform)>& callback) {
  camera_extrinsics_lambda_ = callback;
}
