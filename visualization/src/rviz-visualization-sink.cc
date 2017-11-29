#include "visualization/rviz-visualization-sink.h"

#include <glog/logging.h>

DEFINE_bool(
    rviz_wait_for_subscribers, false,
    "If true, every plotting message "
    "will wait a maximum of 5s for a subsrciber if none is visible yet.");

namespace visualization {

RVizVisualizationSink::RVizVisualizationSink()
    : is_initialized_(false),
      queue_size_(200u),
      latch_(true),
      should_wait_for_subscribers_(FLAGS_rviz_wait_for_subscribers) {
  initImpl();
}

void RVizVisualizationSink::initImpl() {
  if (is_initialized_) {
    return;
  }

  // Initialize ROS, if the host process doesn't initialize it itself.
  if (ros::isInitialized()) {
    VLOG(20) << "ROS is already initialized.";
  } else {
    int argv = 0;
    const std::string kNodeName = "maplab_rviz_interface";
    ros::init(
        argv, nullptr, kNodeName,
        ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  node_handle_.reset(new ros::NodeHandle());
  image_transport_.reset(new image_transport::ImageTransport(*node_handle_));

  is_initialized_ = true;
}

template <>
void RVizVisualizationSink::publishImpl(
    const std::string& topic, const cv::Mat& image) {
  CHECK(!topic.empty());

  // Create the ROS message (and copy the image).
  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now();

  if (image.channels() == 1) {
    sensor_msgs::fillImage(
        msg, sensor_msgs::image_encodings::MONO8, image.rows, image.cols,
        image.cols, image.data);
  } else if (image.channels() == 3) {
    sensor_msgs::fillImage(
        msg, sensor_msgs::image_encodings::BGR8, image.rows, image.cols,
        3 * image.cols, image.data);
  } else {
    LOG(FATAL)
        << "Unsupported image type. The numbers of image channels needs "
        << " to be equal to 1 for grayscale, or equal to 3 for color images."
        << " But the given image has " << image.channels() << " channels.";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Check whether this topic was already registered, otherwise do so.
    TopicToImagePublisherMap::iterator topic_iterator =
        topic_to_image_publisher_map_.find(topic);

    if (topic_iterator == topic_to_image_publisher_map_.end()) {
      // Add a new publisher.
      CHECK(node_handle_);
      CHECK(image_transport_);
      topic_iterator =
          topic_to_image_publisher_map_
              .insert(
                  std::make_pair(
                      topic, image_transport_->advertise(topic, 1, latch_)))
              .first;
    }
    CHECK(topic_iterator != topic_to_image_publisher_map_.end());

    // Publish image.
    topic_iterator->second.publish(msg);
  }
}
}  // namespace visualization
