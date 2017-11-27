#ifndef VISUALIZATION_RVIZ_VISUALIZATION_SINK_INL_H_
#define VISUALIZATION_RVIZ_VISUALIZATION_SINK_INL_H_

#include <string>
#include <unordered_map>

#include <glog/logging.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

namespace visualization {

template <typename T>
void RVizVisualizationSink::publishImpl(
    const std::string& topic, const T& message) {
  CHECK(!topic.empty());

  std::unique_lock<std::mutex> lock(mutex_);

  TopicToPublisherMap::iterator topic_iterator =
      topic_to_publisher_map_.find(topic);

  if (topic_iterator == topic_to_publisher_map_.end()) {
    // Add a new publisher.
    CHECK(node_handle_);
    const ros::Publisher publisher =
        node_handle_->advertise<T>(topic, queue_size_, latch_);
    topic_iterator = topic_to_publisher_map_.emplace(topic, publisher).first;
  }
  CHECK(topic_iterator != topic_to_publisher_map_.end());
  const ros::Publisher& publisher = topic_iterator->second;

  if (should_wait_for_subscribers_) {
    lock.unlock();
    // This waits for RViz to subscribe - otherwise this program might
    // terminate before RViz receives the messages.
    const double kTimeoutSeconds = 5.0;
    const double kLoopFrequencyHz = 2.0;
    const size_t num_loops =
        static_cast<size_t>(std::ceil(kTimeoutSeconds * kLoopFrequencyHz));
    ros::Rate loop_rate(kLoopFrequencyHz);
    size_t i = 0u;
    while (publisher.getNumSubscribers() < 1u) {
      LOG_FIRST_N(INFO, 1) << "Waiting for a subscriber on topic "
                           << publisher.getTopic();

      ros::spinOnce();
      loop_rate.sleep();
      if (++i > num_loops) {
        VLOG(1) << "Timeout. Publishing the message although there's not yet a "
                << "subscriber and continuing.";
        break;
      }
    }
    lock.lock();
  }
  VLOG(10) << "Publishing message on topic " << publisher.getTopic();
  publisher.publish(message);
}
}  // namespace visualization

#endif  // VISUALIZATION_RVIZ_VISUALIZATION_SINK_INL_H_
