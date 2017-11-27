#ifndef VISUALIZATION_RVIZ_VISUALIZATION_SINK_H_
#define VISUALIZATION_RVIZ_VISUALIZATION_SINK_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

namespace visualization {

// RVizVisualizationSink
// This singleton class can be used to conveniently publish messages
// to ROS topics. The class takes care of the ROS node and publisher
// registration.
//
//        Example usage:
//         cv::Mat my_image = getSomeAwsomeImage();
//         visualization::RVizVisualizationSink::publish("image_topic",
//         my_image);
//         visualization_msgs::Marker marker
//         visualization::RVizVisualizationSink::publish("topic", marker)
class RVizVisualizationSink {
 public:
  explicit RVizVisualizationSink(const RVizVisualizationSink&) = delete;
  void operator=(const RVizVisualizationSink&) = delete;

  static inline void init() {
    RVizVisualizationSink::getInstance();
  }

  template <typename T>
  static inline void publish(const std::string& topic, const T& message) {
    RVizVisualizationSink::getInstance().publishImpl<T>(topic, message);
  }

  // Singleton instance
  static inline RVizVisualizationSink& getInstance() {
    static RVizVisualizationSink instance;
    return instance;
  };

 private:
  RVizVisualizationSink();
  ~RVizVisualizationSink() = default;

  template <typename T>
  inline void publishImpl(const std::string& topic, const T& message);

  void initImpl();

  std::unique_ptr<ros::NodeHandle> node_handle_;
  std::unique_ptr<image_transport::ImageTransport> image_transport_;

  typedef std::unordered_map<std::string, ros::Publisher> TopicToPublisherMap;
  TopicToPublisherMap topic_to_publisher_map_;

  typedef std::unordered_map<std::string, image_transport::Publisher>
      TopicToImagePublisherMap;
  TopicToImagePublisherMap topic_to_image_publisher_map_;

  bool is_initialized_;

  // Maximum number of outgoing messages to be queued for delivery to
  // subscribers.
  const size_t queue_size_;

  // If true, the last message published on this topic will be saved and sent
  // to new subscribers when they connect.
  const bool latch_;

  const bool should_wait_for_subscribers_;

  std::mutex mutex_;
};

template <>
void RVizVisualizationSink::publishImpl(
    const std::string& topic, const cv::Mat& image);

}  // namespace visualization

#include "visualization/rviz-visualization-sink-inl.h"

#endif  // VISUALIZATION_RVIZ_VISUALIZATION_SINK_H_
