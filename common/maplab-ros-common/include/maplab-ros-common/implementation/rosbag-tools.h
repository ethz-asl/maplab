#ifndef MAPLAB_ROS_COMMON_IMPLEMENTATION_ROSBAG_TOOLS_H_
#define MAPLAB_ROS_COMMON_IMPLEMENTATION_ROSBAG_TOOLS_H_

#include <string>

#include <boost/shared_ptr.hpp>
#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace ros_common {

void forEachMessageInBag(
    const std::string& bag_name, const MessageInstanceCallback& callback) {
  CHECK(!bag_name.empty());
  try {
    const rosbag::Bag bag(bag_name, rosbag::bagmode::Read);
    rosbag::View bag_view(bag);
    for (const rosbag::MessageInstance& message_instance : bag_view) {
      callback(message_instance);
    }
  } catch (const std::exception& ex) {
    LOG(FATAL) << "Could not open the rosbag " << bag_name << ": " << ex.what();
  }
}

template <typename MessageType>
void invokeCallbackIfMessageIsMatching(
    const rosbag::MessageInstance& message_instance,
    const MessageCallback<MessageType>& callback) {
  CHECK(callback);
  if (message_instance.isType<MessageType>()) {
    const uint64_t timestamp_nsec = message_instance.getTime().toNSec();
    const std::string& topic_name = message_instance.getTopic();
    CHECK(!topic_name.empty());
    const boost::shared_ptr<MessageType> message_ptr =
        message_instance.instantiate<MessageType>();
    CHECK(message_ptr);
    callback(timestamp_nsec, topic_name, *message_ptr.get());
  }
}

template <typename MessageType, typename... OtherMessageTypeCallbacks>
void invokeCallbackIfMessageIsMatching(
    const rosbag::MessageInstance& message_instance,
    const MessageCallback<MessageType>& callback,
    OtherMessageTypeCallbacks... other_callbacks) {
  CHECK(callback);
  invokeCallbackIfMessageIsMatching(message_instance, callback);
  invokeCallbackIfMessageIsMatching(message_instance, other_callbacks...);
}

template <typename MessageType>
void forSpecifiedMessageTypesInBag(
    const std::string& bag_name, const MessageCallback<MessageType>& callback) {
  CHECK(!bag_name.empty());
  const MessageInstanceCallback message_instance_callback =
      [&](const rosbag::MessageInstance& message_instance) {
        invokeCallbackIfMessageIsMatching(message_instance, callback);
      };
  forEachMessageInBag(bag_name, message_instance_callback);
}

template <typename MessageType, typename... OtherMessageTypeCallbacks>
void forSpecifiedMessageTypesInBag(
    const std::string& bag_name, const MessageCallback<MessageType>& callback,
    OtherMessageTypeCallbacks... other_callbacks) {
  CHECK(!bag_name.empty());
  const MessageInstanceCallback message_instance_callback =
      [&](const rosbag::MessageInstance& message_instance) {
        invokeCallbackIfMessageIsMatching(
            message_instance, callback, other_callbacks...);
      };
  forEachMessageInBag(bag_name, message_instance_callback);
}

}  // namespace ros_common

#endif  // MAPLAB_ROS_COMMON_IMPLEMENTATION_ROSBAG_TOOLS_H_
