#ifndef MAPLAB_ROS_COMMON_ROSBAG_TOOLS_H_
#define MAPLAB_ROS_COMMON_ROSBAG_TOOLS_H_

#include <functional>
#include <string>

#include <rosbag/message_instance.h>

namespace ros_common {

typedef std::function<void(const rosbag::MessageInstance&)>
    MessageInstanceCallback;

// Iterates through all messages in the rosbag and invokes the given callback
// for all of them.
void forEachMessageInBag(
    const std::string& bag_name, const MessageInstanceCallback& callback);

template <typename MessageType>
using MessageCallback = std::function<void(
    const uint64_t timestamp_nsec, const std::string& topic_name,
    const MessageType& message)>;

// Iterates through all messages in the rosbag and invokes the given callback
// for all messages with a matching message type.
template <typename MessageType>
void forSpecifiedMessageTypesInBag(
    const std::string& bag_name, const MessageCallback<MessageType>& callback);

template <typename MessageType, typename... OtherMessageTypeCallbacks>
void forSpecifiedMessageTypesInBag(
    const std::string& bag_name, const MessageCallback<MessageType>& callback,
    OtherMessageTypeCallbacks... other_callbacks);

}  // namespace ros_common

#include "maplab-ros-common/implementation/rosbag-tools.h"

#endif  // MAPLAB_ROS_COMMON_ROSBAG_TOOLS_H_
