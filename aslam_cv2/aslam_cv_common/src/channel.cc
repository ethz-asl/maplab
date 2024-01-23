#include <string>
#include <unordered_map>

#include <aslam/common/channel.h>
#include <aslam/common/meta.h>

namespace aslam {
namespace channels {

template<>
bool Channel<cv::Mat>::operator==(const Channel<cv::Mat>& other) {
  return cv::countNonZero(value_ != other.value_) == 0;
}

ChannelGroup cloneChannelGroup(const ChannelGroup& channels) {
  std::lock_guard<std::mutex> lock(channels.m_channels_);
  ChannelGroup cloned_group;
  for (const ChannelMap::value_type& channel : channels.channels_) {
    CHECK(channel.second);
    cloned_group.channels_.emplace(channel.first,
                                   std::shared_ptr<ChannelBase>(channel.second->clone()));
  }
  return cloned_group;
}

bool isChannelGroupEqual(const ChannelGroup& left_channels, const ChannelGroup& right_channels) {
  // Early exit if both groups are the same.
  if (&left_channels == &right_channels) {
    return true;
  }
  std::lock_guard<std::mutex> lock_left(left_channels.m_channels_);
  std::lock_guard<std::mutex> lock_right(right_channels.m_channels_);

  if (left_channels.channels_.size() != right_channels.channels_.size()) {
    return false;
  }
  for (const ChannelMap::value_type& left_channel_pair : left_channels.channels_) {
    ChannelMap::const_iterator it_right = right_channels.channels_.find(left_channel_pair.first);
    if (it_right == right_channels.channels_.end()) {
      return false;
    }
    if (!CHECK_NOTNULL(it_right->second.get())->compare(*left_channel_pair.second)) {
      return false;
    }
  }
  return true;
}

}  // namespace channels
}  // namespace aslam
