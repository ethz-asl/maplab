#ifndef ASLAM_CV_COMMON_CHANNEL_DECLARATIONS_H_
#define ASLAM_CV_COMMON_CHANNEL_DECLARATIONS_H_
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <aslam/common/channel.h>
#include <aslam/common/macros.h>

#define DECLARE_CHANNEL_IMPL(NAME, TYPE)                                   \
namespace aslam {                                                          \
namespace channels {                                                       \
                                                                           \
struct NAME : Channel<GET_TYPE(TYPE)> {                                    \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW                                          \
  typedef typename GET_TYPE(TYPE) Type;                                    \
  virtual std::string name() const { return #NAME; }                       \
};                                                                         \
                                                                           \
const std::string NAME##_CHANNEL = #NAME;                                  \
typedef GET_TYPE(TYPE) NAME##_ChannelValueType;                            \
typedef Channel<NAME##_ChannelValueType> NAME##_ChannelType;               \
                                                                           \
NAME##_ChannelValueType& get_##NAME##_Data(                                \
    const ChannelGroup& channel_group) {                                   \
  std::lock_guard<std::mutex> lock(channel_group.m_channels_);             \
  const ChannelMap& channels = channel_group.channels_;                    \
  ChannelMap::const_iterator it = channels.find(NAME##_CHANNEL);           \
  CHECK(it != channels.end()) << "Channelgroup does not "                  \
      "contain channel " << NAME##_CHANNEL;                                \
  std::shared_ptr<NAME##_ChannelType> derived =                            \
     std::dynamic_pointer_cast<NAME##_ChannelType>(it->second);            \
  CHECK(derived) << "Channel cast to derived failed " <<                   \
     "channel: " << NAME##_CHANNEL;                                        \
  return derived->value_;                                                  \
}                                                                          \
                                                                           \
void serialize_##NAME##_Channel(                                           \
    const ChannelGroup& channel_group, std::string* data) {                \
  std::lock_guard<std::mutex> lock(channel_group.m_channels_);             \
  const ChannelMap& channels = channel_group.channels_;                    \
  ChannelMap::const_iterator it = channels.find(NAME##_CHANNEL);           \
  CHECK(it != channels.end()) << "Channelgroup does not "                  \
      "contain channel " << NAME##_CHANNEL;                                \
  std::shared_ptr<NAME##_ChannelType> derived =                            \
     std::dynamic_pointer_cast<NAME##_ChannelType>(it->second);            \
  CHECK(derived) << "Channel cast to derived failed " <<                   \
     "channel: " << NAME##_CHANNEL;                                        \
  derived->serializeToString(data);                                        \
}                                                                          \
                                                                           \
void deserialize_##NAME##_Channel(                                         \
    const ChannelGroup& channel_group, const std::string& data) {          \
  std::lock_guard<std::mutex> lock(channel_group.m_channels_);             \
  const ChannelMap& channels = channel_group.channels_;                    \
  ChannelMap::const_iterator it = channels.find(NAME##_CHANNEL);           \
  CHECK(it != channels.end()) << "Channelgroup does not "                  \
      "contain channel " << NAME##_CHANNEL;                                \
  std::shared_ptr<NAME##_ChannelType> derived =                            \
     std::dynamic_pointer_cast<NAME##_ChannelType>(it->second);            \
  CHECK(derived) << "Channel cast to derived failed " <<                   \
     "channel: " << NAME##_CHANNEL;                                        \
  derived->deSerializeFromString(data);                                    \
}                                                                          \
                                                                           \
NAME##_ChannelValueType& add_##NAME##_Channel(                             \
    ChannelGroup* channel_group) {                                         \
  CHECK_NOTNULL(channel_group);                                            \
  std::lock_guard<std::mutex> lock(channel_group->m_channels_);            \
  ChannelMap* channels = &channel_group->channels_;                        \
  ChannelMap::iterator it = channels->find(NAME##_CHANNEL);                \
  CHECK(it == channels->end()) << "Channelgroup already "                  \
      "contains channel " << NAME##_CHANNEL;                               \
  std::shared_ptr<NAME##_ChannelType> derived(new NAME##_ChannelType);     \
  (*channels)[NAME##_CHANNEL] = derived;                                   \
  return derived->value_;                                                  \
}                                                                          \
                                                                           \
bool has_##NAME##_Channel(const ChannelGroup& channel_group) {             \
  std::lock_guard<std::mutex> lock(channel_group.m_channels_);             \
  const ChannelMap& channels = channel_group.channels_;                    \
  ChannelMap::const_iterator it = channels.find(NAME##_CHANNEL);           \
  return it != channels.end();                                             \
}                                                                          \
                                                                           \
void remove_##NAME##_Channel(ChannelGroup* channel_group) {                \
  CHECK_NOTNULL(channel_group);                                            \
  std::lock_guard<std::mutex> lock(channel_group->m_channels_);            \
  ChannelMap& channels = channel_group->channels_;                         \
  CHECK_EQ(channels.erase(NAME##_CHANNEL), 1u)                             \
    << "Channelgroup does not contain channel " << NAME##_CHANNEL;         \
}                                                                          \
}                                                                          \
}                                                                          \

// Wrap types that contain commas inside braces.
#define DECLARE_CHANNEL(x, ...) DECLARE_CHANNEL_IMPL(x, (__VA_ARGS__))

namespace aslam {
namespace channels {
template<typename CHANNEL_DATA_TYPE>
CHANNEL_DATA_TYPE& getChannelData(const std::string& channel_name,
                                  const ChannelGroup& channel_group) {
  std::lock_guard<std::mutex> lock(channel_group.m_channels_);
  ChannelMap::const_iterator it = channel_group.channels_.find(channel_name);
  CHECK(it != channel_group.channels_.end()) << "Channelgroup does not "
      "contain channel " << channel_name;
  typedef Channel<CHANNEL_DATA_TYPE> DerivedChannel;
  std::shared_ptr<DerivedChannel> derived =
      std::dynamic_pointer_cast < DerivedChannel > (it->second);
  CHECK(derived) << "Channel cast to derived failed " <<
                    "channel: " << channel_name;
  return derived->value_;
}

inline bool hasChannel(const std::string& channel_name,
                       const ChannelGroup& channel_group) {
  std::lock_guard<std::mutex> lock(channel_group.m_channels_);
  ChannelMap::const_iterator it = channel_group.channels_.find(channel_name);
  return it != channel_group.channels_.end();
}

template<typename CHANNEL_DATA_TYPE>
CHANNEL_DATA_TYPE& addChannel(const std::string& channel_name,
                              ChannelGroup* channel_group) {
  CHECK_NOTNULL(channel_group);
  std::lock_guard<std::mutex> lock(channel_group->m_channels_);
  ChannelMap::iterator it = channel_group->channels_.find(channel_name);
  CHECK(it == channel_group->channels_.end()) << "Channelgroup already "
      "contains channel " << channel_name;
  typedef Channel<CHANNEL_DATA_TYPE> DerivedChannel;
  std::shared_ptr < DerivedChannel > derived(new DerivedChannel);
  channel_group->channels_[channel_name] = derived;
  return derived->value_;
}
}  // namespace channels
}  // namespace aslam

#endif  // ASLAM_CV_COMMON_CHANNEL_DECLARATIONS_H_
