#ifndef MAPLAB_COMMON_NETWORK_COMMON_H_
#define MAPLAB_COMMON_NETWORK_COMMON_H_

#include <functional>
#include <utility>
#include <vector>

#include <glog/logging.h>

namespace network {

typedef std::pair<void*, size_t> RawMessageData;  // Raw byte array and length.
typedef std::vector<RawMessageData> RawMessageDataList;
typedef std::function<void(const RawMessageDataList&)> MessageCallback;

const MessageCallback kEmptyMessageCallback =
    [](const RawMessageDataList& /*raw_data*/) {};  // NOLINT

/// Returns the element at index \p index if \p raw_data_list contains enough
/// elements. If there are less elements in the list, the function crashes.
inline const network::RawMessageData& checkAndGetEntry(
    const network::RawMessageDataList& raw_data_list, const size_t index) {
  CHECK_GT(raw_data_list.size(), index);
  return raw_data_list[index];
}

inline void copyRawMessageDataList(
    const RawMessageDataList& input, RawMessageDataList* output) {
  CHECK_NOTNULL(output);
  for (const RawMessageData& message_part : input) {
    void* raw_data_copy = new uint8_t[message_part.second];
    memcpy(raw_data_copy, message_part.first, message_part.second);
    output->emplace_back(raw_data_copy, message_part.second);
  }
}

}  // namespace network

#endif  // MAPLAB_COMMON_NETWORK_COMMON_H_
