#ifndef INTERNAL_VIZ_DATA_COLLECTOR_INL_H_
#define INTERNAL_VIZ_DATA_COLLECTOR_INL_H_

#include <mutex>
#include <string>
#include <unordered_map>

namespace visualization {
namespace internal {

template <typename DataType>
void VizDataCollectorImpl::pushData(
    const SlotId& slot_id, const std::string& channel_name,
    const DataType& data) {
  VizChannelGroup* slot = getSlotAndCreateIfNecessary(slot_id);
  CHECK_NOTNULL(slot);
  slot->setChannel(channel_name, data);
}

template <typename DataType>
bool VizDataCollectorImpl::getDataSafe(
    const SlotId& slot_id, const std::string& channel_name,
    const DataType** data) const {
  const VizChannelGroup* slot;
  if ((slot = getSlot(slot_id)) == nullptr) {
    return false;
  }
  return slot->getChannelSafe(channel_name, data);
}

template <typename DataType>
std::string VizDataCollectorImpl::printData(
    const SlotId& slot_id, const std::string& channel_name) const {
  std::ostringstream out;
  const DataType* data;
  if (getDataSafe(slot_id, channel_name, &data)) {
    out << std::setprecision(5) << *CHECK_NOTNULL(data) << std::fixed;
  } else {
    out << "Channel not available.";
  }
  return out.str();
}

}  // namespace internal
}  // namespace visualization
#endif  // INTERNAL_VIZ_DATA_COLLECTOR_INL_H_
