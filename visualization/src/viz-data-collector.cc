#include "visualization/viz-data-collector.h"

#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>

#include <Eigen/Core>
#include <aslam/common/unique-id.h>
#include <maplab-common/accessors.h>
#include <maplab-common/macros.h>

namespace visualization {
namespace internal {
const VizDataCollectorImpl::SlotId VizDataCollectorImpl::kCommonSlotId =
    VizDataCollectorImpl::SlotId::Random();

VizDataCollectorImpl::VizChannelGroup* VizDataCollectorImpl::getSlot(
    const SlotId& slot_id) {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);
  return common::getValuePtr(channel_groups_, slot_id);
}

const VizDataCollectorImpl::VizChannelGroup* VizDataCollectorImpl::getSlot(
    const SlotId& slot_id) const {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);
  return common::getValuePtr(channel_groups_, slot_id);
}

VizDataCollectorImpl::VizChannelGroup*
VizDataCollectorImpl::getSlotAndCreateIfNecessary(const SlotId& slot_id) {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);
  SlotIdSlotMap::iterator iterator;
  bool inserted = false;
  std::tie(iterator, inserted) = channel_groups_.emplace(
      std::piecewise_construct, std::make_tuple(slot_id), std::make_tuple());
  return &iterator->second;
}

void VizDataCollectorImpl::removeSlotIfAvailable(const SlotId& slot_id) {
  CHECK(slot_id.isValid());
  std::lock_guard<std::mutex> lock(m_channel_groups_);
  channel_groups_.erase(slot_id);
}

bool VizDataCollectorImpl::hasSlot(const SlotId& slot_id) const {
  return (getSlot(slot_id) != nullptr);
}

bool VizDataCollectorImpl::hasChannel(
    const SlotId& slot_id, const std::string& channel_name) const {
  const VizChannelGroup* slot;
  if ((slot = getSlot(slot_id)) == nullptr) {
    return false;
  }
  return slot->hasChannel(channel_name);
}

}  // namespace internal
}  // namespace visualization
