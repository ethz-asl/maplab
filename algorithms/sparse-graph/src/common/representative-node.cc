#include "sparse-graph/common/representative-node.h"

namespace spg {

RepresentativeNode::RepresentativeNode(
    const aslam::Transformation& pose, const int64_t timestamp_ns,
    const uint32_t submap_id)
    : pose_(pose), timestamp_ns_(timestamp_ns), submap_id_(submap_id) {}

const aslam::Transformation& RepresentativeNode::getPose() const noexcept {
  return pose_;
}

uint32_t RepresentativeNode::getAssociatedSubmapId() const noexcept {
  return submap_id_;
}

int64_t RepresentativeNode::getTimestampNanoseconds() const noexcept {
  return timestamp_ns_;
}

bool RepresentativeNode::isEqualTo(const RepresentativeNode& rhs) const
    noexcept {
  return submap_id_ == rhs.submap_id_ && timestamp_ns_ == rhs.timestamp_ns_;
}

bool RepresentativeNode::isEarlierThan(const RepresentativeNode& rhs) const
    noexcept {
  return timestamp_ns_ < rhs.timestamp_ns_;
}
bool RepresentativeNode::isLaterThan(const RepresentativeNode& rhs) const
    noexcept {
  return timestamp_ns_ > rhs.timestamp_ns_;
}

bool operator==(const RepresentativeNode& lhs, const RepresentativeNode& rhs) {
  return lhs.isEqualTo(rhs);
}
bool operator!=(const RepresentativeNode& lhs, const RepresentativeNode& rhs) {
  return !lhs.isEqualTo(rhs);
}

bool operator<(const RepresentativeNode& lhs, const RepresentativeNode& rhs) {
  return lhs.isEarlierThan(rhs);
}
bool operator>(const RepresentativeNode& lhs, const RepresentativeNode& rhs) {
  return lhs.isLaterThan(rhs);
}

}  // namespace spg
