#include "sparse-graph/common/representative-node.h"

namespace spg {

RepresentativeNode::RepresentativeNode(
    const aslam::Transformation& pose, const int64_t timestamp_ns,
    const uint64_t submap_id)
    : pose_(pose), timestamp_ns_(timestamp_ns), submap_id_(submap_id) {}

const aslam::Transformation& RepresentativeNode::getPose() const noexcept {
  return pose_;
}

uint64_t RepresentativeNode::getAssociatedSubmapId() const noexcept {
  return submap_id_;
}

int64_t RepresentativeNode::getTimestampNanoseconds() const noexcept {
  return timestamp_ns_;
}

}  // namespace spg
