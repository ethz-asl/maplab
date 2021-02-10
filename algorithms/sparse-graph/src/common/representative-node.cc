#include "sparse-graph/common/representative-node.h"

namespace spg {

RepresentativeNode::RepresentativeNode(
    const aslam::Transformation& pose, const int64_t timestamp_ns,
    const uint32_t submap_id)
    : pose_(pose),
      timestamp_ns_(timestamp_ns),
      submap_id_(submap_id),
      is_active_(true) {}

const aslam::Transformation& RepresentativeNode::getPose() const noexcept {
  return pose_;
}

uint32_t RepresentativeNode::getAssociatedSubmapId() const noexcept {
  return submap_id_;
}

int64_t RepresentativeNode::getTimestampNanoseconds() const noexcept {
  return timestamp_ns_;
}

double& RepresentativeNode::getResidual() {
  return residual_;
}

double RepresentativeNode::getResidual() const noexcept {
  return residual_;
}

void RepresentativeNode::setResidual(const double res) {
  residual_ = res;
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

bool RepresentativeNode::isActive() const noexcept {
  return is_active_;
}

std::vector<uint32_t> RepresentativeNode::getLocalIndex() const noexcept {
  return local_index_;
}

void RepresentativeNode::setLocalIndex(
    const std::vector<uint32_t>& local_index) {
  local_index_ = local_index;
}

void RepresentativeNode::setLocalIndex(std::vector<uint32_t>&& local_index) {
  local_index_ = local_index;
}

bool RepresentativeNode::containsLocalIndex(const uint32_t local_index) const
    noexcept {
  auto it = std::find(local_index_.cbegin(), local_index_.cend(), local_index);
  return it != local_index_.end();
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
