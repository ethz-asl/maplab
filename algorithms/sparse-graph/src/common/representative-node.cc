#include "sparse-graph/common/representative-node.h"

namespace spg {

RepresentativeNode::RepresentativeNode(
    const aslam::Transformation& pose, const pose_graph::VertexIdList& vertices)
    : pose_(pose), vertices_(vertices) {}

const aslam::Transformation& RepresentativeNode::getPose() const noexcept {
  return pose_;
}

const pose_graph::VertexIdList& RepresentativeNode::getVertices() const
    noexcept {
  return vertices_;
}

}  // namespace spg
