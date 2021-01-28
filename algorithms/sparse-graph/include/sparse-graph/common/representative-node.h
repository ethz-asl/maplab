#ifndef SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_
#define SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_

#include <vi-map/vi-map.h>

namespace spg {

class RepresentativeNode {
 public:
  explicit RepresentativeNode(
      const aslam::Transformation& pose,
      const pose_graph::VertexIdList& vertices);

  const aslam::Transformation& getPose() const noexcept;
  const pose_graph::VertexIdList& getVertices() const noexcept;

 private:
  const aslam::Transformation& pose_;
  const pose_graph::VertexIdList& vertices_;
};

}  // namespace spg

#endif  // SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_
