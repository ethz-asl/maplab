#ifndef POSEGRAPH_EXAMPLE_POSE_GRAPH_H_
#define POSEGRAPH_EXAMPLE_POSE_GRAPH_H_

#include <posegraph/pose-graph.h>

#include <memory>

#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>

#include <posegraph/example/edge.h>
#include <posegraph/example/vertex.h>

namespace pose_graph {
namespace example {

class PoseGraph : public pose_graph::PoseGraph {
 public:
  void addVertex(Vertex::UniquePtr vertex);
  void addVertex(const VertexId& id);
  void addEdge(const VertexId& from, const VertexId& to, const EdgeId& id);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace example */
} /* namespace pose_graph */

#endif  // POSEGRAPH_EXAMPLE_POSE_GRAPH_H_
