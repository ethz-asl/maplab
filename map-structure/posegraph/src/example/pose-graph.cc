#include "posegraph/example/pose-graph.h"

#include <aslam/common/memory.h>
#include <glog/logging.h>

namespace pose_graph {
namespace example {

void PoseGraph::addEdge(
    const VertexId& from, const VertexId& to, const EdgeId& id) {
  ::pose_graph::PoseGraph::addEdge(Edge::UniquePtr(new Edge(from, to, id)));
}

void PoseGraph::addVertex(const VertexId& id) {
  Vertex* vertex_ptr(new Vertex(id));
  CHECK(vertices_.emplace(id, Vertex::UniquePtr(vertex_ptr)).second)
      << "Vertex with ID " << id.hexString() << " already exists.";
}

void PoseGraph::addVertex(Vertex::UniquePtr vertex) {
  CHECK(vertex != nullptr);
  CHECK(vertices_.emplace(vertex->id(), std::move(vertex)).second)
      << "Vertex with ID " << vertex->id().hexString() << " already exists.";
}

}  // namespace example
}  // namespace pose_graph
