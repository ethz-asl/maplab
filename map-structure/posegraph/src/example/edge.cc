#include <glog/logging.h>
#include <posegraph/example/edge.h>

namespace pose_graph {
namespace example {

Edge::Edge(const EdgeId& id) : id_(id) {}

Edge::Edge(const VertexId& from, const VertexId& to, const EdgeId& id)
    : id_(id), from_(from), to_(to) {}

Edge::~Edge() {}

const EdgeId& Edge::id() const {
  return id_;
}

const VertexId& Edge::from() const {
  return from_;
}

const VertexId& Edge::to() const {
  return to_;
}

}  // namespace example
}  // namespace pose_graph
