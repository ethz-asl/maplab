#include <glog/logging.h>
#include <posegraph/example/vertex.h>

namespace pose_graph {
namespace example {

Vertex::Vertex(const VertexId& id) : id_(id) {}

Vertex::~Vertex() {}

const VertexId& Vertex::id() const {
  return id_;
}

bool Vertex::addIncomingEdge(const EdgeId& edge) {
  return incoming_.insert(edge).second;
}
bool Vertex::addOutgoingEdge(const EdgeId& edge) {
  return outgoing_.insert(edge).second;
}

void Vertex::getOutgoingEdges(std::unordered_set<EdgeId>* edges) const {
  CHECK(edges);
  edges->insert(outgoing_.begin(), outgoing_.end());
}

void Vertex::getIncomingEdges(std::unordered_set<EdgeId>* edges) const {
  CHECK(edges);
  edges->insert(incoming_.begin(), incoming_.end());
}

bool Vertex::hasIncomingEdges() const {
  return !incoming_.empty();
}

bool Vertex::hasOutgoingEdges() const {
  return !outgoing_.empty();
}

void Vertex::removeIncomingEdge(const pose_graph::EdgeId& edge_id) {
  CHECK_GT(incoming_.count(edge_id), 0u);
  incoming_.erase(edge_id);
}

void Vertex::removeOutgoingEdge(const pose_graph::EdgeId& edge_id) {
  CHECK_GT(outgoing_.count(edge_id), 0u);
  outgoing_.erase(edge_id);
}

}  // namespace example
}  // namespace pose_graph
