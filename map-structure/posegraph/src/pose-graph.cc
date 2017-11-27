#include "posegraph/pose-graph.h"

#include <unordered_map>

#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>

#include "posegraph/edge.h"
#include "posegraph/vertex.h"

namespace pose_graph {

void PoseGraph::swap(PoseGraph* other) {
  CHECK_NOTNULL(other);
  vertices_.swap(other->vertices_);
  edges_.swap(other->edges_);
}

void PoseGraph::addVertex(Vertex::UniquePtr vertex) {
  CHECK(vertex != nullptr);
  const VertexId& vertex_id = vertex->id();
  CHECK(vertices_.emplace(vertex_id, std::move(vertex)).second)
      << "Vertex already exists.";
}

void PoseGraph::addEdge(Edge::UniquePtr edge) {
  // Insert new edge and do necessary book-keeping in vertices.
  CHECK(edge != nullptr);
  const Edge* const edge_raw = edge.get();
  CHECK(edges_.emplace(edge_raw->id(), std::move(edge)).second)
      << "Edge already exists.";
  Vertex& vertex_from = getVertexMutable(edge_raw->from());
  Vertex& vertex_to = getVertexMutable(edge_raw->to());
  CHECK(
      vertex_from.addOutgoingEdge(edge_raw->id()) &&
      vertex_to.addIncomingEdge(edge_raw->id()));
}

const Vertex& PoseGraph::getVertex(const VertexId& id) const {
  const VertexMap::const_iterator it = vertices_.find(id);
  CHECK(it != vertices_.end()) << "Vertex with ID " << id
                               << " not in posegraph.";
  return *CHECK_NOTNULL(it->second.get());
}

const Edge& PoseGraph::getEdge(const EdgeId& id) const {
  const EdgeMap::const_iterator it = edges_.find(id);
  CHECK(it != edges_.end()) << "Edge with ID " << id << " not in posegraph.";
  return *CHECK_NOTNULL(it->second.get());
}

Vertex& PoseGraph::getVertexMutable(const VertexId& id) {
  return *getVertexPtrMutable(id);
}

Edge& PoseGraph::getEdgeMutable(const EdgeId& id) {
  return *getEdgePtrMutable(id);
}

Vertex* PoseGraph::getVertexPtrMutable(const VertexId& id) {
  return common::getChecked(vertices_, id).get();
}

Edge* PoseGraph::getEdgePtrMutable(const EdgeId& id) {
  return common::getChecked(edges_, id).get();
}

const Vertex* PoseGraph::getVertexPtr(const VertexId& id) const {
  return common::getChecked(vertices_, id).get();
}

const Edge* PoseGraph::getEdgePtr(const EdgeId& id) const {
  return common::getChecked(edges_, id).get();
}

bool PoseGraph::vertexExists(const VertexId& id) const {
  return vertices_.count(id) > 0u;
}

bool PoseGraph::edgeExists(const EdgeId& id) const {
  return edges_.count(id) > 0u;
}

bool PoseGraph::edgeExists(const VertexId& v1, const VertexId& v2) const {
  CHECK(vertexExists(v2)) << "Vertex with ID " << v2.hexString()
                          << " not in posegraph.";
  const Vertex& from = getVertex(v1);
  std::unordered_set<EdgeId> incident_edges;
  from.incidentEdges(&incident_edges);
  for (const EdgeId& element : incident_edges) {
    const Edge& edge = getEdge(element);
    if (edge.to() == v2 || edge.from() == v2) {
      return true;
    }
  }
  return false;
}

void PoseGraph::getAllVertexIds(VertexIdList* vertices) const {
  CHECK_NOTNULL(vertices)->clear();
  vertices->reserve(vertices_.size());
  for (const VertexMap::value_type& vertex_id_pair : vertices_) {
    vertices->emplace_back(vertex_id_pair.first);
  }
}

void PoseGraph::getAllEdgeIds(EdgeIdList* edges) const {
  CHECK_NOTNULL(edges)->clear();
  edges->reserve(edges_.size());
  for (const EdgeMap::value_type& edge_id_pair : edges_) {
    edges->emplace_back(edge_id_pair.first);
  }
}

typename PoseGraph::EdgeMap::iterator PoseGraph::removeEdge(
    const pose_graph::EdgeId& id) {
  const EdgeMap::const_iterator edge_iterator = edges_.find(id);
  CHECK(id.isValid());
  CHECK(edge_iterator != edges_.end()) << "Edge with ID " << id.hexString()
                                       << " does not exist.";
  getVertexPtrMutable(edge_iterator->second->from())->removeOutgoingEdge(id);
  getVertexPtrMutable(edge_iterator->second->to())->removeIncomingEdge(id);
  return edges_.erase(edge_iterator);
}

void PoseGraph::removeVertex(const pose_graph::VertexId& id) {
  const VertexMap::const_iterator it = vertices_.find(id);
  CHECK(it != vertices_.end()) << "Vertex with ID " << id << " does not exist.";
  CHECK(!it->second->hasIncomingEdges())
      << "Vertex can't be linked with edges if you want to remove it.";
  CHECK(!it->second->hasOutgoingEdges())
      << "Vertex can't be linked with edges if you want to remove it.";
  vertices_.erase(it);
}

}  // namespace pose_graph
