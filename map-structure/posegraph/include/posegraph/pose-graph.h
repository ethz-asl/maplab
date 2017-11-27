#ifndef POSEGRAPH_POSE_GRAPH_H_
#define POSEGRAPH_POSE_GRAPH_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "posegraph/edge.h"
#include "posegraph/unique-id.h"
#include "posegraph/vertex.h"

namespace pose_graph {

class PoseGraph {
 protected:
  // Accessible by derived classes for more flexible extension.
  typedef std::unordered_map<VertexId, AlignedUniquePtr<Vertex>> VertexMap;
  VertexMap vertices_;
  typedef std::unordered_map<EdgeId, AlignedUniquePtr<Edge>> EdgeMap;
  EdgeMap edges_;

 public:
  MAPLAB_POINTER_TYPEDEFS(PoseGraph);

  PoseGraph() = default;
  virtual ~PoseGraph() {}

  void swap(PoseGraph* other);  // NOLINT

  /****************************************
   * Basic modification
   ****************************************/
  void addVertex(AlignedUniquePtr<Vertex> vertex);

  void addEdge(AlignedUniquePtr<Edge> edge);

  /****************************************
   * Const ops
   ****************************************/

  // Checks if a vertex with VertexId n exists.
  bool vertexExists(const VertexId& n) const;

  // Checks if an edge with EdgeId e exists.
  bool edgeExists(const EdgeId& e) const;

  // Check if an edge between vertices v1 and v2 exists,
  // note that direction does not matter.
  bool edgeExists(const VertexId& v1, const VertexId& v2) const;

  /// \brief Get set of node ids.
  void getAllVertexIds(std::vector<VertexId>* vertices) const;

  /// \brief Get set of edge ids.
  void getAllEdgeIds(std::vector<EdgeId>* edges) const;

  // returns a Vertex with the given ID.
  const Vertex& getVertex(const VertexId& id) const;

  // returns an Edge with the given ID.
  const Edge& getEdge(const EdgeId& id) const;

  // returns a Vertex with the given ID.
  Vertex& getVertexMutable(const VertexId& id);

  // returns an Edge with the given ID.
  Edge& getEdgeMutable(const EdgeId& id);

  Vertex* getVertexPtrMutable(const VertexId& id);

  Edge* getEdgePtrMutable(const EdgeId& id);

  const Vertex* getVertexPtr(const VertexId& id) const;

  const Edge* getEdgePtr(const EdgeId& id) const;

  typename EdgeMap::iterator removeEdge(const pose_graph::EdgeId& id);

  void removeVertex(const pose_graph::VertexId& id);

  template <pose_graph::Edge::EdgeType edge_type>
  size_t removeEdgesOfType();

  size_t numVertices() const {
    return vertices_.size();
  }
  size_t numEdges() const {
    return edges_.size();
  }

  inline void clear();
};

}  // namespace pose_graph

#include "posegraph/pose-graph-inl.h"

#endif  // POSEGRAPH_POSE_GRAPH_H_
