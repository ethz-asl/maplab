#ifndef POSEGRAPH_VERTEX_H_
#define POSEGRAPH_VERTEX_H_

#include <memory>
#include <unordered_set>

#include <maplab-common/macros.h>
#include <posegraph/unique-id.h>

namespace pose_graph {

class Vertex {
  friend class PoseGraph;

 public:
  MAPLAB_GET_AS_CASTER
  MAPLAB_POINTER_TYPEDEFS(Vertex);

  virtual ~Vertex() {}

  virtual const VertexId& id() const = 0;

  virtual bool addIncomingEdge(const EdgeId& edge) = 0;
  virtual bool addOutgoingEdge(const EdgeId& edge) = 0;

  void incidentEdges(EdgeIdSet* edges) const;
  virtual void getOutgoingEdges(EdgeIdSet* edges) const = 0;
  virtual void getIncomingEdges(EdgeIdSet* edges) const = 0;

  virtual bool hasIncomingEdges() const = 0;
  virtual bool hasOutgoingEdges() const = 0;

 private:
  virtual void removeIncomingEdge(const pose_graph::EdgeId& edge_id) = 0;
  virtual void removeOutgoingEdge(const pose_graph::EdgeId& edge_id) = 0;
};

}  // namespace pose_graph

#endif  // POSEGRAPH_VERTEX_H_
