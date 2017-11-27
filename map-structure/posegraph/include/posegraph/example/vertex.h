#ifndef POSEGRAPH_EXAMPLE_VERTEX_H_
#define POSEGRAPH_EXAMPLE_VERTEX_H_

#include <unordered_set>

#include <maplab-common/pose_types.h>
#include <posegraph/vertex.h>

namespace pose_graph {
namespace example {

class Vertex : public pose_graph::Vertex {
 public:
  explicit Vertex(const VertexId& id);
  virtual ~Vertex();

  virtual const VertexId& id() const;

  virtual bool addIncomingEdge(const EdgeId& edge);
  virtual bool addOutgoingEdge(const EdgeId& edge);

  virtual void getOutgoingEdges(std::unordered_set<EdgeId>* edges) const;
  virtual void getIncomingEdges(std::unordered_set<EdgeId>* edges) const;

  virtual bool hasIncomingEdges() const;
  virtual bool hasOutgoingEdges() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  virtual void removeIncomingEdge(const pose_graph::EdgeId& edge_id);
  virtual void removeOutgoingEdge(const pose_graph::EdgeId& edge_id);

  VertexId id_;
  std::unordered_set<EdgeId> incoming_;
  std::unordered_set<EdgeId> outgoing_;
};

}  // namespace example
}  // namespace pose_graph

#endif  // POSEGRAPH_EXAMPLE_VERTEX_H_
