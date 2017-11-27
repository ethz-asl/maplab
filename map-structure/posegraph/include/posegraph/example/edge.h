#ifndef POSEGRAPH_EXAMPLE_EDGE_H_
#define POSEGRAPH_EXAMPLE_EDGE_H_

#include <posegraph/edge.h>

namespace pose_graph {
namespace example {

class Edge : public pose_graph::Edge {
 public:
  explicit Edge(const EdgeId& id);
  Edge(const VertexId& from, const VertexId& to, const EdgeId& id);

  virtual ~Edge();

  EdgeType type() const;

  virtual const EdgeId& id() const;

  virtual const VertexId& from() const;
  virtual const VertexId& to() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  EdgeId id_;
  VertexId from_, to_;
};

}  // namespace example
}  // namespace pose_graph

#endif  // POSEGRAPH_EXAMPLE_EDGE_H_
