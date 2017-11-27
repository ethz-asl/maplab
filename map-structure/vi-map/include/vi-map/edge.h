#ifndef VI_MAP_EDGE_H_
#define VI_MAP_EDGE_H_

#include <aslam/common/memory.h>
#include <posegraph/edge.h>

#include "vi-map/mission.h"
#include "vi-map/vi_map_deprecated.pb.h"

namespace vi_map {
namespace proto {
class Edge;
}  // namespace proto
}  // namespace vi_map

namespace vi_map {
class Edge : public pose_graph::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(Edge);
  explicit Edge(pose_graph::Edge::EdgeType edge_type)
      : pose_graph::Edge(edge_type) {}
  Edge(
      pose_graph::Edge::EdgeType edge_type, const pose_graph::EdgeId& id,
      const pose_graph::VertexId& from, const pose_graph::VertexId& to)
      : pose_graph::Edge(edge_type), id_(id), from_(from), to_(to) {}
  virtual ~Edge() {}

  const pose_graph::EdgeId& id() const {
    return id_;
  }
  const pose_graph::VertexId& from() const {
    return from_;
  }
  const pose_graph::VertexId& to() const {
    return to_;
  }

  void setId(const pose_graph::EdgeId& id);
  void setFrom(const pose_graph::VertexId& from);
  void setTo(const pose_graph::VertexId& to);

  virtual bool operator==(const Edge& other) const {
    return static_cast<const pose_graph::Edge&>(*this) == other;
  }

  void serialize(vi_map::proto::Edge* proto) const;
  static Edge::UniquePtr deserialize(
      const pose_graph::EdgeId& edge_id, const vi_map::proto::Edge& proto);
  static Edge::UniquePtr deserialize(
      const pose_graph::EdgeId& edge_id,
      const vi_map_deprecated::proto::Edge& proto);

  // Copies this object into a new edge.
  // Input: pointer to a shared pointer which should store the copied edge.
  void copyEdgeInto(Edge** new_edge) const;

 protected:
  // Helper function to copy edge.
  template <typename EdgeType>
  void copyEdge(Edge** new_edge) const {
    CHECK_NOTNULL(new_edge);
    const EdgeType* original_edge =
        dynamic_cast<const EdgeType*>(this);  // NOLINT
    CHECK_NOTNULL(original_edge);
    *new_edge = new EdgeType(*original_edge);
    CHECK(*new_edge != nullptr);
  }

  pose_graph::EdgeId id_;
  pose_graph::VertexId from_, to_;
};
}  // namespace vi_map
#endif  // VI_MAP_EDGE_H_
