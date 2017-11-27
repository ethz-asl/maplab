#ifndef POSEGRAPH_EDGE_H_
#define POSEGRAPH_EDGE_H_

#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>

#include <aslam/common/hash-id.h>
#include <maplab-common/macros.h>
#include <maplab-common/pose_types.h>
#include <posegraph/unique-id.h>

namespace pose_graph {

class Edge {
 public:
  MAPLAB_GET_AS_CASTER
  MAPLAB_POINTER_TYPEDEFS(Edge);

  enum class EdgeType {
    kUndefined = -1,
    kOdometry = 0,
    kLoopClosure = 1,
    kViwls = 2,
    kStructureLoopClosure = 3,
    k6DoFGps = 4,
    kLaser = 5,
    kTrajectory = 6,
    kCklamImuLandmark = 7
  };

  static std::string edgeTypeToString(const EdgeType edge_type);
  static EdgeType stringToEdgeType(const std::string& edge_type);

  Edge();
  explicit Edge(Edge::EdgeType edge_type);

  virtual ~Edge() {}

  virtual bool operator==(const Edge& other) const;

  virtual const EdgeId& id() const = 0;
  virtual const VertexId& from() const = 0;
  virtual const VertexId& to() const = 0;

  inline EdgeType getType() const {
    return edge_type_;
  }

  void incidentVertices(std::pair<VertexId, VertexId>* vertices) const;

 protected:
  const EdgeType edge_type_;
};

}  // namespace pose_graph

#endif  // POSEGRAPH_EDGE_H_
