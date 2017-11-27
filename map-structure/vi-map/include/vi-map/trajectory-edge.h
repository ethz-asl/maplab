#ifndef VI_MAP_TRAJECTORY_EDGE_H_
#define VI_MAP_TRAJECTORY_EDGE_H_
#include <string>

#include <maplab-common/pose_types.h>
#include <maplab-common/traits.h>

#include "vi-map/edge.h"
#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class TrajectoryEdge : public vi_map::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(TrajectoryEdge);
  TrajectoryEdge();
  TrajectoryEdge(const TrajectoryEdge&) = default;

  TrajectoryEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& trajectory_timestamps_ns,
      const Eigen::Matrix<double, 7, Eigen::Dynamic>& trajectory_G_T_I_pq,
      const uint32_t trajectory_identifier);

  // Constructors used for testing.
  TrajectoryEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to);

  bool operator==(const TrajectoryEdge& other) const;

  virtual ~TrajectoryEdge() {}

  void serialize(vi_map::proto::TrajectoryEdge* proto) const;
  void deserialize(
      const pose_graph::EdgeId& id, const vi_map::proto::TrajectoryEdge& proto);

  const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& getTrajectoryTimestamps()
      const;
  const Eigen::Matrix<double, 7, Eigen::Dynamic>& getTrajectoryData() const;
  uint32_t getIdentifier() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  TrajectoryEdge& operator=(const TrajectoryEdge&) = delete;

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> trajectory_timestamps_ns_;

  // For now, x, y, z, q_w, q_x, q_y and q_z will be stored here, where q_w,
  // q_x, q_y and q_z are the coefficients of an active quaternion in Hamilton
  // notation.
  Eigen::Matrix<double, 7, Eigen::Dynamic> trajectory_G_T_I_pq_;

  // For now, this identifier will distinguish between different trajectory
  // edges (eg. trajectories representing different frames transformations).
  uint32_t trajectory_identifier_;
};

}  // namespace vi_map

#endif  // VI_MAP_TRAJECTORY_EDGE_H_
