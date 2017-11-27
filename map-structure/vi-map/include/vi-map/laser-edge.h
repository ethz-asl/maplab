#ifndef VI_MAP_LASER_EDGE_H_
#define VI_MAP_LASER_EDGE_H_
#include <string>

#include <maplab-common/pose_types.h>
#include <maplab-common/traits.h>

#include "vi-map/edge.h"
#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class LaserEdge : public vi_map::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(LaserEdge);
  LaserEdge();
  LaserEdge(const LaserEdge&) = default;

  LaserEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& laser_timestamps_ns,
      const Eigen::Matrix<double, 4, Eigen::Dynamic>& laser_data_xyzi);

  // Constructors used for testing.
  LaserEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to);

  bool operator==(const LaserEdge& other) const;

  virtual ~LaserEdge() {}

  void serialize(vi_map::proto::LaserEdge* proto) const;
  void deserialize(
      const pose_graph::EdgeId& id, const vi_map::proto::LaserEdge& proto);

  const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& getLaserTimestamps() const;
  const Eigen::Matrix<double, 4, Eigen::Dynamic>& getLaserData() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  LaserEdge& operator=(const LaserEdge&) = delete;

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> laser_timestamps_ns_;
  // For now, X,Y,Z and Intensity will be stored here.
  Eigen::Matrix<double, 4, Eigen::Dynamic> laser_data_xyzi_;
};

}  // namespace vi_map

#endif  // VI_MAP_LASER_EDGE_H_
