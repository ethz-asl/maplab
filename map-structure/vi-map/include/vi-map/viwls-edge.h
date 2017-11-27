#ifndef VI_MAP_VIWLS_EDGE_H_
#define VI_MAP_VIWLS_EDGE_H_
#include <string>

#include <maplab-common/pose_types.h>
#include <maplab-common/traits.h>

#include "vi-map/edge.h"
#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class ViwlsEdge : public vi_map::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(ViwlsEdge);
  ViwlsEdge();
  ViwlsEdge(const ViwlsEdge&) = default;

  ViwlsEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data);

  // Constructors used for testing.
  ViwlsEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to);

  bool operator==(const ViwlsEdge& other) const;

  virtual ~ViwlsEdge() {}

  void serialize(vi_map::proto::ViwlsEdge* proto) const;
  void deserialize(
      const pose_graph::EdgeId& id, const vi_map::proto::ViwlsEdge& proto);

  const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& getImuTimestamps() const;
  const Eigen::Matrix<double, 6, Eigen::Dynamic>& getImuData() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  ViwlsEdge& operator=(const ViwlsEdge&) = delete;

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_;
};

}  // namespace vi_map

#endif  // VI_MAP_VIWLS_EDGE_H_
