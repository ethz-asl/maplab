#ifndef VI_MAP_TRANSFORMATION_EDGE_H_
#define VI_MAP_TRANSFORMATION_EDGE_H_

#include <string>

#include <maplab-common/pose_types.h>
#include <maplab-common/traits.h>
#include <sensors/sensor.h>

#include "vi-map/edge.h"
#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"
#include "vi-map/vi_map_deprecated.pb.h"

namespace vi_map {

class TransformationEdge : public vi_map::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(TransformationEdge);
  TransformationEdge() = delete;
  explicit TransformationEdge(vi_map::Edge::EdgeType edge_type);
  TransformationEdge(const TransformationEdge&) = default;

  TransformationEdge(
      vi_map::Edge::EdgeType edge_type, const pose_graph::EdgeId& id,
      const pose_graph::VertexId& from, const pose_graph::VertexId& to,
      const pose::Transformation& T_A_B,
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance_p_q);

  TransformationEdge(
      vi_map::Edge::EdgeType edge_type, const pose_graph::EdgeId& id,
      const pose_graph::VertexId& from, const pose_graph::VertexId& to,
      const pose::Transformation& T_A_B,
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance_p_q,
      const SensorId& sensor_id);
  virtual ~TransformationEdge() {}

  virtual bool operator==(const TransformationEdge& other) const {
    bool is_same = true;
    is_same &= static_cast<const vi_map::Edge&>(*this) == other;
    is_same &= id_ == other.id_;
    is_same &= from_ == other.from_;
    is_same &= to_ == other.to_;
    is_same &= T_A_B_ == other.T_A_B_;
    is_same &= T_A_B_covariance_p_q_ == other.T_A_B_covariance_p_q_;
    is_same &= sensor_id_ == other.sensor_id_;
    return is_same;
  }

  void serialize(vi_map::proto::TransformationEdge* proto) const;
  void deserialize(
      const pose_graph::EdgeId& id,
      const vi_map::proto::TransformationEdge& proto);
  void deserialize(
      const pose_graph::EdgeId& id,
      const vi_map_deprecated::proto::TransformationEdge& proto);

  void set_T_A_B(const pose::Transformation& T_A_B);
  const pose::Transformation& getT_A_B() const;

  void set_T_A_B_Covariance_p_q(
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance);
  const Eigen::Matrix<double, 6, 6>& get_T_A_B_Covariance_p_q() const;

  inline const SensorId& getSensorId() const {
    return sensor_id_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  TransformationEdge& operator=(const TransformationEdge&) = delete;

  pose::Transformation T_A_B_;
  Eigen::Matrix<double, 6, 6> T_A_B_covariance_p_q_;
  SensorId sensor_id_;
};

}  // namespace vi_map

#endif  // VI_MAP_TRANSFORMATION_EDGE_H_
