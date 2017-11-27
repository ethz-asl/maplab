#include "vi-map/transformation-edge.h"

#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {
TransformationEdge::TransformationEdge(vi_map::Edge::EdgeType edge_type)
    : vi_map::Edge(edge_type) {
  CHECK(
      edge_type == vi_map::Edge::EdgeType::kOdometry ||
      edge_type == vi_map::Edge::EdgeType::k6DoFGps)
      << "Invalid edge"
         "type. Only odometry and GPS edges can be transformation edges.";
}

TransformationEdge::TransformationEdge(
    vi_map::Edge::EdgeType edge_type, const pose_graph::EdgeId& id,
    const pose_graph::VertexId& from, const pose_graph::VertexId& to,
    const pose::Transformation& T_A_B,
    const Eigen::Matrix<double, 6, 6>& T_A_B_covariance_p_q)
    : vi_map::Edge(edge_type, id, from, to),
      T_A_B_(T_A_B),
      T_A_B_covariance_p_q_(T_A_B_covariance_p_q) {
  sensor_id_.setInvalid();
}

TransformationEdge::TransformationEdge(
    vi_map::Edge::EdgeType edge_type, const pose_graph::EdgeId& id,
    const pose_graph::VertexId& from, const pose_graph::VertexId& to,
    const pose::Transformation& T_A_B,
    const Eigen::Matrix<double, 6, 6>& T_A_B_covariance_p_q,
    const SensorId& sensor_id)
    : vi_map::Edge(edge_type, id, from, to),
      T_A_B_(T_A_B),
      T_A_B_covariance_p_q_(T_A_B_covariance_p_q),
      sensor_id_(sensor_id) {
  CHECK(sensor_id_.isValid());
}

void TransformationEdge::serialize(
    vi_map::proto::TransformationEdge* proto) const {
  CHECK_NOTNULL(proto);

  proto->Clear();
  from_.serialize(proto->mutable_from());
  to_.serialize(proto->mutable_to());
  sensor_id_.serialize(proto->mutable_sensor_id());
  common::eigen_proto::serialize(
      T_A_B_covariance_p_q_, proto->mutable_t_a_b_covariance());
  common::eigen_proto::serialize(T_A_B_, proto->mutable_t_a_b());
}

void TransformationEdge::deserialize(
    const pose_graph::EdgeId& id,
    const vi_map::proto::TransformationEdge& proto) {
  id_ = id;
  from_.deserialize(proto.from());
  to_.deserialize(proto.to());
  if (proto.has_sensor_id()) {
    sensor_id_.deserialize(proto.sensor_id());
  } else {
    sensor_id_.setInvalid();
  }
  common::eigen_proto::deserialize(
      proto.t_a_b_covariance(), &T_A_B_covariance_p_q_);
  common::eigen_proto::deserialize(proto.t_a_b(), &T_A_B_);
}

void TransformationEdge::deserialize(
    const pose_graph::EdgeId& id,
    const vi_map_deprecated::proto::TransformationEdge& proto) {
  id_ = id;
  from_.deserialize(proto.from());
  to_.deserialize(proto.to());
  if (proto.has_optional_sensor_extrinsics_id()) {
    sensor_id_.deserialize(proto.optional_sensor_extrinsics_id());
    CHECK(sensor_id_.isValid());
  } else {
    sensor_id_.setInvalid();
  }

  common::eigen_proto::deserialize(
      proto.t_a_b_covariance(), &T_A_B_covariance_p_q_);
  common::eigen_proto::deserialize(proto.t_a_b(), &T_A_B_);
}

void TransformationEdge::set_T_A_B(const pose::Transformation& T_A_B) {
  T_A_B_ = T_A_B;
}

const pose::Transformation& TransformationEdge::getT_A_B() const {
  return T_A_B_;
}

void TransformationEdge::set_T_A_B_Covariance_p_q(
    const Eigen::Matrix<double, 6, 6>& T_A_B_covariance) {
  T_A_B_covariance_p_q_ = T_A_B_covariance;
}

const Eigen::Matrix<double, 6, 6>&
TransformationEdge::get_T_A_B_Covariance_p_q() const {
  return T_A_B_covariance_p_q_;
}
}  // namespace vi_map
