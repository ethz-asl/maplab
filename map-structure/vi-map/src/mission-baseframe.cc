#include "vi-map/mission-baseframe.h"

#include <maplab-common/eigen-proto.h>

namespace vi_map {

MissionBaseFrame::MissionBaseFrame(
    const MissionBaseFrameId& mission_baseframe_id,
    const pose::Transformation& T_G_M,
    const Eigen::Matrix<double, 6, 6>& T_G_M_covariance)
    : id_(mission_baseframe_id),
      is_T_G_M_known_(false),
      T_G_M_(T_G_M),
      T_G_M_covariance_(T_G_M_covariance) {
  CHECK(mission_baseframe_id.isValid());
}

Eigen::Vector3d MissionBaseFrame::transformPointInMissionFrameToGlobalFrame(
    const Eigen::Vector3d& M_p_fi) const {
  return T_G_M_ * M_p_fi;
}

Eigen::Quaterniond
MissionBaseFrame::transformRotationInMissionFrameToGlobalFrame(
    const Eigen::Quaterniond& M_q_fi) const {
  return T_G_M_.getRotation().toImplementation() * M_q_fi;
}

Eigen::Vector3d MissionBaseFrame::transformPointInGlobalFrameToMissionFrame(
    const Eigen::Vector3d& G_p_fi) const {
  return T_G_M_.inverse() * G_p_fi;
}

void MissionBaseFrame::serialize(vi_map::proto::MissionBaseframe* proto) const {
  CHECK_NOTNULL(proto);
  common::eigen_proto::serialize(T_G_M_, proto->mutable_t_g_m());
  common::eigen_proto::serialize(
      T_G_M_covariance_, proto->mutable_t_g_m_covariance());
  proto->set_is_t_g_m_known(is_T_G_M_known_);
}

void MissionBaseFrame::deserialize(
    const vi_map::MissionBaseFrameId& baseframe_id,
    const vi_map::proto::MissionBaseframe& proto) {
  CHECK(baseframe_id.isValid());
  id_ = baseframe_id;
  common::eigen_proto::deserialize(proto.t_g_m(), &T_G_M_);
  common::eigen_proto::deserialize(
      proto.t_g_m_covariance(), &T_G_M_covariance_);

  if (proto.has_is_t_g_m_known()) {
    is_T_G_M_known_ = proto.is_t_g_m_known();
  }
}

}  // namespace vi_map
