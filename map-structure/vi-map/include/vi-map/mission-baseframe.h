#ifndef VI_MAP_MISSION_BASEFRAME_H_
#define VI_MAP_MISSION_BASEFRAME_H_

#include <unordered_map>

#include <Eigen/Dense>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <glog/logging.h>
#include <maplab-common/macros.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/unique-id.h>

#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {
class MissionBaseFrame {
 public:
  MissionBaseFrame() : is_T_G_M_known_(false) {
    T_G_M_.setIdentity();
    T_G_M_covariance_.setIdentity();
  }

  MissionBaseFrame(
      const MissionBaseFrameId& mission_baseframe_id,
      const pose::Transformation& T_G_M,
      const Eigen::Matrix<double, 6, 6>& T_G_M_covariance);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(MissionBaseFrame);
  MAPLAB_GET_AS_CASTER

  const Eigen::Quaterniond& get_q_G_M() const {
    return T_G_M_.getRotation().toImplementation();
  }
  const Eigen::Matrix<double, 3, 1>& get_p_G_M() const {
    return T_G_M_.getPosition();
  }
  double* get_q_G_M_Mutable() {
    return T_G_M_.getRotation().toImplementation().coeffs().data();
  }
  double* get_p_G_M_Mutable() {
    return T_G_M_.getPosition().data();
  }

  void set_q_G_M(const Eigen::Quaterniond& q_G_M) {
    T_G_M_.getRotation().toImplementation() = q_G_M;
  }
  void set_p_G_M(const Eigen::Matrix<double, 3, 1>& p_G_M) {
    T_G_M_.getPosition() = p_G_M;
  }

  inline const pose::Transformation& get_T_G_M() const {
    return T_G_M_;
  }

  inline void set_T_G_M(const pose::Transformation& T_G_M) {
    T_G_M_ = T_G_M;
  }

  const MissionBaseFrameId& id() const {
    return id_;
  }

  void setId(const MissionBaseFrameId& id) {
    id_ = id;
  }

  inline const Eigen::Matrix<double, 6, 6>& get_T_G_M_Covariance() const {
    return T_G_M_covariance_;
  }

  void set_T_G_M_Covariance(const Eigen::Matrix<double, 6, 6>& covariance) {
    T_G_M_covariance_ = covariance;
  }

  bool is_T_G_M_known() const {
    return is_T_G_M_known_;
  }

  void set_is_T_G_M_known(bool is_T_G_M_known) {
    is_T_G_M_known_ = is_T_G_M_known;
  }

  inline bool operator==(const MissionBaseFrame& lhs) const {
    bool is_same = true;
    is_same &= T_G_M_ == lhs.T_G_M_;
    is_same &= T_G_M_covariance_ == lhs.T_G_M_covariance_;
    is_same &= is_T_G_M_known_ == lhs.is_T_G_M_known_;
    return is_same;
  }
  inline bool operator!=(const MissionBaseFrame& lhs) const {
    return !operator==(lhs);
  }

  Eigen::Vector3d transformPointInMissionFrameToGlobalFrame(
      const Eigen::Vector3d& M_p_fi) const;
  Eigen::Quaterniond transformRotationInMissionFrameToGlobalFrame(
      const Eigen::Quaterniond& M_q_fi) const;
  Eigen::Vector3d transformPointInGlobalFrameToMissionFrame(
      const Eigen::Vector3d& G_p_fi) const;

  void serialize(vi_map::proto::MissionBaseframe* proto) const;
  void deserialize(
      const vi_map::MissionBaseFrameId& baseframe_id,
      const vi_map::proto::MissionBaseframe& proto);

 private:
  MissionBaseFrameId id_;
  bool is_T_G_M_known_;
  aslam::Transformation T_G_M_;

  // Ordering [orientation, position].
  Eigen::Matrix<double, 6, 6> T_G_M_covariance_;
};

typedef AlignedUnorderedMap<MissionBaseFrameId, MissionBaseFrame>
    MissionBaseFrameMap;
}  // namespace vi_map
#endif  // VI_MAP_MISSION_BASEFRAME_H_
