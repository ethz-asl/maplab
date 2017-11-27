#ifndef VI_MAP_CKLAM_EDGE_H_
#define VI_MAP_CKLAM_EDGE_H_

#include <string>

#include <maplab-common/pose_types.h>
#include <maplab-common/traits.h>

#include "vi-map/edge.h"
#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class CklamEdge : public vi_map::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(CklamEdge);
  CklamEdge() = delete;
  CklamEdge(const CklamEdge&) = default;

  CklamEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to, const Eigen::Matrix<double, 12, 1>& b,
      const Eigen::Matrix<double, 12, 12>& A,
      const Eigen::Matrix<double, 7, 1>& keyframe_T_G_B_from,
      const Eigen::Matrix<double, 7, 1>& keyframe_T_G_B_to);
  virtual ~CklamEdge() {}

  virtual bool operator==(const CklamEdge& other) const {
    bool is_same = true;
    is_same &= static_cast<const vi_map::Edge&>(*this) == other;
    is_same &= id_ == other.id_;
    is_same &= from_ == other.from_;
    is_same &= to_ == other.to_;
    is_same &= b_ == other.b_;
    is_same &= A_ == other.A_;
    is_same &= keyframe_T_G_B_from_ == other.keyframe_T_G_B_from_;
    is_same &= keyframe_T_G_B_to_ == other.keyframe_T_G_B_to_;
    return is_same;
  }

  void serialize(vi_map::proto::CklamEdge* proto) const;
  void deserialize(
      const pose_graph::EdgeId& id, const vi_map::proto::CklamEdge& proto);

  void setb(const Eigen::Matrix<double, 12, 1>& b);
  const Eigen::Matrix<double, 12, 1>& getb() const;

  void setA(const Eigen::Matrix<double, 12, 12>& A);
  const Eigen::Matrix<double, 12, 12>& getA() const;

  void setKeyframePoseFrom(
      const Eigen::Matrix<double, 7, 1>& keyframe_pose_from);
  const Eigen::Matrix<double, 7, 1>& getKeyframePoseFrom() const;

  void setKeyframePoseTo(const Eigen::Matrix<double, 7, 1>& keyframe_pose_to);
  const Eigen::Matrix<double, 7, 1>& getKeyframePoseTo() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  CklamEdge& operator=(const CklamEdge&) = delete;

  // The cost associated with a CKLAM edge will be calculated as
  // (A*e+b)^2 where e will be a 12x1 vector with first part being
  // (keyframe_T_G_B_from_ - keyframe_T_G_B_from_hat) and second half
  // is (keyframe_T_G_B_to_ - keyframe_T_G_B_to_hat). Since
  // keyframe_T_G_B_'s will have quaternions, the (-) operator
  // for first 4 dimension of each T_G_B component will have to
  // follow JPL multiplication rule.
  Eigen::Matrix<double, 12, 1> b_;
  Eigen::Matrix<double, 12, 12> A_;
  // In the following poses we use first 4 entries as JPL quaternion
  // for frame orientation and last three as the position of global
  // frame in body frame.
  Eigen::Matrix<double, 7, 1> keyframe_T_G_B_from_;
  Eigen::Matrix<double, 7, 1> keyframe_T_G_B_to_;
};

}  // namespace vi_map

#endif  // VI_MAP_CKLAM_EDGE_H_
