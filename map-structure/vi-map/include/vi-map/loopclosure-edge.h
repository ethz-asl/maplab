#ifndef VI_MAP_LOOPCLOSURE_EDGE_H_
#define VI_MAP_LOOPCLOSURE_EDGE_H_

#include <string>

#include <maplab-common/pose_types.h>
#include <maplab-common/traits.h>

#include "vi-map/edge.h"
#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class LoopClosureEdge : public vi_map::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(LoopClosureEdge);
  LoopClosureEdge();
  LoopClosureEdge(const LoopClosureEdge&) = default;

  LoopClosureEdge(const pose_graph::EdgeId& id,
                  const pose_graph::VertexId& from,
                  const pose_graph::VertexId& to,
                  const double switch_variable,
                  const double switch_variable_variance,
                  const pose::Transformation& T_A_B,
                  const Eigen::Matrix<double, 6, 6>& T_A_B_covariance);
  virtual ~LoopClosureEdge() {}

  void serialize(vi_map::proto::LoopclosureEdge* proto) const;
  void deserialize(
      const pose_graph::EdgeId& id,
      const vi_map::proto::LoopclosureEdge& proto);

  virtual bool operator==(const LoopClosureEdge& other) const {
    bool is_same = true;
    is_same &= static_cast<const vi_map::Edge&>(*this) == other;
    is_same &= id_ == other.id_;
    is_same &= from_ == other.from_;
    is_same &= to_ == other.to_;
    is_same &= switch_variable_ == other.switch_variable_;
    is_same &= switch_variable_variance_ == other.switch_variable_variance_;
    is_same &= T_A_B_ == other.T_A_B_;
    is_same &= T_A_B_covariance_ == other.T_A_B_covariance_;
    return is_same;
  }

  inline void setSwitchVariable(double switch_variable) {
    switch_variable_ = switch_variable;
  }

  inline double getSwitchVariable() const {
    return switch_variable_;
  }
  inline double getSwitchVariableVariance() const {
    return switch_variable_variance_;
  }
  inline void setSwitchVariableVariance(const double switch_variable_variance) {
    switch_variable_variance_ = switch_variable_variance;
    CHECK_GT(switch_variable_variance_, 0.0);
  }
  inline double* getSwitchVariableMutable() {
    return &switch_variable_;
  }

  void set_T_A_B(const pose::Transformation& T_A_B);
  const pose::Transformation& getT_A_B() const;

  void set_T_A_B_Covariance(
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance);
  const Eigen::Matrix<double, 6, 6>& getT_A_BCovariance() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  LoopClosureEdge& operator=(const LoopClosureEdge&) = delete;

  double switch_variable_;
  double switch_variable_variance_;
  pose::Transformation T_A_B_;
  Eigen::Matrix<double, 6, 6> T_A_B_covariance_;
};

}  // namespace vi_map

#endif  // VI_MAP_LOOPCLOSURE_EDGE_H_
