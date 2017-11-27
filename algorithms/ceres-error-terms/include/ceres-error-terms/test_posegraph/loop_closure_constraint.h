#ifndef TEST_POSEGRAPH_LOOP_CLOSURE_CONSTRAINT_H_
#define TEST_POSEGRAPH_LOOP_CLOSURE_CONSTRAINT_H_

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <maplab-common/pose_types.h>

namespace ceres_error_terms {

typedef class LoopClosureConstraint {
 public:
  int from_node_, to_node_;
  double switch_variable_;
  double switch_variable_covariance_;
  pose::Transformation T_A_B_;
  Eigen::Matrix<double, 6, 6> T_A_B_covariance_;

  LoopClosureConstraint() {}

  LoopClosureConstraint(
      const int& from, const int& to, const pose::Transformation& T_A_B,
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance,
      const double& switch_variable_covariance,
      const double& initial_switch_variable)
      : from_node_(from),
        to_node_(to),
        switch_variable_(initial_switch_variable),
        switch_variable_covariance_(switch_variable_covariance),
        T_A_B_(T_A_B),
        T_A_B_covariance_(T_A_B_covariance) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} LoopClosureConstraint;

}  // namespace ceres_error_terms

#endif  // TEST_POSEGRAPH_LOOP_CLOSURE_CONSTRAINT_H_
