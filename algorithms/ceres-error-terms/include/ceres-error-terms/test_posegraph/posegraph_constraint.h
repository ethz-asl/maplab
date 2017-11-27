#ifndef TEST_POSEGRAPH_POSEGRAPH_CONSTRAINT_H_
#define TEST_POSEGRAPH_POSEGRAPH_CONSTRAINT_H_

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <maplab-common/pose_types.h>

namespace ceres_error_terms {

typedef class PosegraphConstraint {
 public:
  int from_node_, to_node_;
  pose::Transformation T_A_B_;
  Eigen::Matrix<double, 6, 6> T_A_B_covariance_;

  PosegraphConstraint() {}
  PosegraphConstraint(
      const int& from, const int& to, const pose::Transformation& T_A_B,
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance)
      : from_node_(from),
        to_node_(to),
        T_A_B_(T_A_B),
        T_A_B_covariance_(T_A_B_covariance) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} PosegraphConstraint;

struct PosePriorConstraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int node_;
  pose::Transformation T_G_I_;
  Eigen::Matrix<double, 6, 6> T_G_I_covariance_;
  PosePriorConstraint(
      const int node, const pose::Transformation& T_G_I,
      const Eigen::Matrix<double, 6, 6>& T_G_I_covariance)
      : node_(node), T_G_I_(T_G_I), T_G_I_covariance_(T_G_I_covariance) {}
};

}  // namespace ceres_error_terms

#endif  // TEST_POSEGRAPH_POSEGRAPH_CONSTRAINT_H_
