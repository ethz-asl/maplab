#ifndef TEST_POSEGRAPH_POSEGRAPH_H_
#define TEST_POSEGRAPH_POSEGRAPH_H_

#include <vector>

#include <Eigen/Dense>

#include <aslam/common/memory.h>
#include <ceres-error-terms/test_posegraph/loop_closure_constraint.h>
#include <ceres-error-terms/test_posegraph/posegraph_constraint.h>
#include <maplab-common/pose_types.h>

namespace ceres_error_terms {

typedef struct {
  Aligned<std::vector, pose::Transformation> poses;
  Aligned<std::vector, PosegraphConstraint> constraints;
  Aligned<std::vector, LoopClosureConstraint> loop_closure_constraints;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} Posegraph;

typedef struct {
  Eigen::MatrixXd vertex_poses;
  Eigen::MatrixXd baseframe_poses;
  std::unordered_map<int, int> vertex_pose_idx_to_baseframe_idx;
  Aligned<std::vector, PosegraphConstraint> constraints;
  Aligned<std::vector, PosePriorConstraint> pose_prior_constriants;
  Aligned<std::vector, LoopClosureConstraint> loop_closure_constraints;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} BlockPosegraph;

}  // namespace ceres_error_terms

#endif  // TEST_POSEGRAPH_POSEGRAPH_H_
