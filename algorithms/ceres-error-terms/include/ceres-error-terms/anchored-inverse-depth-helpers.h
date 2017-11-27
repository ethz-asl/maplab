#ifndef CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_HELPERS_H_
#define CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_HELPERS_H_

#include <Eigen/Core>
#include <memory>

#include <ceres-error-terms/parameterization/unit3-param.h>

namespace ceres_error_terms {

// Anchored Inverse Depth helpers
namespace aid_helpers {

// Returns the unit vector and inverse depth given a Landmark and Anchor in Map
// coordinates.
inline void Euclidean2AID(
    const Eigen::Vector3d& M_p_ML, const Eigen::Vector3d& M_p_MA,
    Eigen::Quaterniond* A_u_AL, double* A_inverse_depth) {
  CHECK_NOTNULL(A_u_AL);
  CHECK_NOTNULL(A_inverse_depth);

  const Eigen::Vector3d M_p_AL = M_p_ML - M_p_MA;
  double A_depth = M_p_AL.norm();
  // TODO(burrimi): What to do in this case?
  const double kEpsilon = 1e-6;
  if (A_depth < kEpsilon) {
    A_depth = kEpsilon;
  }
  *A_inverse_depth = 1 / A_depth;
  Unit3::GetFromVector(*A_inverse_depth * M_p_AL, A_u_AL);
}

// Returns the unit vector and inverse depth given a Landmark and Anchor in Map
// coordinates.
inline void AID2Euclidean(
    const Eigen::Vector3d& M_p_MA, const Eigen::Quaterniond& A_u_AL,
    const double A_inverse_depth, Eigen::Vector3d* M_p_ML) {
  CHECK_NOTNULL(M_p_ML);

  double A_inverse_depth_temp = A_inverse_depth;
  // TODO(burrimi): What to do in this case?
  const double kEpsilon = 1e-6;
  if (A_inverse_depth_temp < kEpsilon) {
    A_inverse_depth_temp = kEpsilon;
  }
  const Eigen::Vector3d A_p_AL =
      Unit3::GetNormalVector(A_u_AL) / A_inverse_depth_temp;

  *M_p_ML = M_p_MA + A_p_AL;
}

}  // namespace aid_helpers
}  // namespace ceres_error_terms
#endif  // CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_HELPERS_H_
