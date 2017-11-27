#ifndef CERES_ERROR_TERMS_PARAMETERIZATION_POSE_PARAM_JPL_H_
#define CERES_ERROR_TERMS_PARAMETERIZATION_POSE_PARAM_JPL_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "ceres-error-terms/parameterization/quaternion-param-jpl.h"

namespace ceres_error_terms {

class JplPoseParameterization : public ceres::ProductParameterization {
 public:
  JplPoseParameterization()
      : ceres::ProductParameterization(
            new JplQuaternionParameterization,
            new ceres::IdentityParameterization(3)) {}
  virtual ~JplPoseParameterization() {}
};

class JplYawOnlyPoseParameterization : public ceres::ProductParameterization {
 public:
  JplYawOnlyPoseParameterization()
      : ceres::ProductParameterization(
            new JplYawQuaternionParameterization,
            new ceres::IdentityParameterization(3)) {}
  virtual ~JplYawOnlyPoseParameterization() {}
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_PARAMETERIZATION_POSE_PARAM_JPL_H_
