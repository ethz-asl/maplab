#include "registration-toolbox/mock-controller.h"

namespace regbox {

RegistrationResult MockController::align(
    const resources::PointCloud& target, const resources::PointCloud& source,
    const aslam::Transformation& prior_T_target_source) {
  RegistrationResult result;
  aslam::TransformationCovariance covariance =
      aslam::TransformationCovariance::Identity();
  result.set_T_target_source_covariance(covariance);
  result.set_T_target_source(prior_T_target_source);
  result.hasConverged(true);
  return result;
}

}  // namespace regbox
