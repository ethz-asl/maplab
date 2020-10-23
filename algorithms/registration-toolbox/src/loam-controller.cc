#include "registration-toolbox/loam-controller.h"

namespace regbox {

RegistrationResult LoamController::align(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const aslam::Transformation& prior_T_target_source) {
  return controller_.align(target, source, prior_T_target_source);
}

RegistrationResult LoamController::align(
    const resources::PointCloud& target, const resources::PointCloud& source,
    const aslam::Transformation& prior_T_target_source) {
  return controller_.align(target, source, prior_T_target_source);
}

}  // namespace regbox
