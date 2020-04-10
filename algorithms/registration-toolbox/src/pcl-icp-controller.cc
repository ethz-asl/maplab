#include "registration-toolbox/pcl-icp-controller.h"

namespace regbox {

RegistrationResult PclIcpController::align(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const aslam::Transformation& prior_T_target_source) {
  return controller_.align(target, source, prior_T_target_source);
}

RegistrationResult PclIcpController::align(
    const resources::PointCloud& target, const resources::PointCloud& source,
    const aslam::Transformation& prior_T_target_source) {
  return controller_.align(target, source, prior_T_target_source);
}

}  // namespace regbox
