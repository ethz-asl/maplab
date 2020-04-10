#include "registration-toolbox/model/registration-result.h"

namespace regbox {

RegistrationResult::RegistrationResult(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& reg,
    const aslam::TransformationCovariance& T_target_source_covariance,
    const aslam::Transformation& T_target_source, const bool has_converged)
    : registered_(reg),
      T_target_source_covariance_(T_target_source_covariance),
      T_target_source_(T_target_source),
      has_converged_(has_converged) {}

void RegistrationResult::setRegisteredCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& reg) {
  registered_ = reg;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RegistrationResult::getRegisteredCloud()
    const noexcept {
  return registered_;
}

void RegistrationResult::set_T_target_source_covariance(
    const aslam::TransformationCovariance& T_target_source_covariance) {
  T_target_source_covariance_ = T_target_source_covariance;
}

aslam::TransformationCovariance
RegistrationResult::get_T_target_source_covariance() const noexcept {
  return T_target_source_covariance_;
}

void RegistrationResult::set_T_target_source(
    const Eigen::Matrix4d& T_target_source) {
  T_target_source_ = aslam::Transformation(T_target_source);
}

void RegistrationResult::set_T_target_source(
    const aslam::Transformation& T_target_source) {
  T_target_source_ = T_target_source;
}

aslam::Transformation RegistrationResult::get_T_target_source() const noexcept {
  return T_target_source_;
}

bool RegistrationResult::hasConverged() const noexcept {
  return has_converged_;
}

void RegistrationResult::hasConverged(const bool converged) {
  has_converged_ = converged;
}

}  // namespace regbox
