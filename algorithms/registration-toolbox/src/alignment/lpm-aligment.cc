#include "registration-toolbox/alignment/lpm-alignment.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointmatcher_ros/point_cloud.h>

#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

LpmAlignment::LpmAlignment() : input_filters_(nullptr) {
  if (FLAGS_regbox_lpm_config_path.empty()) {
    icp_.setDefault();
  } else {
    std::ifstream input_config(FLAGS_regbox_lpm_config_path);
    icp_.loadFromYaml(input_config);
  }
  if (!FLAGS_regbox_lpm_input_filter_config_path.empty()) {
    std::ifstream input_filter_config(
        FLAGS_regbox_lpm_input_filter_config_path);
    input_filters_.reset(
        new PointMatcher<double>::DataPointsFilters(input_filter_config));
  }
}

RegistrationResult LpmAlignment::registerCloudImpl(
    const PclPointCloudPtr<pcl::PointXYZI>& target,
    const PclPointCloudPtr<pcl::PointXYZI>& source,
    const aslam::Transformation& prior_T_target_source) {
  CHECK_NOTNULL(target);
  CHECK_NOTNULL(source);

  sensor_msgs::PointCloud2 target_msg;
  pcl::toROSMsg(*target, target_msg);
  sensor_msgs::PointCloud2 source_msg;
  pcl::toROSMsg(*source, source_msg);

  PointMatcher<double>::DataPoints target_points =
      PointMatcher_ros::rosMsgToPointMatcherCloud<double>(target_msg);
  PointMatcher<double>::DataPoints source_points =
      PointMatcher_ros::rosMsgToPointMatcherCloud<double>(source_msg);

  VLOG(1) << target_points.features(0, 0);
  VLOG(1) << "Number of points in target: " << target_points.getNbPoints();
  VLOG(1) << "Number of points in source: " << source_points.getNbPoints();

  // If we configured input filters, we need to apply them before the
  // registration.
  if (input_filters_ != nullptr) {
    VLOG(1) << "APPLYING FILTERS!";
    input_filters_->apply(target_points);
    input_filters_->apply(source_points);
  }

  VLOG(1) << "Number of points in target after filtering: "
          << target_points.getNbPoints();
  VLOG(1) << "Number of points in source after filtering: "
          << source_points.getNbPoints();

  aslam::Transformation test;
  test.setIdentity();
  PointMatcher<double>::TransformationParameters T = icp_.compute(
      source_points, target_points,
      prior_T_target_source.getTransformationMatrix());

  return createResultFromTransformation(source, std::move(T));
}

RegistrationResult LpmAlignment::createResultFromTransformation(
    const PclPointCloudPtr<pcl::PointXYZI>& source,
    PointMatcher<double>::TransformationParameters&& T) const noexcept {
  RegistrationResult result;

  // TODO(lbern): fleg for applying transform.
  PclPointCloudPtr<pcl::PointXYZI> reg(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*source, *reg, T.cast<float>());
  result.setRegisteredCloud(reg);
  // aslam::Transformation T_target_source = convertEigenToKindr(T);
  result.set_T_target_source(T);
  // result.set_T_target_source_covariance(icp_.errorMinimizer->getCovariance());
  const Eigen::MatrixXd lpm_cov = icp_.errorMinimizer->getCovariance();
  if (isValidCovariance(lpm_cov)) {
    VLOG(1) << "cov is valid";
    result.set_T_target_source_covariance(lpm_cov);
  } else {
    VLOG(1) << "cov is invalid";
    const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 1e-4;
    result.set_T_target_source_covariance(cov);
  }
  result.hasConverged(!T.hasNaN());

  if (result.hasConverged()) {
    VLOG(1) << "transform: \n" << result.get_T_target_source();
    // VLOG(1) << "transform: \n" << ;
  }

  return result;
}

bool LpmAlignment::isValidCovariance(const Eigen::MatrixXd& cov) const {
  // const bool is_nan = ((cov.array() == cov.array())).all();
  // const bool is_nan = cov.hasNaN();
  // const bool is_finite = ((cov - cov).array() == (cov - cov).array()).all();
  return !cov.hasNaN() && !cov.isZero();
}

}  // namespace regbox
