#include "registration-toolbox/alignment/lpm-alignment.h"

#include <memory>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointmatcher_ros/point_cloud.h>

#include "registration-toolbox/common/registration-gflags.h"

#include <glog/logging.h>
#include <ros/package.h>

namespace regbox {

LpmAlignment::LpmAlignment() : input_filters_(nullptr) {
  // Load config for ICP.
  std::string icp_config = FLAGS_regbox_lpm_config_path;
  if (icp_config.empty()) {
    icp_config =
        ros::package::getPath("registration_toolbox") + "/cfg/lpm-config.yaml";
  }
  std::ifstream input_config(icp_config);
  icp_.loadFromYaml(input_config);

  // Load config for pre-alignment filtering.
  std::string icp_input_filter_config =
      FLAGS_regbox_lpm_input_filter_config_path;
  if (icp_input_filter_config.empty()) {
    icp_input_filter_config = ros::package::getPath("registration_toolbox") +
                              "/cfg/lpm-input-filter-config.yaml";
  }
  std::ifstream input_filter_config(icp_input_filter_config);
  input_filters_.reset(
      new PointMatcher<double>::DataPointsFilters(input_filter_config));
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

  // If we configured input filters, we need to apply them before the
  // registration.
  if (input_filters_ != nullptr) {
    input_filters_->apply(target_points);
    input_filters_->apply(source_points);
  }

  PointMatcher<double>::TransformationParameters T = icp_.compute(
      source_points, target_points,
      prior_T_target_source.getTransformationMatrix());

  PointMatcher<double>::DataPoints source_points_aligned(source_points);
  icp_.transformations.apply(source_points_aligned, T);
  icp_.matcher->init(target_points);
  const PointMatcher<double>::Matches matches =
      icp_.matcher->findClosests(source_points_aligned);

  const PointMatcher<double>::OutlierWeights outlier_weights =
      icp_.outlierFilters.compute(
          source_points_aligned, target_points, matches);
  const int n_matched_points = outlier_weights.sum();
  const float match_residuals_m = icp_.errorMinimizer->getResidualError(
      source_points_aligned, target_points, outlier_weights, matches);
  const float match_residual_error_m = match_residuals_m / n_matched_points;
  const bool accept_match =
      match_residual_error_m <
      FLAGS_regbox_lpm_icp_match_residual_error_threshold_m;

  return createResultFromTransformation(source, accept_match, std::move(T));
}

RegistrationResult LpmAlignment::createResultFromTransformation(
    const PclPointCloudPtr<pcl::PointXYZI>& source, const bool accept_match,
    PointMatcher<double>::TransformationParameters&& T) const noexcept {
  RegistrationResult result;

  // TODO(lbern): fleg for applying transform.
  PclPointCloudPtr<pcl::PointXYZI> reg(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::transformPointCloud(*source, *reg, T.cast<float>());
  result.setRegisteredCloud(reg);
  result.set_T_target_source(T);
  const Eigen::MatrixXd& lpm_cov = icp_.errorMinimizer->getCovariance();
  if (FLAGS_regbox_lpm_use_computed_covariance && accept_match &&
      isValidCovariance(lpm_cov)) {
    result.set_T_target_source_covariance(lpm_cov);
  } else {
    Eigen::MatrixXd fixed_cov = Eigen::MatrixXd::Identity(6, 6);
    fixed_cov.block(0, 0, 3, 3) = fixed_cov.block(0, 0, 3, 3) *
                                  FLAGS_regbox_fixed_covariance_translation_m;
    fixed_cov.block(3, 3, 3, 3) = fixed_cov.block(3, 3, 3, 3) *
                                  FLAGS_regbox_fixed_covariance_rotation_rad;
    result.set_T_target_source_covariance(fixed_cov);
  }
  result.hasConverged(accept_match && !T.hasNaN());

  return result;
}

bool LpmAlignment::isValidCovariance(const Eigen::MatrixXd& cov) const {
  return !cov.hasNaN() && !cov.isZero();
}

}  // namespace regbox
