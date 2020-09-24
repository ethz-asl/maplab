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
  result.set_T_target_source(convertEigenToKindr(T));
  result.set_T_target_source_covariance(icp_.errorMinimizer->getCovariance());
  result.hasConverged(true);

  return result;
}

}  // namespace regbox
