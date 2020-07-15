#include "registration-toolbox/alignment/lpm-alignment.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointmatcher_ros/point_cloud.h>

namespace regbox {

RegistrationResult LpmAlignment::registerCloudImpl(
    const PclPointCloudPtr<pcl::PointXYZI>& target,
    const PclPointCloudPtr<pcl::PointXYZI>& source,
    const aslam::Transformation& prior_T_target_source) {

  // TODO(lbern): Implement settings.
  icp_.setDefault();

  sensor_msgs::PointCloud2 target_msg;
  pcl::toROSMsg(*target, target_msg);
  sensor_msgs::PointCloud2 source_msg;
  pcl::toROSMsg(*source, source_msg);

  PointMatcher<double>::DataPoints target_points =
      PointMatcher_ros::rosMsgToPointMatcherCloud<double>(target_msg);
  PointMatcher<double>::DataPoints source_points =
      PointMatcher_ros::rosMsgToPointMatcherCloud<double>(source_msg);

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

  return result;
}

}  // namespace regbox
