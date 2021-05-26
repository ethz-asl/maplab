#ifndef REGISTRATION_TOOLBOX_ALIGNMENT_LPM_ALIGNMENT_H_
#define REGISTRATION_TOOLBOX_ALIGNMENT_LPM_ALIGNMENT_H_

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pointmatcher/PointMatcher.h>

#include "registration-toolbox/alignment/base-alignment.h"
#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;

class LpmAlignment : public BaseAlignment<PclPointCloudPtr<pcl::PointXYZI>> {
 public:
  LpmAlignment();
  virtual ~LpmAlignment() = default;

 protected:
  RegistrationResult registerCloudImpl(
      const PclPointCloudPtr<pcl::PointXYZI>& target,
      const PclPointCloudPtr<pcl::PointXYZI>& source,
      const aslam::Transformation& prior_T_target_source) override;

  RegistrationResult createResultFromTransformation(
      const PclPointCloudPtr<pcl::PointXYZI>& source, const bool accept_match,
      PointMatcher<double>::TransformationParameters&& T) const noexcept;

 private:
  bool isValidCovariance(const Eigen::MatrixXd& cov) const;

  PointMatcher<double>::ICP icp_;
  std::unique_ptr<PointMatcher<double>::DataPointsFilters> input_filters_;
  PointMatcher<double>::DataPoints::Labels feature_labels_;
  PointMatcher<double>::DataPoints::Labels descriptor_labels_;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_ALIGNMENT_LPM_ALIGNMENT_H_
