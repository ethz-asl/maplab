#ifndef REGISTRATION_TOOLBOX_COMMON_BASE_CONTROLLER_H_
#define REGISTRATION_TOOLBOX_COMMON_BASE_CONTROLLER_H_

#include <string>

#include <resources-common/point-cloud.h>

#include "registration-toolbox/common/registration-factory.h"
#include "registration-toolbox/model/registration-result.h"

namespace regbox {

struct BaseController : RegistrationFactory<BaseController, std::string> {
  explicit BaseController(Key) {}
  virtual RegistrationResult align(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
      const aslam::Transformation& prior_T_target_source) = 0;
  virtual RegistrationResult align(
      const resources::PointCloud& target, const resources::PointCloud& source,
      const aslam::Transformation& prior_T_target_source) = 0;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_COMMON_BASE_CONTROLLER_H_
