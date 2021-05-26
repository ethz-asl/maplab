#ifndef REGISTRATION_TOOLBOX_ALIGNMENT_BASE_ALIGNMENT_H_
#define REGISTRATION_TOOLBOX_ALIGNMENT_BASE_ALIGNMENT_H_

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "registration-toolbox/model/registration-result.h"

namespace regbox {

template <typename T_depth_input>
class BaseAlignment {
 public:
  virtual ~BaseAlignment() = default;
  RegistrationResult registerCloud(
      const T_depth_input& target, const T_depth_input& source,
      const aslam::Transformation& prior_T_target_source);

 protected:
  virtual RegistrationResult registerCloudImpl(
      const T_depth_input& target, const T_depth_input& source,
      const aslam::Transformation& prior_T_target_source) = 0;

  constexpr aslam::Transformation convertEigenToKindr(
      const Eigen::Matrix4d& T) const {
      return aslam::Transformation::constructAndRenormalizeRotation(T);
  }
};

template <typename T_depth_input>
RegistrationResult BaseAlignment<T_depth_input>::registerCloud(
    const T_depth_input& target, const T_depth_input& source,
    const aslam::Transformation& prior_T_target_source) {
  CHECK_NOTNULL(target);
  CHECK_NOTNULL(source);
  return registerCloudImpl(target, source, prior_T_target_source);
}

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_ALIGNMENT_BASE_ALIGNMENT_H_
