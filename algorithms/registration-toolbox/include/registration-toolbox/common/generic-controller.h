#ifndef REGISTRATION_TOOLBOX_COMMON_GENERIC_CONTROLLER_H_
#define REGISTRATION_TOOLBOX_COMMON_GENERIC_CONTROLLER_H_

#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map-resources/resource-conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <resources-common/point-cloud.h>

#include "registration-toolbox/alignment/base-alignment.h"

namespace regbox {

template <typename T_align>
class GenericController {
 public:
  GenericController() {
    alignment_.reset(new T_align());
    CHECK_NOTNULL(alignment_);
  }

  template <typename T_input>
  RegistrationResult align(const T_input& target, const T_input& source);

  template <typename T_input>
  RegistrationResult align(
      const T_input& target, const T_input& source,
      const aslam::Transformation& prior_T_target_source);

  T_align& getAlignment() {
    CHECK_NOTNULL(alignment_);
    return *alignment_;
  }

  const T_align& getAlignment() const {
    CHECK_NOTNULL(alignment_);
    return *alignment_;
  }

 private:
  std::unique_ptr<T_align> alignment_;
};

template <typename T_align>
template <typename T_input>
RegistrationResult GenericController<T_align>::align(
    const T_input& target, const T_input& source) {
  aslam::Transformation T_target_source;
  T_target_source.setIdentity();
  return align(target, source, T_target_source);
}

template <typename T_align>
template <typename T_input>
RegistrationResult GenericController<T_align>::align(
    const T_input& target, const T_input& source,
    const aslam::Transformation& prior_T_target_source) {
  return alignment_->registerCloud(target, source, prior_T_target_source);
}

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_COMMON_GENERIC_CONTROLLER_H__
