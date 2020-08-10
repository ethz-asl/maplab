#ifndef REGISTRATION_TOOLBOX_LPM_ICP_CONTROLLER_H_
#define REGISTRATION_TOOLBOX_LPM_ICP_CONTROLLER_H_

#include <string>
#include "registration-toolbox/alignment/lpm-alignment.h"
#include "registration-toolbox/common/base-controller.h"
#include "registration-toolbox/common/generic-controller.h"

namespace regbox {

// TODO(lbern): Add LPM-ICP specific settings here.

class LpmIcpController : public BaseController::Registrar<LpmIcpController> {
 public:
  explicit LpmIcpController(std::string&& name) {}
  RegistrationResult align(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
      const aslam::Transformation& prior_T_target_source) override;
  RegistrationResult align(
      const resources::PointCloud& target, const resources::PointCloud& source,
      const aslam::Transformation& prior_T_target_source) override;

 private:
  GenericController<LpmAlignment> controller_;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_LPM_ICP_CONTROLLER_H_
