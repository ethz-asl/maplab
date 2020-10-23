#ifndef REGISTRATION_TOOLBOX_LOAM_CONTROLLER_H_
#define REGISTRATION_TOOLBOX_LOAM_CONTROLLER_H_

#include <string>
#include "registration-toolbox/alignment/loam-alignment.h"
#include "registration-toolbox/common/base-controller.h"
#include "registration-toolbox/common/generic-controller.h"

namespace regbox {

class LoamController : public BaseController::Registrar<LoamController> {
 public:
  explicit LoamController(std::string&& name) {}
  RegistrationResult align(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
      const aslam::Transformation& prior_T_target_source) override;
  RegistrationResult align(
      const resources::PointCloud& target, const resources::PointCloud& source,
      const aslam::Transformation& prior_T_target_source) override;

 private:
  GenericController<LoamAlignment> controller_;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_LOAM_CONTROLLER_H_
