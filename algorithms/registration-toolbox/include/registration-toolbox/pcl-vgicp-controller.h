#ifndef REGISTRATION_TOOLBOX_PCL_VGICP_CONTROLLER_H_
#define REGISTRATION_TOOLBOX_PCL_VGICP_CONTROLLER_H_

#include <string>
#include "registration-toolbox/alignment/pcl-alignment.h"
#include "registration-toolbox/common/base-controller.h"
#include "registration-toolbox/common/generic-controller.h"

namespace regbox {

// TODO(lbern): Add PCL-GICP specific settings here.

class PclVoxelizedGeneralizedIcpController
    : public BaseController::Registrar<PclVoxelizedGeneralizedIcpController> {
 public:
  explicit PclVoxelizedGeneralizedIcpController(std::string&& name) {}
  RegistrationResult align(
      const resources::PointCloud& target, const resources::PointCloud& source,
      const aslam::Transformation& prior_T_target_source) override;

 private:
  GenericController<VoxelizedGeneralizedIcpAlignment> controller_;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_PCL_VGICP_CONTROLLER_H_
