#ifndef REGISTRATION_TOOLBOX_MOCK_CONTROLLER_H_
#define REGISTRATION_TOOLBOX_MOCK_CONTROLLER_H_

#include <string>
#include "registration-toolbox/common/base-controller.h"
#include "registration-toolbox/common/generic-controller.h"

namespace regbox {

class MockController : public BaseController::Registrar<MockController> {
 public:
  explicit MockController(std::string&& name) {}
  RegistrationResult align(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
      const aslam::Transformation& prior_T_target_source) override;
  RegistrationResult align(
      const resources::PointCloud& target, const resources::PointCloud& source,
      const aslam::Transformation& prior_T_target_source) override;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_MOCK_CONTROLLER_H_
