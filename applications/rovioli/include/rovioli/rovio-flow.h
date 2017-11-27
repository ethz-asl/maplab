#ifndef ROVIOLI_ROVIO_FLOW_H_
#define ROVIOLI_ROVIO_FLOW_H_

#include <functional>
#include <memory>
#include <vector>

#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

#include "rovioli/rovio-estimate.h"
#include "rovioli/rovio-factory.h"

namespace rovioli {
class RovioFlow {
 public:
  explicit RovioFlow(
      const aslam::NCamera& camera_calibration,
      const vi_map::ImuSigmas& imu_sigmas);

  void attachToMessageFlow(message_flow::MessageFlow* flow);
  void processRovioUpdate(const rovio::RovioState& state);

 private:
  std::unique_ptr<rovio::RovioInterface> rovio_interface_;
  std::function<void(const RovioEstimate::ConstPtr&)> publish_rovio_estimates_;

  // Indicates if the camera at the corresponding index should be used for
  // motion tracking.
  std::vector<char> is_camera_idx_active_in_motion_tracking_;
};
}  // namespace rovioli
#endif  // ROVIOLI_ROVIO_FLOW_H_
