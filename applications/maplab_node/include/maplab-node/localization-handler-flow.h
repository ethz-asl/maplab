#ifndef MAPLAB_NODE_LOCALIZATION_HANDLER_FLOW_H_
#define MAPLAB_NODE_LOCALIZATION_HANDLER_FLOW_H_

#include <functional>
#include <memory>

#include <message-flow/message-flow.h>
#include <vi-map/sensor-manager.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "maplab-node/localization-handler.h"

namespace maplab {
class LocalizationHandlerFlow {
 public:
  explicit LocalizationHandlerFlow(
      const vi_map::SensorManager& sensor_manager,
      const vio_common::PoseLookupBuffer& T_M_B_buffer);

  void attachToMessageFlow(message_flow::MessageFlow* flow);

 private:
  void processLocalizationResult(
      const common::LocalizationResult::ConstPtr& localization_result);

  LocalizationHandler localization_handler_;

  std::function<void(common::LocalizationResult::ConstPtr)>
      publish_fused_localization_result_;
};
}  // namespace maplab
#endif  // MAPLAB_NODE_LOCALIZATION_HANDLER_FLOW_H_
