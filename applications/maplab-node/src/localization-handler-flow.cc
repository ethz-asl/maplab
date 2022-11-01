#include "maplab-node/localization-handler-flow.h"

#include <gflags/gflags.h>
#include <message-flow/message-flow.h>
#include <vio-common/vio-types.h>

#include "maplab-node/flow-topics.h"

namespace maplab {

LocalizationHandlerFlow::LocalizationHandlerFlow(
    const vi_map::SensorManager& sensor_manager,
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : localization_handler_(sensor_manager, T_M_B_buffer) {}

void LocalizationHandlerFlow::attachToMessageFlow(
    message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "LocalizationHandler";

  publish_fused_localization_result_ =
      flow->registerPublisher<message_flow_topics::FUSED_LOCALIZATION_RESULT>();

  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      std::bind(
          &LocalizationHandlerFlow::processLocalizationResult, this,
          std::placeholders::_1));
}

void LocalizationHandlerFlow::processLocalizationResult(
    const common::LocalizationResult::ConstPtr& localization_result) {
  CHECK(localization_result);

  common::LocalizationResult::Ptr fused_localization_result =
      std::make_shared<common::FusedLocalizationResult>();

  common::FusedLocalizationResult::Ptr fused_localization_result_ptr =
      std::static_pointer_cast<common::FusedLocalizationResult>(
          fused_localization_result);

  const bool success = localization_handler_.processLocalizationResult(
      localization_result, fused_localization_result_ptr.get());

  if (success) {
    common::LocalizationResult::ConstPtr const_fused_localization_result =
        std::const_pointer_cast<const common::LocalizationResult>(
            fused_localization_result);
    publish_fused_localization_result_(const_fused_localization_result);
  }
}
}  // namespace maplab
