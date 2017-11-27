#ifndef ROVIOLI_LOCALIZER_FLOW_H_
#define ROVIOLI_LOCALIZER_FLOW_H_

#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-flow.h>
#include <vio-common/vio-types.h>

#include "rovioli/flow-topics.h"
#include "rovioli/localizer.h"

namespace rovioli {

class LocalizerFlow {
 public:
  explicit LocalizerFlow(
      const summary_map::LocalizationSummaryMap& localization_map,
      const bool visualize_localization)
      : localizer_(localization_map, visualize_localization) {}

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    static constexpr char kSubscriberNodeName[] = "LocalizerFlow";

    // Subscribe-publish: nframe to localization.
    std::function<void(vio::LocalizationResult::ConstPtr)> publish_result =
        flow->registerPublisher<message_flow_topics::LOCALIZATION_RESULT>();

    // NOTE: the publisher function pointer is copied intentionally; otherwise
    // we would capture a reference to a temporary.
    flow->registerSubscriber<
        message_flow_topics::THROTTLED_TRACKED_NFRAMES_AND_IMU>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [publish_result,
         this](const vio::SynchronizedNFrameImu::ConstPtr& nframe_imu) {
          CHECK(nframe_imu);
          vio::LocalizationResult::Ptr loc_result(new vio::LocalizationResult);
          const bool success = this->localizer_.localizeNFrame(
              nframe_imu->nframe, loc_result.get());
          if (success) {
            publish_result(loc_result);
          }
        });
  }

 private:
  Localizer localizer_;
};
}  // namespace rovioli
#endif  // ROVIOLI_LOCALIZER_FLOW_H_
