#ifndef ROVIOLI_LOCALIZER_FLOW_H_
#define ROVIOLI_LOCALIZER_FLOW_H_

#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-flow.h>
#include <vio-common/vio-types.h>

#include "rovioli/localizer.h"

namespace rovioli {
class LocalizerFlow {
 public:
  explicit LocalizerFlow(
      const summary_map::LocalizationSummaryMap& localization_map,
      const bool visualize_localization);

  void attachToMessageFlow(message_flow::MessageFlow* flow);

 private:
  void processTrackedNFrameAndImu(
      const vio::SynchronizedNFrameImu::ConstPtr& nframe_imu);

  Localizer localizer_;

  std::function<void(vio::LocalizationResult::ConstPtr)>
      publish_localization_result_;

  // All members below are used for throttling the localizations.
  const int64_t min_localization_timestamp_diff_ns_;
  int64_t previous_nframe_timestamp_ns_;
  mutable std::mutex m_previous_nframe_timestamp_ns_;
};
}  // namespace rovioli
#endif  // ROVIOLI_LOCALIZER_FLOW_H_
