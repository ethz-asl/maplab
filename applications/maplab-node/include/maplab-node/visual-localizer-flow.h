#ifndef MAPLAB_NODE_VISUAL_LOCALIZER_FLOW_H_
#define MAPLAB_NODE_VISUAL_LOCALIZER_FLOW_H_

#include <memory>

#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-flow.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "maplab-node/visual-localizer.h"

namespace maplab {
class VisualLocalizerFlow {
 public:
  VisualLocalizerFlow(
      const vi_map::SensorManager& sensor_manager,
      const vio_common::PoseLookupBuffer& T_M_B_buffer,
      const bool visualize_localization);

  void setLocalizationMap(
      std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map);

  void attachToMessageFlow(message_flow::MessageFlow* flow);

 private:
  void processTrackedNFrame(
      const vio::SynchronizedNFrame::ConstPtr& nframe_imu);

  VisualLocalizer localizer_;

  const vio_common::PoseLookupBuffer& T_M_B_buffer_;

  std::function<void(common::LocalizationResult::ConstPtr)>
      publish_localization_result_;

  // All members below are used for throttling the localizations.
  const int64_t min_localization_timestamp_diff_ns_;
  int64_t previous_nframe_timestamp_ns_;
  mutable std::mutex m_previous_nframe_timestamp_ns_;
};
}  // namespace maplab
#endif  // MAPLAB_NODE_VISUAL_LOCALIZER_FLOW_H_
