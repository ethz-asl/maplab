#include "maplab-node/visual-localizer-flow.h"

#include <gflags/gflags.h>
#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-flow.h>
#include <vio-common/vio-types.h>

#include "maplab-node/flow-topics.h"
#include "maplab-node/visual-localizer.h"

DEFINE_double(
    vio_max_localization_frequency_hz, 2.0,
    "Maximum localization frequency [hz].");

namespace maplab {

VisualLocalizerFlow::VisualLocalizerFlow(
    const vi_map::SensorManager& sensor_manager,
    const vio_common::PoseLookupBuffer& T_M_B_buffer,
    const bool visualize_localization)
    : localizer_(sensor_manager, visualize_localization),
      T_M_B_buffer_(T_M_B_buffer),
      min_localization_timestamp_diff_ns_(
          kSecondsToNanoSeconds / FLAGS_vio_max_localization_frequency_hz),
      previous_nframe_timestamp_ns_(-1) {
  CHECK_GT(FLAGS_vio_max_localization_frequency_hz, 0.);
}

void VisualLocalizerFlow::setLocalizationMap(
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map) {
  CHECK(localization_map);
  localizer_.setLocalizationMap(std::move(localization_map));
}

void VisualLocalizerFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "VisualLocalizer";

  // Subscribe-publish: nframe to localization.
  publish_localization_result_ =
      flow->registerPublisher<message_flow_topics::LOCALIZATION_RESULT>();

  // NOTE: the publisher function pointer is copied intentionally; otherwise
  // we would capture a reference to a temporary.
  flow->registerSubscriber<message_flow_topics::TRACKED_NFRAMES>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      std::bind(
          &VisualLocalizerFlow::processTrackedNFrame, this,
          std::placeholders::_1));
}

void VisualLocalizerFlow::processTrackedNFrame(
    const vio::SynchronizedNFrame::ConstPtr& nframe) {
  CHECK(nframe);
  // Throttle the localization rate.
  const int64_t current_timestamp =
      nframe->nframe->getMinTimestampNanoseconds();
  {
    std::unique_lock<std::mutex> lock(m_previous_nframe_timestamp_ns_);
    if ((previous_nframe_timestamp_ns_ != -1) &&
        ((current_timestamp - previous_nframe_timestamp_ns_) <
         min_localization_timestamp_diff_ns_)) {
      // Skip this frame.
      return;
    }
    CHECK_GT(current_timestamp, previous_nframe_timestamp_ns_);
    previous_nframe_timestamp_ns_ = current_timestamp;
  }

  // Localize this nframe.
  common::LocalizationResult::Ptr loc_result(new vio::LocalizationResult);

  vio::LocalizationResult* visual_loc_result =
      static_cast<vio::LocalizationResult*>(loc_result.get());

  const bool success =
      this->localizer_.localizeNFrame(nframe->nframe, visual_loc_result);
  if (success) {
    publish_localization_result_(loc_result);
  }
}
}  // namespace maplab
