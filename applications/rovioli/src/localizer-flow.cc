#include "rovioli/localizer-flow.h"

#include <gflags/gflags.h>
#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-flow.h>
#include <vio-common/vio-types.h>

#include "rovioli/flow-topics.h"
#include "rovioli/localizer.h"

DEFINE_double(
    vio_max_localization_frequency_hz, 2.0,
    "Maximum localization frequency [hz].");

namespace rovioli {
LocalizerFlow::LocalizerFlow(
    const summary_map::LocalizationSummaryMap& localization_map,
    const bool visualize_localization)
    : localizer_(localization_map, visualize_localization),
      min_localization_timestamp_diff_ns_(
          kSecondsToNanoSeconds / FLAGS_vio_max_localization_frequency_hz),
      previous_nframe_timestamp_ns_(-1) {
  CHECK_GT(FLAGS_vio_max_localization_frequency_hz, 0.);
}

void LocalizerFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "LocalizerFlow";

  // Subscribe-publish: nframe to localization.
  publish_localization_result_ =
      flow->registerPublisher<message_flow_topics::LOCALIZATION_RESULT>();

  // NOTE: the publisher function pointer is copied intentionally; otherwise
  // we would capture a reference to a temporary.
  flow->registerSubscriber<message_flow_topics::TRACKED_NFRAMES_AND_IMU>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      std::bind(
          &LocalizerFlow::processTrackedNFrameAndImu, this,
          std::placeholders::_1));
}

void LocalizerFlow::processTrackedNFrameAndImu(
    const vio::SynchronizedNFrameImu::ConstPtr& nframe_imu) {
  CHECK(nframe_imu);
  // Throttle the localization rate.
  const int64_t current_timestamp =
      nframe_imu->nframe->getMinTimestampNanoseconds();
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
  vio::LocalizationResult::Ptr loc_result(new vio::LocalizationResult);
  const bool success =
      this->localizer_.localizeNFrame(nframe_imu->nframe, loc_result.get());
  if (success) {
    publish_localization_result_(loc_result);
  }
}
}  // namespace rovioli
