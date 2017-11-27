#include "rovioli/synced-nframe-throttler.h"

#include <maplab-common/conversions.h>

DEFINE_double(
    vio_throttler_max_output_frequency_hz, 1.0,
    "Maximum output frequency of the synchronized IMU-NFrame structures "
    "from the synchronizer.");

namespace rovioli {

SyncedNFrameThrottler::SyncedNFrameThrottler()
    : min_nframe_timestamp_diff_ns_(
          kSecondsToNanoSeconds / FLAGS_vio_throttler_max_output_frequency_hz),
      previous_nframe_timestamp_ns_(-1) {
  CHECK_GT(FLAGS_vio_throttler_max_output_frequency_hz, 0.);
}

bool SyncedNFrameThrottler::shouldPublishNFrame(
    const vio::SynchronizedNFrameImu::ConstPtr& synced_nframe_imu) {
  CHECK(synced_nframe_imu != nullptr);

  const int64_t current_timestamp =
      synced_nframe_imu->nframe->getMinTimestampNanoseconds();

  std::unique_lock<std::mutex> lock(m_previous_nframe_timestamp_ns_);
  if (previous_nframe_timestamp_ns_ == -1) {
    previous_nframe_timestamp_ns_ = current_timestamp;
    return true;
  }

  CHECK_GE(previous_nframe_timestamp_ns_, 0);
  CHECK_GT(current_timestamp, previous_nframe_timestamp_ns_);
  if (current_timestamp - previous_nframe_timestamp_ns_ <
      min_nframe_timestamp_diff_ns_) {
    return false;
  }

  previous_nframe_timestamp_ns_ = current_timestamp;
  return true;
}

}  // namespace rovioli
