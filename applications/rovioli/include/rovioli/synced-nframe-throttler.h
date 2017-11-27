#ifndef ROVIOLI_SYNCED_NFRAME_THROTTLER_H_
#define ROVIOLI_SYNCED_NFRAME_THROTTLER_H_

#include <memory>
#include <vector>

#include <vio-common/imu-measurements-buffer.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>

namespace rovioli {

class SyncedNFrameThrottler {
 public:
  MAPLAB_POINTER_TYPEDEFS(SyncedNFrameThrottler);

  SyncedNFrameThrottler();

  bool shouldPublishNFrame(
      const vio::SynchronizedNFrameImu::ConstPtr& synced_nframe_imu);

 private:
  // Minimum interval of the output, as configured by the maximum frequency
  // flag.
  const int64_t min_nframe_timestamp_diff_ns_;

  int64_t previous_nframe_timestamp_ns_;
  std::mutex m_previous_nframe_timestamp_ns_;
};

}  // namespace rovioli

#endif  // ROVIOLI_SYNCED_NFRAME_THROTTLER_H_
