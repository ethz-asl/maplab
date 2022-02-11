#include "maplab-node/map-update-builder.h"

#include <maplab-common/interpolation-helpers.h>

namespace maplab {

MapUpdateBuilder::MapUpdateBuilder(
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : T_M_B_buffer_(T_M_B_buffer),
      last_received_timestamp_tracked_nframe_(aslam::time::getInvalidTime()) {}

void MapUpdateBuilder::processTrackedNFrame(
    const vio::SynchronizedNFrame::ConstPtr& tracked_nframe) {
  CHECK(tracked_nframe != nullptr);

  std::lock_guard<std::mutex> lock(process_nframe_mutex_);
  VLOG(3) << "[MaplabNode-MapUpdateBuilder] Received synchronized visual frame";

  CHECK_GT(
      tracked_nframe->nframe->getMaxTimestampNanoseconds(),
      last_received_timestamp_tracked_nframe_.load());
  const int64_t timestamp_nframe_ns =
      tracked_nframe->nframe->getMinTimestampNanoseconds();
  vio::MapUpdate::Ptr map_update = aligned_shared<vio::MapUpdate>();

  // Try to find/interpolate an odometry estimate for this nframe.
  vio_common::PoseLookupBuffer::ResultStatus vinode_lookup_result =
      T_M_B_buffer_.interpolateViNodeStateAt(
          timestamp_nframe_ns, &(map_update->vinode));
  CHECK(
      vinode_lookup_result !=
      vio_common::PoseLookupBuffer::ResultStatus::kFailedNotYetAvailable);
  CHECK(
      vinode_lookup_result !=
      vio_common::PoseLookupBuffer::ResultStatus::kFailedWillNeverSucceed);

  // If this is the first frame, we don't need to associate imu data with it,
  // since there is no edge incomming.
  bool skip_frame = false;
  if (aslam::time::isValidTime(
          last_received_timestamp_tracked_nframe_.load())) {
    // Match the imu data. There is no need to wait here, since the synchronizer
    // ensures this data is available already.
    const int64_t kWaitTimeoutNanoseconds = 0;
    vio_common::ImuMeasurementBuffer::QueryResult result =
        T_M_B_buffer_.imu_buffer().getImuDataInterpolatedBordersBlocking(
            last_received_timestamp_tracked_nframe_, timestamp_nframe_ns,
            kWaitTimeoutNanoseconds, &map_update->imu_timestamps,
            &map_update->imu_measurements);

    CHECK(
        result !=
        vio_common::ImuMeasurementBuffer::QueryResult::kDataNotYetAvailable)
        << "This should never happen, as it is ensure by the synchronizer";

    if (result ==
        vio_common::ImuMeasurementBuffer::QueryResult::kQueueShutdown) {
      // Shutdown.
      return;
    } else if (
        result ==
        vio_common::ImuMeasurementBuffer::QueryResult::kDataNeverAvailable) {
      LOG(ERROR) << "[MaplabNode-MapUpdateBuilder] No IMU data available "
                 << "for this key frame! This can happen if we are lagging "
                 << "behind and the data has already been released by the "
                 << "synchronizer. Skipping NFrame.";

      skip_frame = true;
    } else {
      CHECK(
          result ==
          vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable);
    }
  }

  if (!skip_frame) {
    // Build map update.
    map_update->timestamp_ns = timestamp_nframe_ns;
    map_update->keyframe = tracked_nframe;
    map_update->vio_state = vio::EstimatorState::kRunning;
    map_update->map_update_type = vio::UpdateType::kNormalUpdate;

    // Publish map update.
    VLOG(3) << "[MaplabNode-MapUpdateBuilder] Published a MapUpdate.";
    CHECK(map_update_publish_function_);
    map_update_publish_function_(map_update);
  }

  last_received_timestamp_tracked_nframe_.store(
      tracked_nframe->nframe->getMinTimestampNanoseconds());
}

}  // namespace maplab
