#include "maplab-node/map-update-builder.h"

#include <maplab-common/interpolation-helpers.h>

DEFINE_int64(
    localization_buffer_history_ns, aslam::time::seconds(30u),
    "History length of the buffered localization results 6DOF. This buffer is "
    "used for map building to look up the most recent localization results and "
    "incorporate their transformation into the pose graph.");

namespace maplab {

MapUpdateBuilder::MapUpdateBuilder(
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : T_M_B_buffer_(T_M_B_buffer),
      localization_buffer_(FLAGS_localization_buffer_history_ns),
      last_received_timestamp_tracked_nframe_queue_(
          aslam::time::getInvalidTime()),
      last_localization_state_(common::LocalizationState::kUninitialized) {}

void MapUpdateBuilder::processTrackedNFrame(
    const vio::SynchronizedNFrame::ConstPtr& tracked_nframe) {
  CHECK(tracked_nframe != nullptr);
  const int64_t timestamp_nframe_ns =
      tracked_nframe->nframe->getMaxTimestampNanoseconds();
  CHECK_GT(
      timestamp_nframe_ns,
      last_received_timestamp_tracked_nframe_queue_.load());

  std::lock_guard<std::recursive_mutex> lock(queue_mutex_);

  VLOG(3) << "[MaplabNode-MapUpdateBuilder] Received synchronized visual frame";

  tracked_nframe_queue_.push(tracked_nframe);
  findMatchAndPublish();

  last_received_timestamp_tracked_nframe_queue_.store(
      tracked_nframe->nframe->getMinTimestampNanoseconds());
}

void MapUpdateBuilder::processLocalizationResult(
    const common::LocalizationResult::ConstPtr& localization_result) {
  CHECK(localization_result);

  std::lock_guard<std::mutex> lock(localization_buffer_mutex_);
  // Make a copy.
  CHECK(
      localization_result->localization_type ==
      common::LocalizationType::kFused);
  const common::FusedLocalizationResult* fused_localization_result =
      dynamic_cast<const common::FusedLocalizationResult*>(
          localization_result.get());
  CHECK_NOTNULL(fused_localization_result);

  localization_buffer_.addValue(
      localization_result->timestamp_ns, *fused_localization_result);
}

void MapUpdateBuilder::findMatchAndPublish() {
  vio::SynchronizedNFrame::ConstPtr oldest_unmatched_tracked_nframe;

  int64_t timestamp_nframe_ns;
  {
    std::lock_guard<std::recursive_mutex> lock(queue_mutex_);

    if (tracked_nframe_queue_.empty()) {
      // Nothing to do.
      return;
    }
    oldest_unmatched_tracked_nframe = tracked_nframe_queue_.front();
    timestamp_nframe_ns =
        oldest_unmatched_tracked_nframe->nframe->getMinTimestampNanoseconds();
  }

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
          last_received_timestamp_tracked_nframe_queue_.load())) {
    // Match the imu data. There is no need to wait here, since the synchronizer
    // ensures this data is available already.
    const int64_t kWaitTimeoutNanoseconds = 0;
    vio_common::ImuMeasurementBuffer::QueryResult result =
        T_M_B_buffer_.imu_buffer().getImuDataInterpolatedBordersBlocking(
            last_received_timestamp_tracked_nframe_queue_, timestamp_nframe_ns,
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

  // TODO(mfehr): Not sure this works, since the localizations are gonna be
  // slower than the nframes and therefore they will not reach the
  // MapUpdateBuilder in time and will always be taken from the past.

  // Get a localization if one is available for this frame.
  {
    std::lock_guard<std::mutex> lock(localization_buffer_mutex_);

    if (localization_buffer_.empty()) {
      map_update->T_G_M.setIdentity();
      map_update->localization_state = common::LocalizationState::kNotLocalized;
    } else {
      int64_t timestamp_localization_ns = -1;
      common::FusedLocalizationResult localization_result;
      if (!localization_buffer_.getValueAtOrBeforeTime(
              timestamp_nframe_ns, &timestamp_localization_ns,
              &localization_result)) {
        // TODO(mfehr): rm
        LOG(WARNING)
            << "[MaplabNode-MapUpdateBuilder] No localization found for "
            << "the nframe at " << timestamp_nframe_ns;
      } else {
        if (localization_result.is_T_G_M_set) {
          map_update->T_G_M = localization_result.T_G_M;
        } else if (localization_result.is_T_G_B_set) {
          map_update->T_G_M = localization_result.T_G_B *
                              map_update->vinode.get_T_M_I().inverse();
        } else {
          LOG(FATAL) << "[MaplabNode-MapUpdateBuilder] Received localization "
                        "result but "
                        "it doesn't contain any localization transformation!";
        }
        switch (localization_result.localization_mode) {
          case common::LocalizationMode::kGlobal:
            map_update->localization_state =
                common::LocalizationState::kLocalized;
            break;
          case common::LocalizationMode::kMapTracking:
            map_update->localization_state =
                common::LocalizationState::kMapTracking;
            break;
          default:
            LOG(FATAL) << "Unknown localization mode: "
                       << static_cast<int>(
                              localization_result.localization_mode);
        }
      }
    }
  }

  if (!skip_frame) {
    // Build map update.
    map_update->timestamp_ns = timestamp_nframe_ns;
    map_update->keyframe = oldest_unmatched_tracked_nframe;
    map_update->vio_state = vio::EstimatorState::kRunning;
    map_update->map_update_type = vio::UpdateType::kNormalUpdate;

    // Publish map update.
    VLOG(3) << "[MaplabNode-MapUpdateBuilder] Published a MapUpdate.";
    CHECK(map_update_publish_function_);
    map_update_publish_function_(map_update);
  }
  // Clean up the queue.
  tracked_nframe_queue_.pop();
}

}  // namespace maplab
