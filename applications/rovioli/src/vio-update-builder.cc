#include "rovioli/vio-update-builder.h"

#include <maplab-common/interpolation-helpers.h>

namespace rovioli {

VioUpdateBuilder::VioUpdateBuilder()
    : last_received_timestamp_synced_nframe_queue_(
          aslam::time::getInvalidTime()),
      last_received_timestamp_rovio_estimate_queue(
          aslam::time::nanoSecondsToSeconds(aslam::time::getInvalidTime())) {}

void VioUpdateBuilder::processSynchronizedNFrameImu(
    const vio::SynchronizedNFrameImu::ConstPtr& synced_nframe_imu) {
  CHECK(synced_nframe_imu != nullptr);
  const int64_t timestamp_nframe_ns =
      synced_nframe_imu->nframe->getMaxTimestampNanoseconds();
  CHECK_GT(timestamp_nframe_ns, last_received_timestamp_synced_nframe_queue_);
  last_received_timestamp_synced_nframe_queue_ = timestamp_nframe_ns;

  std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
  synced_nframe_imu_queue_.push(synced_nframe_imu);
  findMatchAndPublish();
}

void VioUpdateBuilder::processRovioEstimate(
    const RovioEstimate::ConstPtr& rovio_estimate) {
  CHECK(rovio_estimate != nullptr);
  const double timestamp_rovio_estimate_s = rovio_estimate->timestamp_s;
  CHECK_GT(
      timestamp_rovio_estimate_s, last_received_timestamp_rovio_estimate_queue);
  last_received_timestamp_rovio_estimate_queue = timestamp_rovio_estimate_s;

  std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
  rovio_estimate_queue_.push_back(rovio_estimate);
  findMatchAndPublish();
}

void VioUpdateBuilder::processLocalizationResult(
    const vio::LocalizationResult::ConstPtr& localization_result) {
  CHECK(localization_result);
  std::lock_guard<std::mutex> lock(mutex_last_localization_state_);
  switch (localization_result->localization_type) {
    case vio::LocalizationResult::LocalizationMode::kGlobal:
      last_localization_state_ = vio::LocalizationState::kLocalized;
      break;
    case vio::LocalizationResult::LocalizationMode::kMapTracking:
      last_localization_state_ = vio::LocalizationState::kMapTracking;
      break;
  }
}

void VioUpdateBuilder::findMatchAndPublish() {
  std::lock_guard<std::recursive_mutex> lock(queue_mutex_);

  if (synced_nframe_imu_queue_.empty() || rovio_estimate_queue_.empty()) {
    // Nothing to do.
    return;
  }
  const vio::SynchronizedNFrameImu::ConstPtr& oldest_unmatched_synced_nframe =
      synced_nframe_imu_queue_.front();
  const int64_t timestamp_nframe_ns =
      oldest_unmatched_synced_nframe->nframe->getMinTimestampNanoseconds();

  // We need to use iterator instead of const_iterator because erase isn't
  // defined for const_iterators in g++ 4.8.
  RovioEstimateQueue::iterator it_rovio_estimate_before_nframe =
      rovio_estimate_queue_.end();
  RovioEstimateQueue::iterator it_rovio_estimate_after_nframe =
      rovio_estimate_queue_.end();

  bool found_exact_match = false;
  bool found_matches_to_interpolate = false;
  // Need at least two values for interpolation.
  for (it_rovio_estimate_before_nframe = rovio_estimate_queue_.begin();
       it_rovio_estimate_before_nframe != rovio_estimate_queue_.end();
       ++it_rovio_estimate_before_nframe) {
    it_rovio_estimate_after_nframe = it_rovio_estimate_before_nframe + 1;
    // Check if exact match.
    if (aslam::time::secondsToNanoSeconds(
            (*it_rovio_estimate_before_nframe)->timestamp_s) ==
        timestamp_nframe_ns) {
      found_exact_match = true;
      break;
    }
    if (it_rovio_estimate_after_nframe != rovio_estimate_queue_.end() &&
        aslam::time::secondsToNanoSeconds(
            (*it_rovio_estimate_before_nframe)->timestamp_s) <=
            timestamp_nframe_ns &&
        aslam::time::secondsToNanoSeconds(
            (*it_rovio_estimate_after_nframe)->timestamp_s) >
            timestamp_nframe_ns) {
      // Found matching vi nodes.
      found_matches_to_interpolate = true;
      break;
    }
  }

  if (!found_exact_match && !found_matches_to_interpolate) {
    return;
  }

  CHECK(it_rovio_estimate_before_nframe != rovio_estimate_queue_.end());
  CHECK(
      found_exact_match ||
      it_rovio_estimate_after_nframe != rovio_estimate_queue_.end());
  CHECK(it_rovio_estimate_before_nframe != it_rovio_estimate_after_nframe);
  const RovioEstimate::ConstPtr& rovio_estimate_before_nframe =
      *it_rovio_estimate_before_nframe;
  const RovioEstimate::ConstPtr& rovio_estimate_after_nframe =
      *it_rovio_estimate_after_nframe;

  // Build VioUpdate.
  vio::VioUpdate::Ptr vio_update = aligned_shared<vio::VioUpdate>();
  vio_update->timestamp_ns = timestamp_nframe_ns;
  vio_update->keyframe_and_imudata = oldest_unmatched_synced_nframe;
  if (found_exact_match) {
    vio_update->vinode = rovio_estimate_before_nframe->vinode;

    if (rovio_estimate_before_nframe->has_T_G_M) {
      vio_update->T_G_M = rovio_estimate_before_nframe->T_G_M;
    }
  } else {
    // Need to interpolate ViNode.
    const int64_t t_before = aslam::time::secondsToNanoSeconds(
        rovio_estimate_before_nframe->timestamp_s);
    const int64_t t_after = aslam::time::secondsToNanoSeconds(
        rovio_estimate_after_nframe->timestamp_s);

    vio::ViNodeState interpolated_vi_node;
    interpolateViNodeState(
        t_before, rovio_estimate_before_nframe->vinode, t_after,
        rovio_estimate_after_nframe->vinode, timestamp_nframe_ns,
        &interpolated_vi_node);
    vio_update->vinode = interpolated_vi_node;

    if (rovio_estimate_before_nframe->has_T_G_M &&
        rovio_estimate_after_nframe->has_T_G_M) {
      common::interpolateTransformation(
          t_before, rovio_estimate_before_nframe->T_G_M, t_after,
          rovio_estimate_after_nframe->T_G_M, timestamp_nframe_ns,
          &vio_update->T_G_M);
    }
  }
  vio_update->vio_state = vio::EstimatorState::kRunning;
  vio_update->vio_update_type = vio::UpdateType::kNormalUpdate;
  {
    std::lock_guard<std::mutex> lock(mutex_last_localization_state_);
    vio_update->localization_state = last_localization_state_;
    last_localization_state_ = vio::LocalizationState::kUninitialized;
  }

  // Publish VIO update.
  CHECK(vio_update_publish_function_);
  vio_update_publish_function_(vio_update);

  // Clean up queues.
  if (it_rovio_estimate_before_nframe != rovio_estimate_queue_.begin()) {
    if (found_exact_match) {
      rovio_estimate_queue_.erase(
          rovio_estimate_queue_.begin(), it_rovio_estimate_before_nframe);
    } else {
      // Keep the two ViNodeStates that were used for interpolation as a
      // subsequent SynchronizedNFrameImu may need to be interpolated between
      // those two points again.
      rovio_estimate_queue_.erase(
          rovio_estimate_queue_.begin(), it_rovio_estimate_before_nframe - 1);
    }
  }
  synced_nframe_imu_queue_.pop();
}

void VioUpdateBuilder::interpolateViNodeState(
    const int64_t timestamp_ns_a, const vio::ViNodeState& vi_node_a,
    const int64_t timestamp_ns_b, const vio::ViNodeState& vi_node_b,
    const int64_t timestamp_ns_interpolated,
    vio::ViNodeState* vi_node_interpolated) {
  CHECK_NOTNULL(vi_node_interpolated);
  CHECK_LT(timestamp_ns_a, timestamp_ns_b);
  CHECK_LE(timestamp_ns_a, timestamp_ns_interpolated);
  CHECK_LE(timestamp_ns_interpolated, timestamp_ns_b);

  // Interpolate pose.
  aslam::Transformation interpolated_T_M_I;
  common::interpolateTransformation(
      timestamp_ns_a, vi_node_a.get_T_M_I(), timestamp_ns_b,
      vi_node_b.get_T_M_I(), timestamp_ns_interpolated, &interpolated_T_M_I);
  vi_node_interpolated->set_T_M_I(interpolated_T_M_I);

  // Interpolate velocity.
  Eigen::Vector3d interpolated_v_M_I;
  common::linerarInterpolation(
      timestamp_ns_a, vi_node_a.get_v_M_I(), timestamp_ns_b,
      vi_node_b.get_v_M_I(), timestamp_ns_interpolated, &interpolated_v_M_I);
  vi_node_interpolated->set_v_M_I(interpolated_v_M_I);

  // Interpolate biases.
  Eigen::Vector3d interpolated_acc_bias, interpolated_gyro_bias;
  common::linerarInterpolation(
      timestamp_ns_a, vi_node_a.getAccBias(), timestamp_ns_b,
      vi_node_b.getAccBias(), timestamp_ns_interpolated,
      &interpolated_acc_bias);
  common::linerarInterpolation(
      timestamp_ns_a, vi_node_a.getGyroBias(), timestamp_ns_b,
      vi_node_b.getGyroBias(), timestamp_ns_interpolated,
      &interpolated_gyro_bias);
  vi_node_interpolated->setAccBias(interpolated_acc_bias);
  vi_node_interpolated->setGyroBias(interpolated_gyro_bias);
}

}  // namespace rovioli
