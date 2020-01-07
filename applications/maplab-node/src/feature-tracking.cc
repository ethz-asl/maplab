#include "maplab-node/feature-tracking.h"

#include <maplab-common/conversions.h>

DEFINE_bool(
    descriptor_rotation_invariance, true,
    "Use rotation invariant descriptors.");

namespace maplab {
FeatureTracking::FeatureTracking(
    const aslam::NCamera::Ptr& camera_system,
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : camera_system_(camera_system),
      T_M_B_buffer_(T_M_B_buffer),
      previous_nframe_timestamp_ns_(-1) {
  CHECK(camera_system_ != nullptr);

  // Initialize the settings from GFlags.
  const feature_tracking::FeatureTrackingDetectorSettings detector_settings;
  feature_tracking::FeatureTrackingExtractorSettings extractor_settings;
  extractor_settings.rotation_invariant = FLAGS_descriptor_rotation_invariance;

  tracker_.reset(new feature_tracking::VOFeatureTrackingPipeline(
      camera_system_, extractor_settings, detector_settings));
}

bool FeatureTracking::trackSynchronizedNFrameCallback(
    const vio::SynchronizedNFrame::Ptr& synced_nframe) {
  CHECK(synced_nframe != nullptr);
  std::lock_guard<std::mutex> lock(m_previous_synced_nframe_imu_);

  const int64_t current_nframe_timestamp_ns =
      synced_nframe->nframe->getMinTimestampNanoseconds();

  // The first frame will not contain any tracking information on the first
  // call, but it will be added in the second call.
  if (!previous_synced_nframe_) {
    previous_synced_nframe_ = synced_nframe;
    previous_nframe_timestamp_ns_ = current_nframe_timestamp_ns;
    return false;
  }

  CHECK(previous_synced_nframe_);
  CHECK_GT(current_nframe_timestamp_ns, previous_nframe_timestamp_ns_);
  CHECK(tracker_);

  // Preintegrate the IMU measurements.
  aslam::Quaternion q_Ikp1_Ik;
  if (!getInterframeRotationEstimate(
          previous_nframe_timestamp_ns_, current_nframe_timestamp_ns,
          &q_Ikp1_Ik)) {
    LOG(WARNING) << "[FeatureTracking] Unable to obtain rotation estimate "
                 << "between visual NFrames, dropping frame! Make sure "
                 << "--odometry_buffer_history_ns corresponds to the max "
                 << "latency of your camera/imu setup!";
    return false;
  }

  aslam::FrameToFrameMatchesList inlier_matches_kp1_k;
  aslam::FrameToFrameMatchesList outlier_matches_kp1_k;
  tracker_->trackFeaturesNFrame(
      q_Ikp1_Ik, synced_nframe->nframe.get(),
      previous_synced_nframe_->nframe.get(), &inlier_matches_kp1_k,
      &outlier_matches_kp1_k);

  previous_synced_nframe_ = synced_nframe;
  previous_nframe_timestamp_ns_ = current_nframe_timestamp_ns;
  return true;
}

bool FeatureTracking::getInterframeRotationEstimate(
    const int64_t previous_nframe_timestamp_ns,
    const int64_t current_nframe_timestamp_ns,
    aslam::Quaternion* q_Ikp1_Ik) const {
  CHECK_NOTNULL(q_Ikp1_Ik);
  CHECK_GT(current_nframe_timestamp_ns, previous_nframe_timestamp_ns);

  switch (T_M_B_buffer_.getRotationBetween(
      current_nframe_timestamp_ns, previous_nframe_timestamp_ns, q_Ikp1_Ik)) {
    case vio_common::PoseLookupBuffer::ResultStatus::kFailedNotYetAvailable:
    // Fall through intended.
    case vio_common::PoseLookupBuffer::ResultStatus::kFailedWillNeverSucceed:
      return false;
    default:
      break;
  }
  return true;
}

}  // namespace maplab
