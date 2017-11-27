#include "rovioli/feature-tracking.h"

#include <maplab-common/conversions.h>

namespace rovioli {

FeatureTracking::FeatureTracking(
    const aslam::NCamera::Ptr& camera_system, const vi_map::Imu& imu_sensor)
    : camera_system_(camera_system),
      imu_sensor_(imu_sensor),
      current_imu_bias_(Eigen::Matrix<double, 6, 1>::Zero()),
      current_imu_bias_timestamp_nanoseconds_(aslam::time::getInvalidTime()),
      previous_nframe_timestamp_ns_(-1),
      tracker_(camera_system) {
  CHECK(camera_system_ != nullptr);
}

bool FeatureTracking::trackSynchronizedNFrameImuCallback(
    const vio::SynchronizedNFrameImu::Ptr& synced_nframe_imu) {
  CHECK(synced_nframe_imu != nullptr);
  std::lock_guard<std::mutex> lock(m_previous_synced_nframe_imu_);

  // The first frame will not contain any tracking information on the first
  // call, but it will be added in the second call.
  if (previous_synced_nframe_imu_ == nullptr) {
    previous_synced_nframe_imu_ = synced_nframe_imu;
    previous_nframe_timestamp_ns_ =
        synced_nframe_imu->nframe->getMinTimestampNanoseconds();
    return false;
  }

  // Check if the IMU bias is up to date, if not - use zero.
  if (!hasUpToDateImuBias(
          synced_nframe_imu->nframe->getMinTimestampNanoseconds())) {
    LOG(WARNING) << "No bias from the estimator available. Assuming zero bias.";
    std::unique_lock<std::mutex> bias_lock(m_current_imu_bias_);
    current_imu_bias_.setZero();
  }

  // Preintegrate the IMU measurements.
  aslam::Quaternion q_Ikp1_Ik;
  integrateInterframeImuRotation(
      synced_nframe_imu->imu_timestamps, synced_nframe_imu->imu_measurements,
      &q_Ikp1_Ik);

  CHECK(previous_synced_nframe_imu_ != nullptr);
  aslam::FrameToFrameMatchesList inlier_matches_kp1_k;
  aslam::FrameToFrameMatchesList outlier_matches_kp1_k;
  CHECK_GT(
      synced_nframe_imu->nframe->getMinTimestampNanoseconds(),
      previous_synced_nframe_imu_->nframe->getMinTimestampNanoseconds());
  tracker_.trackFeaturesNFrame(
      q_Ikp1_Ik, synced_nframe_imu->nframe.get(),
      previous_synced_nframe_imu_->nframe.get(), &inlier_matches_kp1_k,
      &outlier_matches_kp1_k);

  previous_synced_nframe_imu_ = synced_nframe_imu;
  return true;
}

void FeatureTracking::setCurrentImuBias(
    const RovioEstimate::ConstPtr& rovio_estimate) {
  CHECK(rovio_estimate != nullptr);

  const int64_t bias_timestamp_ns =
      rovio_estimate->timestamp_s * kSecondsToNanoSeconds;

  // Only update the bias if we have a newer measurement.
  std::unique_lock<std::mutex> lock(m_current_imu_bias_);
  if (bias_timestamp_ns > current_imu_bias_timestamp_nanoseconds_) {
    current_imu_bias_timestamp_nanoseconds_ =
        rovio_estimate->timestamp_s * kSecondsToNanoSeconds;
    current_imu_bias_ = rovio_estimate->vinode.getImuBias();
    VLOG(5) << "Updated IMU bias in Pipeline node.";
  } else {
    LOG(WARNING) << "Received an IMU bias estimate that has an earlier "
                 << "timestamp than the previous one. Previous timestamp: "
                 << current_imu_bias_timestamp_nanoseconds_
                 << "ns, received timestamp: " << bias_timestamp_ns << "ns.";
  }
}

bool FeatureTracking::hasUpToDateImuBias(
    const int64_t current_timestamp_ns) const {
  std::unique_lock<std::mutex> lock(m_current_imu_bias_);
  if (current_imu_bias_timestamp_nanoseconds_ == -1) {
    // No bias was ever set.
    return false;
  }
  constexpr int64_t kImuBiasAgeThresholdNs = 10 * kSecondsToNanoSeconds;
  if (current_timestamp_ns - current_imu_bias_timestamp_nanoseconds_ >
      kImuBiasAgeThresholdNs) {
    // The bias estimate is not up to date.
    return false;
  }
  return true;
}

void FeatureTracking::integrateInterframeImuRotation(
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements,
    aslam::Quaternion* q_Ikp1_Ik) const {
  CHECK_NOTNULL(q_Ikp1_Ik);
  CHECK_GT(imu_timestamps.cols(), 2);
  CHECK_EQ(imu_measurements.cols(), imu_timestamps.cols());

  q_Ikp1_Ik->setIdentity();
  for (int i = 1; i < imu_measurements.cols(); ++i) {
    const double delta_s =
        (imu_timestamps(i) - imu_timestamps(i - 1)) * kNanosecondsToSeconds;
    CHECK_GT(delta_s, 0);
    std::unique_lock<std::mutex> bias_lock(m_current_imu_bias_);
    const Eigen::Vector3d gyro_measurement =
        imu_measurements.col(i).tail<3>() - current_imu_bias_.tail<3>();
    bias_lock.unlock();

    *q_Ikp1_Ik =
        *q_Ikp1_Ik * aslam::Quaternion::exp(gyro_measurement * delta_s);
  }
  // We actually need to inverse the rotation so that transform from Ikp1 to Ik.
  *q_Ikp1_Ik = q_Ikp1_Ik->inverse();
}

}  // namespace rovioli
