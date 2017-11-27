#ifndef ROVIOLI_FEATURE_TRACKING_H_
#define ROVIOLI_FEATURE_TRACKING_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <feature-tracking/vo-feature-tracking-pipeline.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>

#include "rovioli/rovio-estimate.h"

namespace rovioli {

class FeatureTracking {
 public:
  MAPLAB_POINTER_TYPEDEFS(FeatureTracking);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FeatureTracking() = delete;

  explicit FeatureTracking(
      const aslam::NCamera::Ptr& camera_system, const vi_map::Imu& imu_sensor);

  bool trackSynchronizedNFrameImuCallback(
      const vio::SynchronizedNFrameImu::Ptr& synced_nframe_imu);

  void setCurrentImuBias(const RovioEstimate::ConstPtr& rovio_estimate);

 private:
  bool hasUpToDateImuBias(const int64_t current_timestamp_ns) const;

  void integrateInterframeImuRotation(
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements,
      aslam::Quaternion* q_Ikp1_Ik) const;

  const aslam::NCamera::Ptr camera_system_;
  const vi_map::Imu imu_sensor_;

  Eigen::Matrix<double, 6, 1> current_imu_bias_;
  int64_t current_imu_bias_timestamp_nanoseconds_;
  mutable std::mutex m_current_imu_bias_;

  vio::SynchronizedNFrameImu::Ptr previous_synced_nframe_imu_;
  int64_t previous_nframe_timestamp_ns_;
  std::mutex m_previous_synced_nframe_imu_;

  feature_tracking::VOFeatureTrackingPipeline tracker_;
};

}  // namespace rovioli

#endif  // ROVIOLI_FEATURE_TRACKING_H_
