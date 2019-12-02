#ifndef MAPLAB_NODE_FEATURE_TRACKING_H_
#define MAPLAB_NODE_FEATURE_TRACKING_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <feature-tracking/vo-feature-tracking-pipeline.h>
#include <sensors/imu.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>

#include "maplab-node/odometry-estimate.h"

namespace maplab {

class FeatureTracking {
 public:
  MAPLAB_POINTER_TYPEDEFS(FeatureTracking);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FeatureTracking() = delete;

  explicit FeatureTracking(
      const aslam::NCamera::Ptr& camera_system,
      const vio_common::PoseLookupBuffer& T_M_B_buffer);

  bool trackSynchronizedNFrameCallback(
      const vio::SynchronizedNFrame::Ptr& synced_nframe);

 private:
  bool getInterframeRotationEstimate(
      const int64_t previous_nframe_timestamp_ns,
      const int64_t current_nframe_timestamp_ns,
      aslam::Quaternion* q_Ikp1_Ik) const;

  const aslam::NCamera::Ptr camera_system_;
  const vio_common::PoseLookupBuffer& T_M_B_buffer_;

  vio::SynchronizedNFrame::Ptr previous_synced_nframe_;
  int64_t previous_nframe_timestamp_ns_;
  std::mutex m_previous_synced_nframe_imu_;

  std::unique_ptr<feature_tracking::VOFeatureTrackingPipeline> tracker_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_FEATURE_TRACKING_H_
