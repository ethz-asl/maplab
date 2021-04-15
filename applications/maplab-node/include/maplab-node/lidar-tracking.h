#ifndef MAPLAB_NODE_LIDAR_TRACKING_H_
#define MAPLAB_NODE_LIDAR_TRACKING_H_

#include <Eigen/Core>
#include <feature-tracking-pipelines/feature-tracker-gyro-aided-laser.h>
#include <memory>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <vector>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
//#include <feature-tracking-pipelines/feature-tracker-gyro-aided.h>
#include <aslam/pipeline/visual-npipeline.h>

#include "maplab-node/odometry-estimate.h"

namespace maplab {

class LidarTracking {
 public:
  MAPLAB_POINTER_TYPEDEFS(LidarTracking);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LidarTracking() = delete;

  explicit LidarTracking(
      aslam::NCamera::Ptr ncamera,
      const vio_common::PoseLookupBuffer& T_M_B_buffer);

  bool trackSynchronizedLidarMeasurementCallback(
      pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
      const cv::Mat& projected_lidar_image,
      const int64_t current_lidar_timestamp_ns);

  aslam::VisualNFrame::Ptr getLastTrackedLidarNFrame() const;

 private:
  bool getInterframeRotationEstimate(
      const int64_t previous_nframe_timestamp_ns,
      const int64_t current_nframe_timestamp_ns,
      aslam::Quaternion* q_Ikp1_Ik) const;
  void visualizeTracking(
      const aslam::VisualNFrame::Ptr& nframe_k,
      const aslam::VisualNFrame::Ptr& nframe_kp1);

  const vio_common::PoseLookupBuffer& T_M_B_buffer_;
  aslam::VisualNFrame::Ptr previous_synced_meas_;
  int64_t previous_lidar_timestamp_ns_;
  std::mutex m_previous_synced_lidar_meas_;
  const uint8_t max_queue_size_ = 50;
  aslam::VisualNPipeline::Ptr visual_pipeline_;
  std::unique_ptr<feature_tracking_pipelines::FeatureTrackerGyroAidedLaser>
      tracking_pipeline_;
  bool warm_up_done_ = false;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_LIDAR_TRACKING_H_