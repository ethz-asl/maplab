#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_TRACKER_GYRO_AIDED_LASER_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_TRACKER_GYRO_AIDED_LASER_H_

#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <feature-tracking-pipelines/feature-pipeline-base.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

namespace feature_tracking_pipelines {
class FeatureTrackerGyroAidedLaser {
 public:
  typedef std::function<void(const std::shared_ptr<FeaturePipelineDebugData>&)>
      DebugDataCallback;

  MAPLAB_POINTER_TYPEDEFS(FeatureTrackerGyroAidedLaser);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FeatureTrackerGyroAidedLaser() = delete;

  // Takes ownership of the tracking-pipeline.
  explicit FeatureTrackerGyroAidedLaser(
      const aslam::NCamera& camera_system,
      FeatureTrackingPipelineBase* track_pipeline);

  bool processNFrame(
      pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
      const aslam::Quaternion& q_Ikp1_Ik, aslam::VisualNFrame* in_out_nframe);

  void setCurrentImuBias(
      int64_t timestamp_ns, const Eigen::Matrix<double, 6, 1>& imu_bias);

  void setOptionalDebugDataCallback(const DebugDataCallback& cb) {
    CHECK(cb);
    debug_data_callback_ = cb;
  }

 private:
  bool hasUpToDateImuBias(const int64_t current_timestamp_ns) const;

  void integrateInterframeImuRotation(
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements,
      aslam::Quaternion* q_Ikp1_Ik) const;

  aslam::NCamera camera_system_;

  // TODO(schneith): Consider buffering the biases and interpolate to better
  // support the non-realtime datasources.
  Eigen::Matrix<double, 6, 1> current_imu_bias_;
  int64_t current_imu_bias_timestamp_nanoseconds_;
  mutable std::mutex m_current_imu_bias_;

  std::vector<feature_tracking_pipelines::KeyframeFeatures>
      prev_nframe_features_;
  std::vector<cv::Mat> prev_nframe_images_;
  int64_t prev_nframe_timestamp_ns_;
  std::mutex m_prev_nframe_data_;

  std::unique_ptr<feature_tracking_pipelines::FeatureTrackingPipelineBase>
      tracker_pipeline_;

  DebugDataCallback debug_data_callback_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_TRACKER_GYRO_AIDED_LASER_H_
