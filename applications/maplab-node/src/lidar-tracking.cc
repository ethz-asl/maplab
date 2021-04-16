#include "maplab-node/lidar-tracking.h"

#include <aslam/matcher/match-helpers.h>
#include <aslam/matcher/match-visualization.h>
#include <aslam/pipeline/visual-pipeline-null.h>
#include <feature-tracking-pipelines/feature-tracker-factory.h>
#include <maplab-common/conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace maplab {
LidarTracking::LidarTracking(
    aslam::NCamera::Ptr camera_system,
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : T_M_B_buffer_(T_M_B_buffer), previous_lidar_timestamp_ns_(-1) {
  tracking_pipeline_.reset(
      new feature_tracking_pipelines::FeatureTrackerGyroAidedLaser(
          *camera_system,
          feature_tracking_pipelines::CreateFeaturePipelineFromGFlags()));

  // Initialize the pipeline.
  static constexpr bool kCopyImages = false;
  std::vector<aslam::VisualPipeline::Ptr> mono_pipelines;
  const std::size_t n_cameras = camera_system->getNumCameras();
  for (size_t camera_idx = 0u; camera_idx < n_cameras; ++camera_idx) {
    mono_pipelines.emplace_back(new aslam::NullVisualPipeline(
        camera_system->getCameraShared(camera_idx), kCopyImages));
  }

  const int64_t kNFrameToleranceNs = 500000;
  constexpr size_t kNumThreads = 1u;
  visual_pipeline_.reset(new aslam::VisualNPipeline(
      kNumThreads, mono_pipelines, camera_system, camera_system,
      kNFrameToleranceNs));
  // cv::namedWindow("Tracking", cv::WINDOW_NORMAL);
}

bool LidarTracking::trackSynchronizedLidarMeasurementCallback(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const cv::Mat& projected_lidar_image,
    const int64_t current_lidar_timestamp_ns) {
  std::lock_guard<std::mutex> lock(m_previous_synced_lidar_meas_);

  if (!visual_pipeline_->processImageBlockingIfFull(
          0, projected_lidar_image, current_lidar_timestamp_ns,
          max_queue_size_)) {
    LOG(ERROR)
        << "[MaplabNode-Synchronizer] Failed to process an image of camera "
        << 0 << " into an NFrame at time " << current_lidar_timestamp_ns
        << "ns!";
    return false;
  }

  aslam::VisualNFrame::Ptr current_nframe = visual_pipeline_->getNext();
  // The first frame will not contain any tracking information on the first
  // call, but it will be added in the second call.
  if (!previous_synced_meas_) {
    previous_synced_meas_ = current_nframe;
    previous_lidar_timestamp_ns_ = current_lidar_timestamp_ns;
    return false;
  }

  // Preintegrate the IMU measurements.
  aslam::Quaternion q_Ikp1_Ik;
  if (!getInterframeRotationEstimate(
          previous_lidar_timestamp_ns_, current_lidar_timestamp_ns,
          &q_Ikp1_Ik)) {
    LOG(WARNING) << "[LidarTracking] Unable to obtain rotation estimate "
                    "between lidar measurements, dropping measurement!";
    return false;
  }
  CHECK(previous_synced_meas_);
  CHECK_GT(current_lidar_timestamp_ns, previous_lidar_timestamp_ns_);
  CHECK(tracking_pipeline_);
  // CHECK(current_nframe);

  if (!current_nframe) {
    VLOG(0) << "Check failed: Current NFrame";
    return false;
  }

  aslam::FrameToFrameMatchesList inlier_matches_kp1_k;
  aslam::FrameToFrameMatchesList outlier_matches_kp1_k;
  tracking_pipeline_->processNFrame(cloud, q_Ikp1_Ik, &(*current_nframe));

  previous_synced_meas_ = current_nframe;
  previous_lidar_timestamp_ns_ = current_lidar_timestamp_ns;
  warm_up_done_ = true;
  return true;
}

bool LidarTracking::getInterframeRotationEstimate(
    const int64_t previous_lidar_timestamp_ns,
    const int64_t current_lidar_timestamp_ns,
    aslam::Quaternion* q_Ikp1_Ik) const {
  CHECK_NOTNULL(q_Ikp1_Ik);
  CHECK_GT(current_lidar_timestamp_ns, previous_lidar_timestamp_ns);

  switch (T_M_B_buffer_.getRotationBetween(
      current_lidar_timestamp_ns, previous_lidar_timestamp_ns, q_Ikp1_Ik)) {
    case vio_common::PoseLookupBuffer::ResultStatus::kFailedNotYetAvailable:
    // Fall through intended.
    case vio_common::PoseLookupBuffer::ResultStatus::kFailedWillNeverSucceed:
      return false;
    default:
      break;
  }
  return true;
}

void LidarTracking::visualizeTracking(
    const aslam::VisualNFrame::Ptr& nframe_k,
    const aslam::VisualNFrame::Ptr& nframe_kp1) {
  CHECK(nframe_k);
  CHECK(nframe_kp1);

  CHECK_EQ(nframe_k->getNumFrames(), nframe_kp1->getNumFrames());

  for (size_t frame_idx = 0u; frame_idx < 1; ++frame_idx) {
    aslam::FrameToFrameMatches matches_kp1_kp;
    aslam::extractMatchesFromTrackIdChannel(
        nframe_kp1->getFrame(frame_idx), nframe_k->getFrame(frame_idx),
        &matches_kp1_kp);

    cv::Mat output_image;
    aslam::drawVisualFrameKeyPointsAndMatches(
        nframe_kp1->getFrame(frame_idx), nframe_k->getFrame(frame_idx),
        aslam::FeatureVisualizationType::kHorizontal, matches_kp1_kp,
        &output_image);

    cv::imshow("Tracking", output_image);
    cv::waitKey(1);
  }
}

aslam::VisualNFrame::Ptr LidarTracking::getLastTrackedLidarNFrame() const {
  return previous_synced_meas_;
}

}  // namespace maplab
