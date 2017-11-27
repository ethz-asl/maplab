#ifndef FEATURE_TRACKING_VO_FEATURE_TRACKING_PIPELINE_H_
#define FEATURE_TRACKING_VO_FEATURE_TRACKING_PIPELINE_H_

#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/thread-pool.h>
#include <aslam/frames/feature-track.h>
#include <aslam/tracker/track-manager.h>
#include <maplab-common/macros.h>
#include <posegraph/unique-id.h>

#include "feature-tracking/feature-tracking-pipeline.h"

namespace feature_tracking {

class VOFeatureTrackingPipeline : public FeatureTrackingPipeline {
 public:
  MAPLAB_POINTER_TYPEDEFS(VOFeatureTrackingPipeline);
  MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(VOFeatureTrackingPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VOFeatureTrackingPipeline();
  explicit VOFeatureTrackingPipeline(const aslam::NCamera::ConstPtr& ncamera);
  virtual ~VOFeatureTrackingPipeline();

  void trackFeaturesNFrame(
      const aslam::Quaternion& q_Bkp1_Bk, aslam::VisualNFrame* nframe_kp1,
      aslam::VisualNFrame* nframe_k,
      aslam::FrameToFrameMatchesList* inlier_matches_kp1_k,
      aslam::FrameToFrameMatchesList* outlier_matches_kp1_k);

 private:
  virtual void initialize(const aslam::NCamera::ConstPtr& ncamera) override;
  virtual void trackFeaturesNFrame(
      const aslam::Transformation& T_Bk_Bkp1, aslam::VisualNFrame* nframe_k,
      aslam::VisualNFrame* nframe_kp1) override;

  void trackFeaturesSingleCamera(
      const aslam::Quaternion& q_Bkp1_Bk, const size_t camera_idx,
      aslam::VisualFrame* frame_kp1, aslam::VisualFrame* frame_k,
      aslam::FrameToFrameMatches* inlier_matches_kp1_k,
      aslam::FrameToFrameMatches* outlier_matches_kp1_k);

  aslam::NCamera::ConstPtr ncamera_;
  /// Keypoint detector and descriptor extractors that detect keypoints in each
  /// frame and compute a descriptor for each one of them.
  std::vector<std::unique_ptr<FeatureDetectorExtractor>> detectors_extractors_;
  /// The actual frame-to-frame feature trackers that return a list of keypoint
  /// matches.
  std::vector<std::unique_ptr<aslam::FeatureTracker>> trackers_;
  /// Track managers to initialize new tracks based on the matches and write
  /// them to the track id
  /// channel. (one per camera).
  std::vector<std::unique_ptr<aslam::TrackManager>> track_managers_;
  /// Thread pool for tracking and track extraction.
  std::unique_ptr<aslam::ThreadPool> thread_pool_;

  bool has_feature_extraction_been_performed_on_first_nframe_;
};
}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_VO_FEATURE_TRACKING_PIPELINE_H_
