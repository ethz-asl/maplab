#ifndef FEATURE_TRACKING_VO_OUTLIER_REJECTION_PIPELINE_H_
#define FEATURE_TRACKING_VO_OUTLIER_REJECTION_PIPELINE_H_

#include <sensors/external-features.h>
#include <unordered_set>

#include "feature-tracking/feature-tracking-pipeline.h"
#include "feature-tracking/feature-tracking-types.h"

namespace feature_tracking {

class VOOutlierRejectionPipeline : public FeatureTrackingPipeline {
 public:
  MAPLAB_POINTER_TYPEDEFS(VOOutlierRejectionPipeline);
  MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(VOOutlierRejectionPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VOOutlierRejectionPipeline(
      const aslam::Camera::ConstPtr& camera, const int cam_idx,
      const aslam::Quaternion& q_C_B, const vi_map::FeatureType feature_type,
      const FeatureTrackingOutlierSettings& outlier_settings);
  virtual ~VOOutlierRejectionPipeline();

  void rejectMatchesFrame(
      const aslam::Quaternion& q_Bkp1_Bk, aslam::VisualFrame* frame_kp1,
      aslam::VisualFrame* frame_k);

  void reset();

 private:
  aslam::Camera::ConstPtr camera_;
  const int cam_idx_;
  const aslam::Quaternion q_C_B_;
  const int feature_type_;
  const std::string feature_type_string_;

  // Used to check that the frames being given are connected, otherwise
  // we have to reset the outlier and inlier information we maintain.
  bool initialized_;
  int64_t k_frame_timestamp_;

  // We remember which tracks were marked as outliers in the previous frame
  // so we can continue eliminating them. This is necessary because we can't
  // communicate with the external feature tracker on what was discarded.
  // If a track stops appearing in a frame we no longer need to remember it
  // as an outlier, since it means the tracker has stopped tracking it. This
  // assumes matches are always only between two consecutive frames, but saves
  // a lot of memory and computation at runtime.
  std::unordered_set<int> k_outlier_track_ids;

  // We remember which tracks were valid in the previous frame. This is so
  // that if we remove a current outlier match, we know if the observation
  // in the previous frame has a track length of one and can be deleted.
  std::unordered_set<int> k_valid_track_ids_;

  const FeatureTrackingOutlierSettings outlier_settings_;
};

}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_VO_OUTLIER_REJECTION_PIPELINE_H_
