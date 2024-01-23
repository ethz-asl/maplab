#ifndef ASLAM_FEATURE_TRACKER_BASE_H_
#define ASLAM_FEATURE_TRACKER_BASE_H_

#include <unordered_set>
#include <vector>

#include <aslam/cameras/camera.h>
#include <aslam/common/pose-types.h>
#include <aslam/matcher/match.h>
#include <Eigen/Dense>
#include <glog/logging.h>

namespace aslam {
class VisualFrame;
}

namespace aslam {
/// \class FeatureTracker
/// \brief Base class defining the interface for feature trackers.
class FeatureTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ASLAM_POINTER_TYPEDEFS(FeatureTracker);
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(FeatureTracker);

 protected:
  FeatureTracker() = default;
 public:
  virtual ~FeatureTracker() {}

  /// Track features and return the matches. The matches are not written to the TrackId channels
  /// and should be written to the track id channels using a TrackManager (after e.g. outlier
  /// filtering).
  virtual void track(
      const Quaternion& q_Ckp1_Ck, const VisualFrame& frame_k, aslam::VisualFrame* frame_kp1,
      FrameToFrameMatches* matches_with_score_kp1_k) = 0;

  /// Set a list of keypoint indices that should be aborted during the next call to track.
  /// The vector is swapped and invalidates the source vector.
  virtual void swapKeypointIndicesToAbort(const FrameId& /*frame_id*/,
      std::unordered_set<size_t>* /*keypoint_indices_to_abort*/) {
    LOG(FATAL) << "FeatureTracker does not support track abortion.";
  }
};

}  // namespace aslam

#endif  // ASLAM_FEATURE_TRACKER_BASE_H_
