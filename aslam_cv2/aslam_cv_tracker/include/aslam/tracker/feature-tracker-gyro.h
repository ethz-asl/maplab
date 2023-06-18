#ifndef ASLAM_GYRO_TRACKER_H_
#define ASLAM_GYRO_TRACKER_H_

#include <array>
#include <deque>
#include <memory>
#include <vector>

#include <aslam/common/macros.h>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/features2d/features2d.hpp>

#include "aslam/tracker/feature-tracker.h"

namespace aslam {
class VisualFrame;
class Camera;

struct GyroTrackerSettings {
  GyroTrackerSettings();

  double lk_max_num_candidates_ratio_kp1;
  size_t lk_max_status_track_length;

  // calcOpticalFlowPyrLK parameters:
  cv::TermCriteria lk_termination_criteria;
  cv::Size lk_window_size;
  int lk_max_pyramid_levels;
  int lk_operation_flag;
  double lk_min_eigenvalue_threshold;

  // Keypoint uncertainty.
  static constexpr double kKeypointUncertaintyPx = 0.8;
};


/// \class GyroTracker
/// \brief Feature tracker using an interframe rotation matrix to predict the feature positions
///        while matching. It also tracks a subset of unmatched features with an optical
///        flow algorithm (Lucas-Kanade method).
class GyroTracker : public FeatureTracker{
 public:
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(GyroTracker);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /// \brief Construct the feature tracker.
  /// @param[in] camera The camera used in the tracker for projection/backprojection.
  /// @param[in] min_distance_to_image_border The distance to the image border
  ///                                         that must remain free of keypoints.
  /// @param[in] extractor_ptr Pointer to an extractor objects that is used to
  ///                          compute descriptors for optical flow tracked keypoints.
  explicit GyroTracker(const Camera& camera,
                       const size_t min_distance_to_image_border,
                       const cv::Ptr<cv::DescriptorExtractor>& extractor_ptr,
                       int descriptor_type = 0);
  virtual ~GyroTracker() {}

  /// \brief Track features between the current and the previous frames using a given interframe
  ///        rotation q_Ckp1_Ck to predict the feature positions.
  /// @param[in] q_Ckp1_Ck      Rotation matrix that describes the camera rotation between the
  ///                           two frames that are matched.
  /// @param[int] frame_k       The previous VisualFrame that needs to contain the keypoints and
  ///                           descriptor channels. Usually this is an output of the VisualPipeline.
  /// @param[out] frame_kp1     The current VisualFrame that needs to contain the keypoints and
  ///                           descriptor channels. Usually this is an output of the VisualPipeline.
  /// @param[out] matches_kp1_k  Vector of structs containing the found matches. Indices
  ///                            correspond to the ordering of the keypoint/descriptor vector in the
  ///                            respective frame channels.
  virtual void track(const Quaternion& q_Ckp1_Ck,
                     const VisualFrame& frame_k,
                     VisualFrame* frame_kp1,
                     FrameToFrameMatches* matches_kp1_k) override;

 private:
  enum class FeatureStatus {
    kDetected,
    kLkTracked
  };

  // first: index_k, second: index_km1.
  typedef std::pair<int, int> TrackedMatch;
  typedef std::vector<FeatureStatus> FrameFeatureStatus;
  typedef std::vector<size_t> FrameStatusTrackLength;
  typedef Eigen::VectorXi TrackIds;

  /// Track candidate features from frame k to (k+1) with optical flow.
  /// Extract descriptors for successful tracks and insert keypoint
  /// and descriptor information into frame (k+1).
  virtual void lkTracking(
      const Eigen::Matrix2Xd& predicted_keypoint_positions_kp1,
      const std::vector<unsigned char>& prediction_success,
      const std::vector<int>& lk_candidate_indices_k,
      const VisualFrame& frame_k,
      VisualFrame* frame_kp1,
      FrameToFrameMatches* matches_kp1_k);

  /// In general, not all unmatched features will be tracked with the optical
  /// flow algorithm. This function computes the candidates that will be tracked.
  virtual void computeLKCandidates(
      const FrameToFrameMatches& matches_kp1_k,
      const FrameStatusTrackLength& status_track_length_k,
      const VisualFrame& frame_k,
      const VisualFrame& frame_kp1,
      std::vector<int>* lk_candidate_indices_k) const;

  /// Compute matches from frame k and frame (k-1). They are called tracked
  /// matches since not all original matches get tracked (e.g. rejected by RANSAC).
  virtual void computeTrackedMatches(
      std::vector<TrackedMatch>* tracked_matches) const;

  /// The matcher is not able to match all features from frame k to (k+1).
  /// This function computes the indices of unmatched features in frame k.
  virtual void computeUnmatchedIndicesOfFrameK(
      const FrameToFrameMatches& matches_kp1_k,
      std::vector<int>* unmatched_indices_k) const;

  /// Status track length is defined as the track length since the status
  /// (lk-tracked or detected) of the tracked feature has changed.
  virtual void computeStatusTrackLengthOfFrameK(
      const std::vector<TrackedMatch>& tracked_matches,
      FrameStatusTrackLength* status_track_length_k);

  virtual void initializeFeatureStatusDeque();

  virtual void updateFeatureStatusDeque(
      const FrameFeatureStatus& frame_feature_status_kp1);

  virtual void updateTrackIdDeque(
      const VisualFrame& new_frame_k);

  /// Erase elements of a vector based on a set of indices.
  template <typename Type>
  void eraseVectorElementsByIndex(
          const std::unordered_set<size_t>& indices_to_erase,
          std::vector<Type>* vec) const;

  /// The camera model used in the tracker.
  const aslam::Camera& camera_;
  /// Minimum distance to image border is used to skip image points,
  /// predicted by the lk-tracker, that are too close to the image border.
  const size_t kMinDistanceToImageBorderPx;
  /// Descriptor extractor that is used on lk-tracked points.
  const cv::Ptr<cv::DescriptorExtractor> extractor_;
  /// Remember if we have initialized already.
  bool initialized_;
  // Store track IDs of frame k and (k-1) in that order.
  std::deque<TrackIds> track_ids_k_km1_;
  /// Keep feature status for every index. For frames k and km1 in that order.
  std::deque<FrameFeatureStatus> feature_status_k_km1_;
  /// Keep status track length of frame (k-1) for every index.
  /// Status track length refers to the track length
  /// since the status of the feature has changed.
  FrameStatusTrackLength status_track_length_km1_;
  /// Feature type to track, used to support multiple feature types in
  /// frames. The returned matches will be with respect to the indices
  /// of the descriptor type. To obtain the index offset with respect to the
  /// entire keypoint block use VisualFrame::getDescriptorBlockTypeStartAndSize
  int descriptor_type_;

  const GyroTrackerSettings settings_;
};

template <typename Type>
void GyroTracker::eraseVectorElementsByIndex(
        const std::unordered_set<size_t>& indices_to_erase,
        std::vector<Type>* vec) const {
  CHECK_NOTNULL(vec);
    std::vector<bool> erase_index(vec->size(), false);
    for (const size_t i: indices_to_erase) {
        erase_index[i] = true;
    }
    std::vector<bool>::const_iterator it_to_erase = erase_index.begin();
    typename std::vector<Type>::iterator it_erase_from = std::remove_if(
        vec->begin(), vec->end(),
        [&it_to_erase](const Type& /*whatever*/) -> bool {
          return *it_to_erase++ == true;
        }
    );
    vec->erase(it_erase_from, vec->end());
    vec->shrink_to_fit();
}

} // namespace aslam

#endif  // ASLAM_GYRO_TRACKER_H_
