#ifndef MATCHER_GYRO_TWO_FRAME_MATCHER_H_
#define MATCHER_GYRO_TWO_FRAME_MATCHER_H_

#include <algorithm>
#include <vector>

#include <aslam/common/pose-types.h>
#include <aslam/common/feature-descriptor-ref.h>
#include <aslam/frames/visual-frame.h>
#include <Eigen/Core>
#include <glog/logging.h>

#include "aslam/matcher/match.h"

namespace aslam {

/// \class GyroTwoFrameMatcher
/// \brief Frame to frame matcher using an interframe rotation matrix
///  to predict the feature positions to constrain the search window.
///
/// The initial matcher attempts to match every keypoint of frame k to a keypoint
/// in frame (k+1). This is done by predicting the keypoint location by
/// using an interframe rotation matrix. Then a rectangular search window around
/// that location is searched for the best match greater than a threshold.
/// If the initial search was not successful, the search window is increased once.
/// The initial matcher is allowed to discard a previous match if the new one
/// has a higher score. The discarded matches are called inferior matches and
/// a second matcher tries to match them. The second matcher only tries
/// to match a keypoint of frame k with the queried keypoints of frame (k+1)
/// of the initial matcher. Therefore, it does not compute distances between
/// descriptors anymore because the initial matcher has already done that.
/// The second matcher is executed several times because it is also allowed
/// to discard inferior matches of the current iteration.
/// The matches are exclusive.
class GyroTwoFrameMatcher {
 public:
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(GyroTwoFrameMatcher);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Constructs the GyroTwoFrameMatcher.
  /// @param[in]  q_Ckp1_Ck     Rotation matrix that describes the camera rotation between the
  ///                           two frames that are matched.
  /// @param[in]  frame_kp1     The current VisualFrame that needs to contain the keypoints and
  ///                           descriptor channels. Usually this is an output of the VisualPipeline.
  /// @param[in]  frame_k       The previous VisualFrame that needs to contain the keypoints and
  ///                           descriptor channels. Usually this is an output of the VisualPipeline.
  /// @param[in]  image_height  The image height of the given camera.
  /// @param[in]  predicted_keypoint_positions_kp1  Predicted positions of keypoints in next frame.
  /// @param[in]  prediction_success  Was the prediction successful?
  /// @param[out] matches_kp1_k  Vector of structs containing the found matches. Indices
  ///                            correspond to the ordering of the keypoint/descriptor vector in the
  ///                            respective frame channels.
  GyroTwoFrameMatcher(const Quaternion& q_Ckp1_Ck,
                      const VisualFrame& frame_kp1,
                      const VisualFrame& frame_k,
                      const int descriptor_type,
                      const uint32_t image_height,
                      const Eigen::Matrix2Xd& predicted_keypoint_positions_kp1,
                      const std::vector<unsigned char>& prediction_success,
                      FrameToFrameMatches* matches_kp1_k);
  virtual ~GyroTwoFrameMatcher() {};

  void match();

 private:
  struct KeypointData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeypointData(const Eigen::Vector2d& measurement, const int index)
      : measurement(measurement), channel_index(index) {}
    Eigen::Vector2d measurement;
    int channel_index;
  };

  typedef typename Aligned<std::vector, KeypointData>::const_iterator KeyPointIterator;
  typedef typename FrameToFrameMatches::iterator MatchesIterator;

  struct MatchData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MatchData() = default;
    void addCandidate(
        const KeyPointIterator keypoint_iterator_kp1, const double matching_score) {
      CHECK_GT(matching_score, 0.0);
      CHECK_LE(matching_score, 1.0);
      keypoint_match_candidates_kp1.push_back(keypoint_iterator_kp1);
      match_candidate_matching_scores.push_back(matching_score);
    }
    // Iterators of keypoints of frame (k+1) that were candidates for the match
    // together with their scores.
    std::vector<KeyPointIterator> keypoint_match_candidates_kp1;
    std::vector<double> match_candidate_matching_scores;
  };

  /// \brief Initialize data the matcher relies on.
  void initialize();

  /// \brief Match a keypoint of frame k with one of frame (k+1) if possible.
  ///
  /// Initial matcher that tries to match a keypoint of frame k with
  /// a keypoint of frame (k+1) once. It is allowed to discard an
  /// already existing match.
  void matchKeypoint(const int idx_k);

  void getKeypointIteratorsInWindow(
      const Eigen::Vector2d& predicted_keypoint_position,
      const int window_half_side_length_px,
      KeyPointIterator* it_keypoints_begin,
      KeyPointIterator* it_keypoints_end) const;

  /// \brief Try to match inferior matches without modifying initial matches.
  ///
  /// Second matcher that is only quering keypoints of frame (k+1) that the
  /// initial matcher has queried before. Should be executed several times.
  /// Returns true if matches are still found.
  bool matchInferiorMatches(std::vector<bool>* is_inferior_keypoint_kp1_matched);

  int clamp(const int lower, const int upper, const int in) const;

  // The larger the matching score (which is smaller or equal to 1),
  // the higher the probability that a true match occurred.
  double computeMatchingScore(const int num_matching_bits,
                              const unsigned int descriptor_size_bits) const;

  // Compute ratio test. Test is inspired by David Lowe's "ratio test"
  // for matching descriptors. Returns true if test is passed.
  bool ratioTest(const unsigned int descriptor_size_bits,
                 const unsigned int distance_shortest,
                 const unsigned int distance_second_shortest) const;

  // The current frame.
  const VisualFrame& frame_kp1_;
  // The previous frame.
  const VisualFrame& frame_k_;
  // Rotation matrix that describes the camera rotation between the
  // two frames that are matched.
  const Quaternion& q_Ckp1_Ck_;
  // Descriptor type to match, used to support multiple different types
  // Of descriptors to co-exist in the aslam visual frame
  const int descriptor_type_;
  // Predicted locations of the keypoints in frame k
  // in frame (k+1) based on camera rotation.
  const Eigen::Matrix2Xd& predicted_keypoint_positions_kp1_;
  // Store prediction success for each keypoint of
  // frame k.
  const std::vector<unsigned char>& prediction_success_;
  // Descriptor size in bytes.
  const size_t kDescriptorSizeBytes;
  // Number of keypoints/descriptors in frame (k+1).
  const int kNumPointsKp1;
  // Number of keypoints/descriptors in frame k.
  const int kNumPointsK;
  const uint32_t kImageHeight;

  // Matches with indices corresponding to the ordering of
  // the keypoint/descriptors in the respective channels.
  FrameToFrameMatches* const matches_kp1_k_;
  // Descriptors of frame (k+1).
  std::vector<common::FeatureDescriptorConstRef> descriptors_kp1_wrapped_;
  // Descriptors of frame k.
  std::vector<common::FeatureDescriptorConstRef> descriptors_k_wrapped_;
  // Keypoints of frame (k+1) sorted from small to large y coordinates.
  Aligned<std::vector, KeypointData> keypoints_kp1_sorted_by_y_;
  // corner_row_LUT[i] is the number of keypoints that has y position
  // lower than i in the image.
  std::vector<int> corner_row_LUT_;
  // Remember matched keypoints of frame (k+1).
  std::vector<bool> is_keypoint_kp1_matched_;
  // Map from keypoint indices of frame (k+1) to
  // the corresponding match iterator.
  std::unordered_map<int, MatchesIterator> kp1_idx_to_matches_iterator_map_;
  std::unordered_map<int, double> kp1_idx_to_matches_score_map_;
  // Keep track of processed keypoints s.t. we don't process them again in the
  // large window. Set every element to false for each keypoint (of frame k) iteration!
  std::vector<bool> iteration_processed_keypoints_kp1_;
  // The queried keypoints in frame (k+1) and the corresponding
  // matching score are stored for each attempted match.
  // A map from the keypoint in frame k to the corresponding
  // match data is created.
  std::unordered_map<int, MatchData> idx_k_to_attempted_match_data_map_;
  // Inferior matches are a subset of all attempted matches.
  // Remeber indices of keypoints in frame k that are deemed inferior matches.
  std::vector<int> inferior_match_keypoint_idx_k_;

  // Two descriptors could match if the number of matching bits normalized
  // with the descriptor length in bits is higher than this threshold.
  static constexpr float kMatchingThresholdBitsRatioRelaxed = 0.8f;
  // The more strict threshold is used for matching inferior matches.
  // It is more strict because there is no ratio test anymore.
  static constexpr float kMatchingThresholdBitsRatioStrict = 0.85f;
  // Two descriptors could match if they pass the Lowe ratio test.
  static constexpr float kLoweRatio = 0.8f;
  // Small image space distances for keypoint matches.
  const int small_search_distance_px_;
  // Large image space distances for keypoint matches.
  // Only used if small search was unsuccessful.
  const int large_search_distance_px_;
  // Number of iterations to match inferior matches.
  static constexpr size_t kMaxNumInferiorIterations = 3u;
};

void GyroTwoFrameMatcher::getKeypointIteratorsInWindow(
    const Eigen::Vector2d& predicted_keypoint_position,
    const int window_half_side_length_px,
    KeyPointIterator* it_keypoints_begin,
    KeyPointIterator* it_keypoints_end) const {
  CHECK_NOTNULL(it_keypoints_begin);
  CHECK_NOTNULL(it_keypoints_end);
CHECK_GT(window_half_side_length_px, 0);

  // Compute search area for LUT iterators row-wise.
  int LUT_index_top = clamp(0, kImageHeight - 1, static_cast<int>(
      predicted_keypoint_position(1) + 0.5 - window_half_side_length_px));
  int LUT_index_bottom = clamp(0, kImageHeight - 1, static_cast<int>(
      predicted_keypoint_position(1) + 0.5 + window_half_side_length_px));

  *it_keypoints_begin = keypoints_kp1_sorted_by_y_.begin() + corner_row_LUT_[LUT_index_top];
  *it_keypoints_end = keypoints_kp1_sorted_by_y_.begin() + corner_row_LUT_[LUT_index_bottom];

  CHECK_LE(LUT_index_top, LUT_index_bottom);
  CHECK_GE(LUT_index_bottom, 0);
  CHECK_GE(LUT_index_top, 0);
  CHECK_LT(LUT_index_top, static_cast<int>(kImageHeight));
  CHECK_LT(LUT_index_bottom, static_cast<int>(kImageHeight));
}

inline int GyroTwoFrameMatcher::clamp(
    const int lower, const int upper, const int in) const {
  return std::min<int>(std::max<int>(in, lower), upper);
}

inline double GyroTwoFrameMatcher::computeMatchingScore(
    const int num_matching_bits, const unsigned int descriptor_size_bits) const {
  return static_cast<double>(num_matching_bits)/descriptor_size_bits;
}

inline bool GyroTwoFrameMatcher::ratioTest(
    const unsigned int descriptor_size_bits,
    const unsigned int distance_closest,
    const unsigned int distance_second_closest) const {
  CHECK_LE(distance_closest, distance_second_closest);
  if (distance_second_closest > descriptor_size_bits) {
    // There has never been a second matching candidate.
    // Consequently, we cannot conclude with this test.
    return true;
  } else if (distance_second_closest == 0u) {
    // Unusual case of division by zero:
    // Let the ratio test be successful.
    return true;
  } else {
    return distance_closest/static_cast<float>(distance_second_closest) <
        kLoweRatio;
  }
}

} // namespace aslam

#endif // MATCHER_GYRO_TWO_FRAME_MATCHER_H_
