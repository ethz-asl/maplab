#include "aslam/matcher/gyro-two-frame-matcher.h"

#include <aslam/common/statistics/statistics.h>
#include <glog/logging.h>

DEFINE_int32(gyro_matcher_small_search_distance_px, 10,
    "Small search rectangle size for keypoint matches.");
DEFINE_int32(gyro_matcher_large_search_distance_px, 20,
    "Large search rectangle size for keypoint matches."
    " Only used if small search was unsuccessful.");

namespace aslam {

GyroTwoFrameMatcher::GyroTwoFrameMatcher(
    const Quaternion& q_Ckp1_Ck,
    const VisualFrame& frame_kp1,
    const VisualFrame& frame_k,
    const int descriptor_type,
    const uint32_t image_height,
    const Eigen::Matrix2Xd& predicted_keypoint_positions_kp1,
    const std::vector<unsigned char>& prediction_success,
    FrameToFrameMatches* matches_kp1_k)
  : frame_kp1_(frame_kp1), frame_k_(frame_k), q_Ckp1_Ck_(q_Ckp1_Ck),
    descriptor_type_(descriptor_type),
    predicted_keypoint_positions_kp1_(predicted_keypoint_positions_kp1),
    prediction_success_(prediction_success),
    kDescriptorSizeBytes(frame_kp1.getDescriptorTypeSizeBytes(descriptor_type)),
    kNumPointsKp1(frame_kp1.getNumKeypointMeasurementsOfType(descriptor_type)),
    kNumPointsK(frame_k.getNumKeypointMeasurementsOfType(descriptor_type)),
    kImageHeight(image_height),
    matches_kp1_k_(matches_kp1_k),
    is_keypoint_kp1_matched_(kNumPointsKp1, false),
    iteration_processed_keypoints_kp1_(kNumPointsKp1, false),
    small_search_distance_px_(FLAGS_gyro_matcher_small_search_distance_px),
    large_search_distance_px_(FLAGS_gyro_matcher_large_search_distance_px) {
  CHECK(frame_kp1.isValid());
  CHECK(frame_k.isValid());
  CHECK(frame_kp1.hasDescriptors());
  CHECK(frame_k.hasDescriptors());
  CHECK(frame_kp1.hasKeypointMeasurements());
  CHECK(frame_k.hasKeypointMeasurements());
  CHECK_GT(frame_kp1.getTimestampNanoseconds(), frame_k.getTimestampNanoseconds());
  CHECK_NOTNULL(matches_kp1_k_)->clear();
  CHECK_EQ(kNumPointsKp1, frame_kp1.getDescriptorsOfType(descriptor_type).cols()) <<
      "Number of keypoints and descriptors in frame k+1 is not the same.";
  CHECK_EQ(kNumPointsK, frame_k.getDescriptorsOfType(descriptor_type).cols()) <<
      "Number of keypoints and descriptors in frame k is not the same.";
  CHECK_LE(kDescriptorSizeBytes*8, 512u) << "Usually binary descriptors' size "
      "is less or equal to 512 bits. Adapt the following check if this "
      "framework uses larger binary descriptors.";
  CHECK_GT(kImageHeight, 0u);
  CHECK_EQ(static_cast<int>(iteration_processed_keypoints_kp1_.size()), kNumPointsKp1);
  CHECK_EQ(static_cast<int>(is_keypoint_kp1_matched_.size()), kNumPointsKp1);
  CHECK_EQ(static_cast<int>(prediction_success_.size()), predicted_keypoint_positions_kp1_.cols());
  CHECK_GT(small_search_distance_px_, 0);
  CHECK_GT(large_search_distance_px_, 0);
  CHECK_GE(large_search_distance_px_, small_search_distance_px_);

  descriptors_kp1_wrapped_.reserve(kNumPointsKp1);
  keypoints_kp1_sorted_by_y_.reserve(kNumPointsKp1);
  descriptors_k_wrapped_.reserve(kNumPointsK);
  matches_kp1_k_->reserve(kNumPointsK);
  corner_row_LUT_.reserve(kImageHeight);
}

void GyroTwoFrameMatcher::initialize() {
  // Prepare descriptors for efficient matching.
  const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descriptors_kp1 =
      frame_kp1_.getDescriptorsOfType(descriptor_type_);
  const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descriptors_k =
      frame_k_.getDescriptorsOfType(descriptor_type_);

  for (int descriptor_kp1_idx = 0; descriptor_kp1_idx < kNumPointsKp1;
      ++descriptor_kp1_idx) {
    descriptors_kp1_wrapped_.emplace_back(
        &(descriptors_kp1.coeffRef(0, descriptor_kp1_idx)), kDescriptorSizeBytes);
  }

  for (int descriptor_k_idx = 0; descriptor_k_idx < kNumPointsK;
      ++descriptor_k_idx) {
    descriptors_k_wrapped_.emplace_back(
        &(descriptors_k.coeffRef(0, descriptor_k_idx)), kDescriptorSizeBytes);
  }

  // Sort keypoints of frame (k+1) from small to large y coordinates.
  const Eigen::Block<const Eigen::Matrix2Xd> keypoints_kp1 =
      frame_kp1_.getKeypointMeasurementsOfType(descriptor_type_);
  CHECK_EQ(keypoints_kp1.cols(), kNumPointsKp1);
  for (int i = 0; i < kNumPointsKp1; ++i) {
    keypoints_kp1_sorted_by_y_.emplace_back(keypoints_kp1.block<2, 1>(0, i), i);
  }

  std::sort(keypoints_kp1_sorted_by_y_.begin(), keypoints_kp1_sorted_by_y_.end(),
            [](const KeypointData& lhs, const KeypointData& rhs)-> bool {
              return lhs.measurement(1) < rhs.measurement(1);
            });

  // Lookup table construction.
  // TODO(magehrig):  Sort by y if image height >= image width,
  //                  otherwise sort by x.
  int v = 0;
  for (size_t y = 0u; y < kImageHeight; ++y) {
    while (v < kNumPointsKp1 &&
        y > static_cast<size_t>(keypoints_kp1_sorted_by_y_[v].measurement(1))) {
      ++v;
    }
    corner_row_LUT_.push_back(v);
  }
  CHECK_EQ(corner_row_LUT_.size(), kImageHeight);
}

void GyroTwoFrameMatcher::match() {
  initialize();

  if (kNumPointsK == 0 || kNumPointsKp1 == 0) {
    return;
  }

  for (int i = 0; i < kNumPointsK; ++i) {
    matchKeypoint(i);
  }

  std::vector<bool> is_inferior_keypoint_kp1_matched(
      is_keypoint_kp1_matched_);
  for (size_t i = 0u; i < kMaxNumInferiorIterations; ++i) {
    if(!matchInferiorMatches(&is_inferior_keypoint_kp1_matched)) return;
  }
}

void GyroTwoFrameMatcher::matchKeypoint(const int idx_k) {
  if (!prediction_success_[idx_k]) {
    return;
  }

  std::fill(iteration_processed_keypoints_kp1_.begin(),
            iteration_processed_keypoints_kp1_.end(),
            false);

  bool found = false;
  bool passed_ratio_test = false;
  int n_processed_corners = 0;
  KeyPointIterator it_best;
  const static unsigned int kDescriptorSizeBits = 8 * kDescriptorSizeBytes;
  int best_score = static_cast<int>(
      kDescriptorSizeBits * kMatchingThresholdBitsRatioRelaxed);
  unsigned int distance_best = kDescriptorSizeBits + 1;
  unsigned int distance_second_best = kDescriptorSizeBits + 1;
  const common::FeatureDescriptorConstRef& descriptor_k =
      descriptors_k_wrapped_[idx_k];

  Eigen::Vector2d predicted_keypoint_position_kp1 =
      predicted_keypoint_positions_kp1_.block<2, 1>(0, idx_k);
  KeyPointIterator nearest_corners_begin, nearest_corners_end;
  getKeypointIteratorsInWindow(
      predicted_keypoint_position_kp1, small_search_distance_px_, &nearest_corners_begin, &nearest_corners_end);

  const int bound_left_nearest =
      predicted_keypoint_position_kp1(0) - small_search_distance_px_;
  const int bound_right_nearest =
      predicted_keypoint_position_kp1(0) + small_search_distance_px_;

  MatchData current_match_data;

  // First search small window.
  for (KeyPointIterator it = nearest_corners_begin; it != nearest_corners_end; ++it) {
    if (it->measurement(0) < bound_left_nearest ||
        it->measurement(0) > bound_right_nearest) {
      continue;
    }

    CHECK_LT(it->channel_index, kNumPointsKp1);
    CHECK_GE(it->channel_index, 0);
    const common::FeatureDescriptorConstRef& descriptor_kp1 =
        descriptors_kp1_wrapped_[it->channel_index];
    unsigned int distance = common::GetNumBitsDifferent(descriptor_k, descriptor_kp1);
    int current_score = kDescriptorSizeBits - distance;
    if (current_score > best_score) {
      best_score = current_score;
      distance_second_best = distance_best;
      distance_best = distance;
      it_best = it;
      found = true;
    } else if (distance < distance_second_best) {
      // The second best distance can also belong
      // to two descriptors that do not qualify as match.
      distance_second_best = distance;
    }
    iteration_processed_keypoints_kp1_[it->channel_index] = true;
    ++n_processed_corners;
    const double current_matching_score =
        computeMatchingScore(current_score, kDescriptorSizeBits);
    current_match_data.addCandidate(it, current_matching_score);
  }

  // If no match in small window, increase window and search again.
  if (!found) {
    const int bound_left_near =
        predicted_keypoint_position_kp1(0) - large_search_distance_px_;
    const int bound_right_near =
        predicted_keypoint_position_kp1(0) + large_search_distance_px_;

    KeyPointIterator near_corners_begin, near_corners_end;
    getKeypointIteratorsInWindow(
        predicted_keypoint_position_kp1, large_search_distance_px_, &near_corners_begin, &near_corners_end);

    for (KeyPointIterator it = near_corners_begin; it != near_corners_end; ++it) {
      if (iteration_processed_keypoints_kp1_[it->channel_index]) {
        continue;
      }
      if (it->measurement(0) < bound_left_near ||
          it->measurement(0) > bound_right_near) {
        continue;
      }
      CHECK_LT(it->channel_index, kNumPointsKp1);
      CHECK_GE(it->channel_index, 0);
      const common::FeatureDescriptorConstRef& descriptor_kp1 =
          descriptors_kp1_wrapped_[it->channel_index];
      unsigned int distance =
          common::GetNumBitsDifferent(descriptor_k, descriptor_kp1);
      int current_score = kDescriptorSizeBits - distance;
      if (current_score > best_score) {
        best_score = current_score;
        distance_second_best = distance_best;
        distance_best = distance;
        it_best = it;
        found = true;
      } else if (distance < distance_second_best) {
        // The second best distance can also belong
        // to two descriptors that do not qualify as match.
        distance_second_best = distance;
      }
      ++n_processed_corners;
      const double current_matching_score =
          computeMatchingScore(current_score, kDescriptorSizeBits);
      current_match_data.addCandidate(it, current_matching_score);
    }
  }

  if (found) {
    passed_ratio_test = ratioTest(kDescriptorSizeBits, distance_best,
                                  distance_second_best);
  }

  if (passed_ratio_test) {
    CHECK(idx_k_to_attempted_match_data_map_.insert(
        std::make_pair(idx_k, current_match_data)).second);
    const int best_match_keypoint_idx_kp1 = it_best->channel_index;
    const double matching_score = computeMatchingScore(
        best_score, kDescriptorSizeBits);
    if (is_keypoint_kp1_matched_[best_match_keypoint_idx_kp1]) {
      if (matching_score > kp1_idx_to_matches_score_map_
          [best_match_keypoint_idx_kp1]) {
        // The current match is better than a previous match associated with the
        // current keypoint of frame (k+1). Hence, the inferior match is the
        // previous match associated with the current keypoint of frame (k+1).
        const int inferior_keypoint_idx_k =
            kp1_idx_to_matches_iterator_map_
            [best_match_keypoint_idx_kp1]->getKeypointIndexInFrameB();
        inferior_match_keypoint_idx_k_.push_back(inferior_keypoint_idx_k);

        kp1_idx_to_matches_score_map_
        [best_match_keypoint_idx_kp1] = matching_score;
        kp1_idx_to_matches_iterator_map_
        [best_match_keypoint_idx_kp1]->setKeypointIndexInFrameA(best_match_keypoint_idx_kp1);
        kp1_idx_to_matches_iterator_map_
        [best_match_keypoint_idx_kp1]->setKeypointIndexInFrameB(idx_k);
      } else {
        // The current match is inferior to a previous match associated with the
        // current keypoint of frame (k+1).
        inferior_match_keypoint_idx_k_.push_back(idx_k);
      }
    } else {
      is_keypoint_kp1_matched_[best_match_keypoint_idx_kp1] = true;
      matches_kp1_k_->emplace_back(best_match_keypoint_idx_kp1, idx_k);

      CHECK(kp1_idx_to_matches_iterator_map_.emplace(
          best_match_keypoint_idx_kp1, matches_kp1_k_->end() - 1).second);
      CHECK(kp1_idx_to_matches_score_map_.emplace(
          best_match_keypoint_idx_kp1, matching_score).second);
    }

    statistics::StatsCollector stats_distance_match(
        "GyroTracker: number of matching bits");
    stats_distance_match.AddSample(best_score);
  }
  statistics::StatsCollector stats_count_processed(
      "GyroTracker: number of computed distances per keypoint");
  stats_count_processed.AddSample(n_processed_corners);
}

bool GyroTwoFrameMatcher::matchInferiorMatches(
    std::vector<bool>* is_inferior_keypoint_kp1_matched) {
  CHECK_NOTNULL(is_inferior_keypoint_kp1_matched);
  CHECK_EQ(is_inferior_keypoint_kp1_matched->size(), is_keypoint_kp1_matched_.size());

  bool found_inferior_match = false;

  std::unordered_set<int> erase_inferior_match_keypoint_idx_k;
  for (const int inferior_keypoint_idx_k : inferior_match_keypoint_idx_k_) {
    const MatchData& match_data =
        idx_k_to_attempted_match_data_map_[inferior_keypoint_idx_k];
    bool found = false;
    double best_matching_score = static_cast<double>(kMatchingThresholdBitsRatioStrict);
    KeyPointIterator it_best;

    for (size_t i = 0u; i < match_data.keypoint_match_candidates_kp1.size(); ++i) {
      const KeyPointIterator& keypoint_kp1 = match_data.keypoint_match_candidates_kp1[i];
      const double matching_score = match_data.match_candidate_matching_scores[i];
      // Make sure that we don't try to match with already matched keypoints
      // of frame (k+1) (also previous inferior matches).
      if (is_keypoint_kp1_matched_[keypoint_kp1->channel_index]) continue;
      if (matching_score > best_matching_score) {
        it_best = keypoint_kp1;
        best_matching_score = matching_score;
        found = true;
      }
    }

    if (found) {
      found_inferior_match = true;
      const int best_match_keypoint_idx_kp1 = it_best->channel_index;
      if ((*is_inferior_keypoint_kp1_matched)[best_match_keypoint_idx_kp1]) {
        if (best_matching_score > kp1_idx_to_matches_score_map_
            [best_match_keypoint_idx_kp1]) {
          // The current match is better than a previous match associated with the
          // current keypoint of frame (k+1). Hence, the revoked match is the
          // previous match associated with the current keypoint of frame (k+1).
          const int revoked_inferior_keypoint_idx_k =
              kp1_idx_to_matches_iterator_map_
              [best_match_keypoint_idx_kp1]->getKeypointIndexInFrameB();
          // The current keypoint k does not have to be matched anymore
          // in the next iteration.
          erase_inferior_match_keypoint_idx_k.insert(inferior_keypoint_idx_k);
          // The keypoint k that was revoked. That means that it can be matched
          // again in the next iteration.
          erase_inferior_match_keypoint_idx_k.erase(revoked_inferior_keypoint_idx_k);

          kp1_idx_to_matches_score_map_
          [best_match_keypoint_idx_kp1] = best_matching_score;
          kp1_idx_to_matches_iterator_map_
          [best_match_keypoint_idx_kp1]->setKeypointIndexInFrameA(best_match_keypoint_idx_kp1);
          kp1_idx_to_matches_iterator_map_
          [best_match_keypoint_idx_kp1]->setKeypointIndexInFrameB(inferior_keypoint_idx_k);
        }
      } else {
        (*is_inferior_keypoint_kp1_matched)[best_match_keypoint_idx_kp1] = true;
        matches_kp1_k_->emplace_back(
            best_match_keypoint_idx_kp1, inferior_keypoint_idx_k);
        erase_inferior_match_keypoint_idx_k.insert(inferior_keypoint_idx_k);

        CHECK(kp1_idx_to_matches_iterator_map_.emplace(
            best_match_keypoint_idx_kp1, matches_kp1_k_->end() - 1).second);
        CHECK(kp1_idx_to_matches_score_map_.emplace(
            best_match_keypoint_idx_kp1, best_matching_score).second);
      }
    }
  }

  if (erase_inferior_match_keypoint_idx_k.size() > 0u) {
    // Do not iterate again over newly matched keypoints of frame k.
    // Hence, remove the matched keypoints.
    std::vector<int>::iterator iter_erase_from = std::remove_if(
        inferior_match_keypoint_idx_k_.begin(), inferior_match_keypoint_idx_k_.end(),
        [&erase_inferior_match_keypoint_idx_k](const int element) -> bool {
          return erase_inferior_match_keypoint_idx_k.count(element) == 1u;
        }
    );
    inferior_match_keypoint_idx_k_.erase(
        iter_erase_from, inferior_match_keypoint_idx_k_.end());
  }

  // Subsequent iterations should not mess with the current matches.
  is_keypoint_kp1_matched_ = *is_inferior_keypoint_kp1_matched;

  return found_inferior_match;
}


} // namespace aslam
