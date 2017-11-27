#include <array>

#include "feature-tracking/feature-detection-extraction.h"

#include <aslam/common/statistics/statistics.h>
#include <aslam/common/timer.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/tracker/tracking-helpers.h>
#include <brisk/brisk.h>
#include <gflags/gflags.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace feature_tracking {

FeatureDetectorExtractor::FeatureDetectorExtractor(const aslam::Camera& camera)
    : camera_(camera) {
  initialize();
}

void FeatureDetectorExtractor::initialize() {
  CHECK_LT(
      2 * detector_settings_.min_tracking_distance_to_image_border_px,
      camera_.imageWidth());
  CHECK_LT(
      2 * detector_settings_.min_tracking_distance_to_image_border_px,
      camera_.imageHeight());

  detector_ = cv::ORB::create(
      detector_settings_.orb_detector_number_features,
      detector_settings_.orb_detector_scale_factor,
      detector_settings_.orb_detector_pyramid_levels,
      detector_settings_.orb_detector_edge_threshold,
      detector_settings_.orb_detector_first_level,
      detector_settings_.orb_detector_WTA_K,
      detector_settings_.orb_detector_score_type,
      detector_settings_.orb_detector_patch_size,
      detector_settings_.orb_detector_fast_threshold);

  switch (extractor_settings_.descriptor_type) {
    case SweFeatureTrackingExtractorSettings::DescriptorType::kBrisk:
      extractor_ = new brisk::BriskDescriptorExtractor(
          extractor_settings_.rotation_invariant,
          extractor_settings_.scale_invariant);
      break;
    case SweFeatureTrackingExtractorSettings::DescriptorType::kOcvFreak:
      extractor_ = cv::xfeatures2d::FREAK::create(
          extractor_settings_.rotation_invariant,
          extractor_settings_.scale_invariant,
          extractor_settings_.freak_pattern_scale,
          detector_settings_.orb_detector_pyramid_levels);
      break;
    default:
      LOG(FATAL) << "Unknown descriptor type.";
      break;
  }
}

cv::Ptr<cv::DescriptorExtractor> FeatureDetectorExtractor::getExtractorPtr()
    const {
  return extractor_;
}

void FeatureDetectorExtractor::detectAndExtractFeatures(
    aslam::VisualFrame* frame) const {
  CHECK_NOTNULL(frame);
  CHECK(frame->hasRawImage())
      << "Can only detect keypoints if the frame has a raw image";
  CHECK_EQ(
      camera_.getId(),
      CHECK_NOTNULL(frame->getCameraGeometry().get())->getId());

  timing::Timer timer_detection("keypoint detection");

  std::vector<cv::KeyPoint> keypoints_cv;
  const cv::Mat& image = frame->getRawImage();

  detector_->detect(image, keypoints_cv, detector_mask_);
  if (detector_settings_.detector_use_nonmaxsuppression &&
      !keypoints_cv.empty()) {
    timing::Timer timer_nms("non-maximum suppression");
    localNonMaximumSuppression(
        detector_settings_.detector_nonmaxsuppression_radius,
        detector_settings_.detector_nonmaxsuppression_ratio_threshold,
        &keypoints_cv);
    statistics::StatsCollector stat_nms(
        "non-maximum suppression (1 image) in ms");
    stat_nms.AddSample(timer_nms.Stop() * 1000);
  }
  cv::KeyPointsFilter::retainBest(
      keypoints_cv, detector_settings_.max_feature_count);

  // The ORB detector tries to always return a constant number of keypoints.
  // If we get into an environment with very few good keypoint candidates
  // the detector adapts it's score such that it even detects keypoints that
  // are dust on the camera. Therefore, we put a lower bound on the
  // threshold.
  size_t num_removed_keypoints = keypoints_cv.size();

  std::vector<cv::KeyPoint>::iterator it_erase_from = std::remove_if(
      keypoints_cv.begin(), keypoints_cv.end(), [this](const cv::KeyPoint& kp) {
        return kp.response <= detector_settings_.orb_detector_score_lower_bound;
      });
  keypoints_cv.erase(it_erase_from, keypoints_cv.end());

  num_removed_keypoints -= keypoints_cv.size();
  VLOG(4) << "Number of removed low score keypoints: " << num_removed_keypoints;

  if (extractor_settings_.flip_descriptor) {
    for (cv::KeyPoint& keypoint : keypoints_cv) {
      keypoint.angle = 180.0f;
    }
  } else {
    // We are doing this because:
    // - Stefan Leutenegger's BRISK implementation uses orientation information
    //   (in case of enabled rotational invariance) of OpenCV keypoints if the
    //   angles are not -1 and the ORB detector assigns orientations to
    //   keypoints.
    //   I don't know which orientation assignment is more robust so feel free
    //   to remove this if you know better. I think there is no significant
    //   performance difference.
    // - Also remove this if you use descriptors (e.g. OpenCV BRISK or ORB)
    //   that rely on orientation information provided by some detectors
    //   (e.g. ORB detector).
    for (cv::KeyPoint& keypoint : keypoints_cv) {
      keypoint.angle = -1.0f;
    }
  }
  timer_detection.Stop();

  timing::Timer timer_extraction("descriptor extraction");

  // Compute the descriptors.
  cv::Mat descriptors_cv;
  if (!keypoints_cv.empty()) {
    extractor_->compute(frame->getRawImage(), keypoints_cv, descriptors_cv);
  } else {
    descriptors_cv = cv::Mat(0, 0, CV_8UC1);
  }

  timer_extraction.Stop();

  // Note: It is important that the values are set even if there are no
  // keypoints as downstream code may rely on the keypoints being set.
  aslam::insertCvKeypointsAndDescriptorsIntoEmptyVisualFrame(
      keypoints_cv, descriptors_cv, detector_settings_.keypoint_uncertainty_px,
      frame);
}

// TODO(magehrig): Local non-maximum suppression only on octave level for
//                 improved scale invariance.
void FeatureDetectorExtractor::localNonMaximumSuppression(
    const float radius, const float ratio_threshold,
    std::vector<cv::KeyPoint>* keypoints) const {
  CHECK_GT(radius, 0.0f);
  CHECK_GT(ratio_threshold, 0.0f);
  CHECK_LE(ratio_threshold, 1.0f);
  CHECK_GT(CHECK_NOTNULL(keypoints)->size(), 0);

  const float kRadiusSquared = radius * radius;
  const size_t kNumKeypoints = keypoints->size();
  const size_t kImageHeight = camera_.imageHeight();
  std::vector<bool> erase_keypoints(kNumKeypoints, false);

  struct KeyPointData {
    KeyPointData(const cv::KeyPoint& keypoint, const size_t _keypoint_index)
        : coordinates{keypoint.pt.x, keypoint.pt.y},  // NOLINT
          response(keypoint.response),
          keypoint_index(_keypoint_index) {}
    std::array<float, 2> coordinates;
    float response;
    size_t keypoint_index;
  };

  typedef std::vector<KeyPointData>::const_iterator KeyPointDataIterator;

  std::function<bool(const KeyPointData&, KeyPointDataIterator)>  // NOLINT
      IsInsideCircle = [kRadiusSquared](
          const KeyPointData& keypoint_1,
          KeyPointDataIterator keypoint_iterator_2) -> bool {
    const float x_diff =
        keypoint_1.coordinates[0] - keypoint_iterator_2->coordinates[0];
    const float y_diff =
        keypoint_1.coordinates[1] - keypoint_iterator_2->coordinates[1];
    return x_diff * x_diff + y_diff * y_diff < kRadiusSquared;
  };

  std::function<size_t(const size_t, const size_t, const size_t)> Clamp =
      [](  // NOLINT
          const size_t lower, const size_t upper, const size_t in) -> size_t {
    return std::min<size_t>(std::max<size_t>(in, lower), upper);
  };

  std::vector<KeyPointData> keypoint_data_vector;
  keypoint_data_vector.reserve(kNumKeypoints);
  for (size_t i = 0u; i < kNumKeypoints; ++i) {
    keypoint_data_vector.emplace_back((*keypoints)[i], i);
  }

  std::sort(
      keypoint_data_vector.begin(), keypoint_data_vector.end(),
      [](const KeyPointData& lhs, const KeyPointData& rhs) -> bool {
        return lhs.coordinates[1] < rhs.coordinates[1];
      });

  std::vector<size_t> corner_row_LUT_;
  corner_row_LUT_.reserve(kImageHeight);

  size_t num_kpts_below_y = 0u;
  for (size_t y = 0u; y < kImageHeight; ++y) {
    while (num_kpts_below_y < kNumKeypoints &&
           y > keypoint_data_vector[num_kpts_below_y].coordinates[1]) {
      ++num_kpts_below_y;
    }
    corner_row_LUT_.push_back(num_kpts_below_y);
  }

  for (size_t i = 0u; i < kNumKeypoints; ++i) {
    const KeyPointData& current_keypoint_data = keypoint_data_vector[i];

    size_t y_top = Clamp(
        0u, kImageHeight - 1u,
        static_cast<size_t>(
            current_keypoint_data.coordinates[1] + 0.5f - radius));
    size_t y_bottom = Clamp(
        0u, kImageHeight - 1u,
        static_cast<size_t>(
            current_keypoint_data.coordinates[1] + 0.5f + radius));

    KeyPointDataIterator nearest_corners_begin =
        keypoint_data_vector.begin() + corner_row_LUT_[y_top];
    KeyPointDataIterator nearest_corners_end =
        keypoint_data_vector.begin() + corner_row_LUT_[y_bottom];

    for (KeyPointDataIterator it = nearest_corners_begin;
         it != nearest_corners_end; ++it) {
      if (it->keypoint_index == current_keypoint_data.keypoint_index ||
          erase_keypoints[it->keypoint_index] ||
          !IsInsideCircle(current_keypoint_data, it)) {
        continue;
      }
      const float response_threshold =
          ratio_threshold * current_keypoint_data.response;
      if (response_threshold > it->response) {
        erase_keypoints[it->keypoint_index] = true;
      }
    }
  }

  std::vector<bool>::iterator it_erase = erase_keypoints.begin();

  std::vector<cv::KeyPoint>::iterator it_erase_from = std::remove_if(
      keypoints->begin(), keypoints->end(),
      [&it_erase](const cv::KeyPoint & /*keypoint*/) -> bool {
        return *it_erase++ == true;
      });
  keypoints->erase(it_erase_from, keypoints->end());
}

}  // namespace feature_tracking
