#ifndef FEATURE_TRACKING_FEATURE_TRACKING_TYPES_H_
#define FEATURE_TRACKING_FEATURE_TRACKING_TYPES_H_

#include <string>

namespace feature_tracking {

struct SimpleBriskFeatureTrackingSettings {
  SimpleBriskFeatureTrackingSettings();
  virtual ~SimpleBriskFeatureTrackingSettings() = default;
  const size_t num_octaves;
  const double uniformity_radius;
  const double absolute_threshold;
  const size_t max_number_of_keypoints;
  const bool rotation_invariant;
  const bool scale_invariant;
  const bool copy_images;
  const int matching_descriptor_hamming_distance_threshold;
  const double matching_image_space_distance_threshold_px;
};

struct SweFeatureTrackingExtractorSettings {
  enum class DescriptorType { kOcvFreak, kBrisk };
  SweFeatureTrackingExtractorSettings();
  DescriptorType convertStringToDescriptorType(
      const std::string& descriptor_string);
  /// Type of descriptor used by SWE.
  DescriptorType descriptor_type;

  /// Common settings of all descriptors.
  bool rotation_invariant;
  bool scale_invariant;
  bool flip_descriptor;

  /// FREAK settings.
  float freak_pattern_scale;
};

struct SweFeatureTrackingDetectorSettings {
  SweFeatureTrackingDetectorSettings();

  // Settings for the non-maximum suppression algorithm.
  bool detector_use_nonmaxsuppression;
  float detector_nonmaxsuppression_radius;
  float detector_nonmaxsuppression_ratio_threshold;

  // ORB detector settings.
  // An adaption of the FAST detector designed for the ORB descriptor.
  // Suitable for real-time applications.
  int orb_detector_number_features;
  float orb_detector_scale_factor;
  int orb_detector_pyramid_levels;
  int orb_detector_edge_threshold;
  // It should be 0 according to the OpenCV documentation.
  int orb_detector_first_level;
  // The number of points that produce each element of the oriented
  // BRIEF descriptor. Check OCV documentation for more information.
  int orb_detector_WTA_K;
  // The default HARRIS_SCORE means that Harris algorithm is used to rank
  // features. FAST_SCORE is alternative value of the parameter that produces
  // slightly less stable keypoints, but it is a little faster to compute.
  int orb_detector_score_type;
  // Size of the patch used by the oriented BRIEF descriptor. Of course, on
  // smaller pyramid layers the perceived image area covered by a feature will
  // be larger.
  int orb_detector_patch_size;
  // Lower bound for the keypoint score. Keypoints with a lower score will
  // be removed.
  float orb_detector_score_lower_bound;
  int orb_detector_fast_threshold;


  // Maximum number of keypoint to detect.
  size_t max_feature_count;

  // Enforce a minimal distance to the image border for feature tracking.
  // Hence, features can be detected close to the image border but
  // might not be tracked if the predicted location of the keypoint in the
  // next frame is closer to the image border than this value.
  size_t min_tracking_distance_to_image_border_px;

  double keypoint_uncertainty_px;
};

}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_FEATURE_TRACKING_TYPES_H_
