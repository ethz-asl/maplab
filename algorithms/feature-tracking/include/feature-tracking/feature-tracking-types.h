#ifndef FEATURE_TRACKING_FEATURE_TRACKING_TYPES_H_
#define FEATURE_TRACKING_FEATURE_TRACKING_TYPES_H_

#include <opencv2/features2d/features2d.hpp>

#include <string>

namespace feature_tracking {

struct FeatureTrackingExtractorSettings {
  // TODO(smauq): Completely move away from internal Freak
  enum class DescriptorType { kOcvFreak, kBrisk };
  FeatureTrackingExtractorSettings();
  DescriptorType convertStringToDescriptorType(
      const std::string& descriptor_string);
  DescriptorType descriptor_type;

  /// Common settings of all descriptors.
  bool rotation_invariant;
  bool scale_invariant;
  bool flip_descriptor;

  /// FREAK settings.
  float freak_pattern_scale;
};

struct FeatureTrackingDetectorSettings {
  FeatureTrackingDetectorSettings();

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
  cv::ORB::ScoreType orb_detector_score_type;
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

  // Settings for gridded detector to ensure a certain distribution of keypoints
  // across  the image.
  bool gridded_detector_use_gridded;
  double gridded_detector_cell_num_features_scaler;
  size_t gridded_detector_cell_num_features;
  size_t gridded_detector_num_grid_cols;
  size_t gridded_detector_num_grid_rows;
  size_t gridded_detector_num_threads_per_image;
};

struct FeatureTrackingOutlierSettings {
  FeatureTrackingOutlierSettings();

  // Threshold for the 2-pt RANSAC outlier rejection.
  double two_pt_ransac_threshold;
  // Maximum number of iterations for the outlier rejection RANSAC.
  size_t two_pt_ransac_max_iterations;
  // If the outlier rejection should be deterministic, (i.e. fixed seed).
  // Useful for doing experiments with repeatable results.
  bool deterministic;
};


}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_FEATURE_TRACKING_TYPES_H_
