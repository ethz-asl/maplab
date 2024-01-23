#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/conversions.h>

#include "feature-tracking/feature-tracking-types.h"

DEFINE_string(
    feature_tracking_descriptor_type, "brisk",
    "Descriptor type to compute: 'brisk' or 'freak'");
DEFINE_bool(
    feature_tracking_descriptor_scale_invariance, true,
    "Enable scale invariance of the chosen descriptor. "
    "Only useful if detector provides scale information "
    "by using multiple octaves.");
DEFINE_bool(
    feature_tracking_descriptor_rotation_invariance, false,
    "Enable rotation invariance for descriptor extraction. "
    "Expect deteriorated matching performance.");
DEFINE_bool(
    feature_tracking_flip_descriptors, false,
    "Rotate the descriptor pattern by 180 deg? This can be used to "
    "align descriptors between datasets that have been recorded \"upside "
    "down\".");
DEFINE_double(
    feature_tracking_ocvfreak_pattern_scale, 22.0,
    "Scaling of the description pattern.");
DEFINE_bool(
    feature_tracking_detector_use_nms, true,
    "Should non-maximum suppression be applied after feature detection?");
DEFINE_double(
    feature_tracking_detector_nms_radius, 8.0,
    "Radius around queried keypoint in which other keypoints can be "
    "suppressed.");
DEFINE_double(
    feature_tracking_detector_nms_ratio_threshold, 0.9,
    "Suppress keypoints if their response is lower than this threshold times "
    "the response of the queried keypoint. A lower threshold should make the "
    "suppression more robust. However, this will result in more keypoints "
    "being close to each other.");
DEFINE_int32(
    feature_tracking_detector_orb_num_features, 2000,
    "How many features to detect. "
    "Choose much fewer features if non-max suppression is not used.");
DEFINE_double(
    feature_tracking_detector_orb_scale_factor, 1.2,
    "Pyramid decimation ratio, greater than 1. scaleFactor==2 means "
    "the classical pyramid, where each next level has 4x less pixels "
    "than the previous, but such a big scale factor will degrade "
    "feature matching scores dramatically. On the other hand, too "
    "close to 1 scale factor will mean that to cover certain scale "
    "range you will need more pyramid levels and so the speed will suffer.");
DEFINE_int32(
    feature_tracking_detector_orb_pyramid_levels, 1,
    "The number of pyramid levels. Higher number of pyramids will "
    "increase scale invariance properties but will also lead "
    "accumulations of keypoints in hotspots. Feature detection can "
    "slow down considerably with higher numbers of pyramid levels.");
DEFINE_int32(
    feature_tracking_detector_orb_patch_size, 20,
    "Size of the patch used by the oriented BRIEF descriptor. "
    "On smaller pyramid layers the perceived image "
    "area covered by a feature will be larger.");
DEFINE_double(
    feature_tracking_detector_orb_score_lower_bound, 1e-7,
    "Keypoints with a score lower than this value will get removed. "
    "This can be useful to remove keypoints of low quality or "
    "keypoints that are associated with image noise.");
DEFINE_int32(
    feature_tracking_detector_orb_fast_threshold, 20,
    "Threshold on difference between intensity of the central pixel and pixels "
    "of a circle around this pixel.");
DEFINE_uint64(
    feature_tracking_detector_max_feature_count, 700,
    "Max. number of features to detect. After the whole detection "
    "pipeline the number of features will be cut.");

DEFINE_uint64(
    min_tracking_distance_to_image_border_px, 30u,
    "Minimum tracking distance to the image border in pixels. Has to be "
    "greater or equal to the descriptor patch size.");
DEFINE_bool(
    feature_tracking_gridded_detector_use_gridded, true,
    "Use multiple detectors on a grid?");
DEFINE_uint64(
    feature_tracking_gridded_detector_num_grid_cols, 4u,
    "Number of grid columns for gridded detection.");
DEFINE_uint64(
    feature_tracking_gridded_detector_num_grid_rows, 2u,
    "Number of grid rows for gridded detection.");
DEFINE_double(
    feature_tracking_gridded_detector_cell_num_features_scaler, 2.0,
    "Scaler to increase number of keypoints detector per grid cell "
    "(keypoints_per_cell = total_keypoints * scaler / number_of_cells).");
DEFINE_uint64(
    feature_tracking_gridded_detection_num_threads_per_image, 0u,
    "Number of hardware threads used for detection (0 means N_cell/2).");

DEFINE_double(
    feature_tracker_two_pt_ransac_threshold, 1.0 - cos(0.5 * kDegToRad),
    "Threshold for the 2-pt RANSAC used for feature tracking outlier "
    "removal. The error is defined as (1 - cos(alpha)) where alpha is "
    "the angle between the predicted and measured bearing vectors.");
DEFINE_uint64(
    feature_tracker_two_pt_ransac_max_iterations, 200,
    "Max iterations for the 2-pt RANSAC used for feature tracking "
    "outlier removal.");
DEFINE_bool(
    feature_tracker_deterministic, false,
    "If true, deterministic RANSAC outlier rejection is used.");

namespace feature_tracking {

FeatureTrackingExtractorSettings::FeatureTrackingExtractorSettings()
    : descriptor_type(convertStringToDescriptorType(
          FLAGS_feature_tracking_descriptor_type)),
      rotation_invariant(FLAGS_feature_tracking_descriptor_rotation_invariance),
      scale_invariant(FLAGS_feature_tracking_descriptor_scale_invariance),
      flip_descriptor(FLAGS_feature_tracking_flip_descriptors),
      freak_pattern_scale(FLAGS_feature_tracking_ocvfreak_pattern_scale) {
  CHECK_GT(freak_pattern_scale, 0.0f);
  if (flip_descriptor) {
    // We need to set the rotation invariance to true in case the descriptor
    // masks should be flipped. The actual rotation of the mask is achieved
    // by settings the angle to 180.0 deg for the keypoints passed to the
    // descriptor extractor.
    CHECK(rotation_invariant);
  }
}

FeatureTrackingExtractorSettings::DescriptorType
FeatureTrackingExtractorSettings::convertStringToDescriptorType(
    const std::string& descriptor_string) {
  if (descriptor_string == "brisk") {
    return DescriptorType::kBrisk;
  } else if (descriptor_string == "freak") {
    return DescriptorType::kOcvFreak;
  }
  LOG(FATAL) << "Unknown descriptor type: "
             << FLAGS_feature_tracking_descriptor_type;
  return DescriptorType::kBrisk;
}

FeatureTrackingDetectorSettings::FeatureTrackingDetectorSettings()
    : detector_use_nonmaxsuppression(FLAGS_feature_tracking_detector_use_nms),
      detector_nonmaxsuppression_radius(
          FLAGS_feature_tracking_detector_nms_radius),
      detector_nonmaxsuppression_ratio_threshold(
          FLAGS_feature_tracking_detector_nms_ratio_threshold),
      orb_detector_number_features(
          FLAGS_feature_tracking_detector_orb_num_features),
      orb_detector_scale_factor(
          FLAGS_feature_tracking_detector_orb_scale_factor),
      orb_detector_pyramid_levels(
          FLAGS_feature_tracking_detector_orb_pyramid_levels),
      orb_detector_first_level(0),
      orb_detector_WTA_K(2),
      orb_detector_score_type(cv::ORB::ScoreType::HARRIS_SCORE),
      orb_detector_patch_size(FLAGS_feature_tracking_detector_orb_patch_size),
      orb_detector_score_lower_bound(
          FLAGS_feature_tracking_detector_orb_score_lower_bound),
      orb_detector_fast_threshold(
          FLAGS_feature_tracking_detector_orb_fast_threshold),
      max_feature_count(FLAGS_feature_tracking_detector_max_feature_count),
      min_tracking_distance_to_image_border_px(
          FLAGS_min_tracking_distance_to_image_border_px),
      keypoint_uncertainty_px(0.8),
      gridded_detector_use_gridded(
          FLAGS_feature_tracking_gridded_detector_use_gridded),
      gridded_detector_cell_num_features_scaler(
          FLAGS_feature_tracking_gridded_detector_cell_num_features_scaler),
      gridded_detector_cell_num_features(
          FLAGS_feature_tracking_detector_orb_num_features *
          FLAGS_feature_tracking_gridded_detector_cell_num_features_scaler /
          (FLAGS_feature_tracking_gridded_detector_num_grid_cols *
           FLAGS_feature_tracking_gridded_detector_num_grid_rows)),
      gridded_detector_num_grid_cols(
          FLAGS_feature_tracking_gridded_detector_num_grid_cols),
      gridded_detector_num_grid_rows(
          FLAGS_feature_tracking_gridded_detector_num_grid_rows),
      gridded_detector_num_threads_per_image(
          FLAGS_feature_tracking_gridded_detection_num_threads_per_image) {
  orb_detector_edge_threshold =
      static_cast<int>(min_tracking_distance_to_image_border_px);

  CHECK_LE(orb_detector_patch_size, orb_detector_edge_threshold)
      << "Keypoints might be removed. Choose a smaller patch size.";
  CHECK_GE(orb_detector_pyramid_levels, 1);
  CHECK_GT(orb_detector_scale_factor, 1.0);
  CHECK_GE(orb_detector_score_lower_bound, 0.0f);
  CHECK_GT(orb_detector_fast_threshold, 0);
  CHECK_GE(gridded_detector_cell_num_features_scaler, 0);
  CHECK_GE(gridded_detector_cell_num_features, 0u);
  CHECK_GE(gridded_detector_num_grid_cols, 1u);
  CHECK_GE(gridded_detector_num_grid_rows, 1u);
}

FeatureTrackingOutlierSettings::FeatureTrackingOutlierSettings()
    : two_pt_ransac_threshold(FLAGS_feature_tracker_two_pt_ransac_threshold),
      two_pt_ransac_max_iterations(
          FLAGS_feature_tracker_two_pt_ransac_max_iterations),
      deterministic(FLAGS_feature_tracker_deterministic) {
  CHECK_GE(two_pt_ransac_threshold, 0.0);
  CHECK_LE(two_pt_ransac_threshold, 1.0);
}

}  // namespace feature_tracking
