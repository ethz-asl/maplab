#include "feature-tracking/feature-tracking-types.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/features2d/features2d.hpp>

DEFINE_string(
    swe_feature_tracking_descriptor_type, "brisk",
    "Descriptor type to compute: 'brisk' or 'freak'");
DEFINE_bool(
    swe_feature_tracking_descriptor_scale_invariance, true,
    "Enable scale invariance of the chosen descriptor. "
    "Only useful if detector provides scale information "
    "by using multiple octaves.");
DEFINE_bool(
    swe_feature_tracking_descriptor_rotation_invariance, false,
    "Enable rotation invariance for descriptor extraction. "
    "Expect deteriorated matching performance.");
DEFINE_bool(
    swe_feature_tracking_flip_descriptors, false,
    "Rotate the descriptor pattern by 180 deg? This can be used to "
    "align descriptors between datasets that have been recorded \"upside "
    "down\".");
DEFINE_double(
    swe_feature_tracking_ocvfreak_pattern_scale, 22.0,
    "Scaling of the description pattern.");
DEFINE_bool(
    swe_feature_tracking_detector_use_nms, true,
    "Should non-maximum suppression be applied after feature detection?");
DEFINE_double(
    swe_feature_tracking_detector_nms_radius, 8.0,
    "Radius around queried keypoint in which other keypoints can be "
    "suppressed.");
DEFINE_double(
    swe_feature_tracking_detector_nms_ratio_threshold, 0.9,
    "Suppress keypoints if their response is lower than this threshold times "
    "the response of the queried keypoint. A lower threshold should make the "
    "suppression more robust. However, this will result in more keypoints "
    "being close to each other.");
DEFINE_int32(
    swe_feature_tracking_detector_orb_num_features, 2000,
    "How many features to detect. "
    "Choose much fewer features if non-max suppression is not used.");
DEFINE_double(
    swe_feature_tracking_detector_orb_scale_factor, 1.2,
    "Pyramid decimation ratio, greater than 1. scaleFactor==2 means "
    "the classical pyramid, where each next level has 4x less pixels "
    "than the previous, but such a big scale factor will degrade "
    "feature matching scores dramatically. On the other hand, too "
    "close to 1 scale factor will mean that to cover certain scale "
    "range you will need more pyramid levels and so the speed will suffer.");
DEFINE_int32(
    swe_feature_tracking_detector_orb_pyramid_levels, 1,
    "The number of pyramid levels. Higher number of pyramids will "
    "increase scale invariance properties but will also lead "
    "accumulations of keypoints in hotspots. Feature detection can "
    "slow down considerably with higher numbers of pyramid levels.");
DEFINE_int32(
    swe_feature_tracking_detector_orb_patch_size, 20,
    "Size of the patch used by the oriented BRIEF descriptor. "
    "On smaller pyramid layers the perceived image "
    "area covered by a feature will be larger.");
DEFINE_double(
    swe_feature_tracking_detector_orb_score_lower_bound, 1e-7,
    "Keypoints with a score lower than this value will get removed. "
    "This can be useful to remove keypoints of low quality or "
    "keypoints that are associated with image noise.");
DEFINE_int32(
    swe_feature_tracking_detector_orb_fast_threshold, 20,
    "Threshold on difference between intensity of the central pixel and pixels "
    "of a circle around this pixel.");
DEFINE_uint64(
    swe_feature_tracking_detector_max_feature_count, 700,
    "Max. number of features to detect. After the whole detection "
    "pipeline the number of features will be cut.");

DEFINE_int32(
    feature_tracker_simple_pipeline_brisk_num_octaves, 1,
    "BRISK number of octaves parameter.");
DEFINE_double(
    feature_tracker_simple_pipeline_brisk_uniformity_radius, 0.0,
    "BRISK uniformity radius parameter.");
DEFINE_double(
    feature_tracker_simple_pipeline_brisk_absolute_threshold, 45.0,
    "BRISK absolute threshold parameter.");
DEFINE_int32(
    feature_tracker_simple_pipeline_max_num_keypoints, 1000,
    "The maximum number of keypoints detected in a frame.");
DEFINE_bool(
    feature_tracker_simple_pipeline_brisk_rotation_invariant, false,
    "BRISK rotation invariance parameter.");
DEFINE_bool(
    feature_tracker_simple_pipeline_brisk_scale_invariant, true,
    "BRISK scale invariance parameter.");
DEFINE_bool(
    feature_tracker_simple_pipeline_brisk_copy_images, true,
    "BRISK copy images parameter.");
DEFINE_double(
    feature_tracker_simple_pipeline_matcher_image_space_distance_threshold_px,
    50.0,
    "Search radius in image space for possible frame-to-frame keypoint "
    "matches.");
DEFINE_int32(
    feature_tracker_simple_pipeline_matcher_descriptor_hamming_distance_threshold,
    60, "Frame-matching: max descriptor distance between two keypoints.");

namespace feature_tracking {

SweFeatureTrackingExtractorSettings::SweFeatureTrackingExtractorSettings()
    : descriptor_type(
          convertStringToDescriptorType(
              FLAGS_swe_feature_tracking_descriptor_type)),
      rotation_invariant(
          FLAGS_swe_feature_tracking_descriptor_rotation_invariance ||
          FLAGS_swe_feature_tracking_flip_descriptors),
      scale_invariant(FLAGS_swe_feature_tracking_descriptor_scale_invariance),
      flip_descriptor(FLAGS_swe_feature_tracking_flip_descriptors),
      freak_pattern_scale(FLAGS_swe_feature_tracking_ocvfreak_pattern_scale) {
  CHECK_GT(freak_pattern_scale, 0.0f);
  if (flip_descriptor) {
    // We need to set the rotation invariance to true in case the descriptor
    // masks should be flipped. The actual rotation of the mask is achieved
    // by settings the angle to 180.0 deg for the keypoints passed to the
    // descriptor extractor.
    CHECK(rotation_invariant);
  }
}

SweFeatureTrackingExtractorSettings::DescriptorType
SweFeatureTrackingExtractorSettings::convertStringToDescriptorType(
    const std::string& descriptor_string) {
  if (descriptor_string == "brisk") {
    return DescriptorType::kBrisk;
  } else if (descriptor_string == "freak") {
    return DescriptorType::kOcvFreak;
  }
  LOG(FATAL) << "Unknown descriptor type: "
             << FLAGS_swe_feature_tracking_descriptor_type;
  return DescriptorType::kBrisk;
}

SweFeatureTrackingDetectorSettings::SweFeatureTrackingDetectorSettings()
    : detector_use_nonmaxsuppression(
          FLAGS_swe_feature_tracking_detector_use_nms),
      detector_nonmaxsuppression_radius(
          FLAGS_swe_feature_tracking_detector_nms_radius),
      detector_nonmaxsuppression_ratio_threshold(
          FLAGS_swe_feature_tracking_detector_nms_ratio_threshold),
      orb_detector_number_features(
          FLAGS_swe_feature_tracking_detector_orb_num_features),
      orb_detector_scale_factor(
          FLAGS_swe_feature_tracking_detector_orb_scale_factor),
      orb_detector_pyramid_levels(
          FLAGS_swe_feature_tracking_detector_orb_pyramid_levels),
      orb_detector_first_level(0),
      orb_detector_WTA_K(2),
      orb_detector_score_type(cv::ORB::HARRIS_SCORE),
      orb_detector_patch_size(
          FLAGS_swe_feature_tracking_detector_orb_patch_size),
      orb_detector_score_lower_bound(
          FLAGS_swe_feature_tracking_detector_orb_score_lower_bound),
      orb_detector_fast_threshold(
          FLAGS_swe_feature_tracking_detector_orb_fast_threshold),
      max_feature_count(FLAGS_swe_feature_tracking_detector_max_feature_count),
      min_tracking_distance_to_image_border_px(30u),
      keypoint_uncertainty_px(0.8) {
  orb_detector_edge_threshold =
      static_cast<int>(min_tracking_distance_to_image_border_px);

  CHECK_LE(orb_detector_patch_size, orb_detector_edge_threshold)
      << "Keypoints might be removed. Choose a smaller patch size.";
  CHECK_GE(orb_detector_pyramid_levels, 1);
  CHECK_GT(orb_detector_scale_factor, 1.0);
  CHECK_GE(orb_detector_score_lower_bound, 0.0f);
  CHECK_GT(orb_detector_fast_threshold, 0);
}

SimpleBriskFeatureTrackingSettings::SimpleBriskFeatureTrackingSettings()
    : num_octaves(FLAGS_feature_tracker_simple_pipeline_brisk_num_octaves),
      uniformity_radius(
          FLAGS_feature_tracker_simple_pipeline_brisk_uniformity_radius),
      absolute_threshold(
          FLAGS_feature_tracker_simple_pipeline_brisk_absolute_threshold),
      max_number_of_keypoints(
          FLAGS_feature_tracker_simple_pipeline_max_num_keypoints),
      rotation_invariant(
          FLAGS_feature_tracker_simple_pipeline_brisk_rotation_invariant),
      scale_invariant(
          FLAGS_feature_tracker_simple_pipeline_brisk_scale_invariant),
      copy_images(FLAGS_feature_tracker_simple_pipeline_brisk_copy_images),
      matching_descriptor_hamming_distance_threshold(
          FLAGS_feature_tracker_simple_pipeline_matcher_descriptor_hamming_distance_threshold),
      matching_image_space_distance_threshold_px(
          FLAGS_feature_tracker_simple_pipeline_matcher_image_space_distance_threshold_px) {
}

}  // namespace feature_tracking
