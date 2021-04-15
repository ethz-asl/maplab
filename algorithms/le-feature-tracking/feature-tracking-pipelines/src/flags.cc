#include "feature-tracking-pipelines/flags.h"

#include <cmath>

#include <gflags/gflags.h>
#include <maplab-common/conversions.h>

namespace feature_tracking_pipelines {
DEFINE_int32(NEW_feature_tracker_num_features, 300, "Number of features.");
DEFINE_int32(
    NEW_feature_tracker_num_pyramid_levels, 3, "Number of pyramid levels.");
DEFINE_double(
    NEW_feature_tracker_pyramid_scale_decimation, 1.5,
    "Pyramid decimation ratio, greater than 1.");
DEFINE_double(
    NEW_feature_tracker_min_eigen_threshold, 0.001, "Min Eigen Threshold ");
DEFINE_int32(NEW_feature_tracker_operation_flag, 0, "Operation Flag ");
DEFINE_int32(
    NEW_feature_tracker_criteria_max_count, 20,
    "Maximum iterations of LK-tracker");
DEFINE_double(
    NEW_feature_tracker_criteria_epsilon, 0.03,
    "Termination of search when windows moves less than this of LK-tracker");
DEFINE_int32(
    NEW_feature_tracker_window_height, 11,
    "Window height of the tracking window");
DEFINE_int32(
    NEW_feature_tracker_window_width, 11,
    "Window width of the tracking window");
DEFINE_double(
    NEW_feature_tracker_min_tracking_distance, 1.0,
    "Minimal distance of tracked features");
DEFINE_double(
    NEW_feature_tracker_ransac_threshold, 1.0 - cos(0.5 * kDegToRad),
    "Threshold for the RANSAC; defined as (1 - cos(alpha)) where alpha is "
    "the angle between the predicted and measured bearing vectors.");
DEFINE_int32(
    NEW_feature_tracker_ransac_max_iters, 200, "Max ransac iterations");
DEFINE_bool(
    NEW_eature_tracker_ransac_fix_seed, false, "Fix ransac random seed?");

// ORB detector specific settings.
DEFINE_int32(
    NEW_feature_tracker_detector_orb_edge_threshold, 31,
    "Edge threshold of the detector. Should roughly match the descriptor"
    "pattern size.");
DEFINE_int32(
    NEW_feature_tracker_detector_orb_fast_threshold, 20,
    "Threshold on difference between intensity of the central pixel and pixels "
    "of a circle around this pixel.");

// Brisk.
DEFINE_double(
    NEW_feature_tracker_brisk_uniformity_radius, 5.0,
    "BRISK uniformity radius parameter.");
DEFINE_double(
    NEW_feature_tracker_detector_brisk_absolute_threshold, 35.0,
    "BRISK absolute threshold parameter.");

// SIFT
DEFINE_int32(NEW_feature_tracker_sift_n_features, 0, "Number of features.");
DEFINE_int32(NEW_feature_tracker_sift_n_octave_layers, 3, "Number of layers.");
DEFINE_double(
    NEW_feature_tracker_sift_contrast_threshold, 0.04, "Contrast threshold.");
DEFINE_double(NEW_feature_tracker_sift_edge_threshold, 10, "edge threshold.");
DEFINE_double(
    NEW_feature_tracker_sift_sigma, 1.6, "The sigma of the Gaussian.");

// SURF
DEFINE_double(
    NEW_feature_tracker_surf_hessian_threshold, 100, "Hessian threshold.");
DEFINE_int32(NEW_feature_tracker_surf_n_octaves, 4, "Number of octaves.");
DEFINE_int32(
    NEW_feature_tracker_surf_octave_layers, 3, "Numer of octave layers.");
DEFINE_bool(NEW_feature_tracker_surf_extended, false, "Use 64 or 128 floats.");
DEFINE_bool(
    NEW_feature_tracker_surf_upright, false,
    "Computation of the feature orientation.");

// Feature factory.
DEFINE_string(NEW_feature_detector_type, "brisk", "Feature detector type.");
DEFINE_string(
    NEW_feature_descriptor_type, "freak", "descriptor pipeline type.");
DEFINE_string(NEW_feature_pipeline_type, "lk", "Feature pipeline type.");

}  // namespace feature_tracking_pipelines
