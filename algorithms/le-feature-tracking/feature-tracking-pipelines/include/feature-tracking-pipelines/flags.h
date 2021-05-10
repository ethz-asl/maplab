#ifndef FEATURE_TRACKING_PIPELINES_FLAGS_H_
#define FEATURE_TRACKING_PIPELINES_FLAGS_H_

#include <gflags/gflags.h>

namespace feature_tracking_pipelines {
// General detector settings.
DECLARE_int32(NEW_feature_tracker_num_features);
DECLARE_int32(NEW_feature_tracker_num_pyramid_levels);
DECLARE_double(NEW_feature_tracker_pyramid_scale_decimation);
DECLARE_double(NEW_feature_tracker_min_eigen_threshold);
DECLARE_int32(NEW_feature_tracker_operation_flag);
DECLARE_int32(NEW_feature_tracker_criteria_max_count);
DECLARE_double(NEW_feature_tracker_criteria_epsilon);
DECLARE_int32(NEW_feature_tracker_window_height);
DECLARE_int32(NEW_feature_tracker_window_width);
DECLARE_double(NEW_feature_tracker_min_tracking_distance);

// RANSAC.
DECLARE_double(NEW_feature_tracker_ransac_threshold);
DECLARE_int32(NEW_feature_tracker_ransac_max_iters);
DECLARE_bool(NEW_eature_tracker_ransac_fix_seed);

// ORB detector specific settings.
DECLARE_int32(NEW_feature_tracker_detector_orb_edge_threshold);
DECLARE_int32(NEW_feature_tracker_detector_orb_fast_threshold);

// BRISK.
DECLARE_double(NEW_feature_tracker_brisk_uniformity_radius);
DECLARE_double(NEW_feature_tracker_detector_brisk_absolute_threshold);

// SIFT
DECLARE_int32(NEW_feature_tracker_sift_n_features);
DECLARE_int32(NEW_feature_tracker_sift_n_octave_layers);
DECLARE_double(NEW_feature_tracker_sift_contrast_threshold);
DECLARE_double(NEW_feature_tracker_sift_edge_threshold);
DECLARE_double(NEW_feature_tracker_sift_sigma);

// SURF
DECLARE_double(NEW_feature_tracker_surf_hessian_threshold);
DECLARE_int32(NEW_feature_tracker_surf_n_octaves);
DECLARE_int32(NEW_feature_tracker_surf_octave_layers);
DECLARE_bool(NEW_feature_tracker_surf_extended);
DECLARE_bool(NEW_feature_tracker_surf_upright);

// Feature factory.
DECLARE_string(NEW_feature_detector_type);
DECLARE_string(NEW_feature_descriptor_type);
DECLARE_string(NEW_feature_pipeline_type);

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FLAGS_H_
