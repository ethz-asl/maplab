#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

// Alignment method independent flags
DEFINE_double(
    regbox_fixed_covariance_translation_m, 1e-2,
    "Fixed covariance value for the translational part of registration results "
    "in meteres.");

DEFINE_double(
    regbox_fixed_covariance_rotation_rad, 1e-1,
    "Fixed covariance value for the rotational part of registration results "
    "in radians.");

// pcl config
DEFINE_double(
    regbox_pcl_downsample_leaf_size_m, 0.15f,
    "Defines the leaf size of the voxel grid.");

DEFINE_bool(
    regbox_pcl_use_downsampling, true, "Defines if downsampling is performed.");

DEFINE_int32(
    regbox_pcl_gicp_n_neighbors, 15,
    "Amount of neighboring points used for GICP covariance estimation.");

DEFINE_int32(
    regbox_pcl_max_iterations, 50, "Maximum number of alignment iterations.");

DEFINE_double(
    regbox_pcl_fitness_max_considered_distance_m, 1.,
    "Maximum distance between points to be considered inilers in the euclidean "
    "fitness score calculation");

DEFINE_double(
    regbox_pcl_max_fitness_score_m, 0.15,
    "Maximum fitness score allowed for successful alignment");

// libpointmatcher config
DEFINE_string(
    regbox_lpm_config_path, "",
    "Path to the configuration for libpointmatcher.");
DEFINE_string(
    regbox_lpm_input_filter_config_path, "",
    "Path to the input filter configuration for libpointmatcher.");
DEFINE_double(
    regbox_lpm_icp_match_residual_error_threshold_m, 0.1,
    "Threshold to decide whether the registration was successful.");
DEFINE_bool(
    regbox_lpm_use_computed_covariance, false,
    "If true, the LC edge uses the computed covariance if it is valid.");
}  // namespace regbox
