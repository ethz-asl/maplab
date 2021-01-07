#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

// Point cloud preprocessing.
DEFINE_double(
    regbox_pcl_downsample_leaf_size_m, 0.4f,
    "Defines the leaf size of the voxel grid.");

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
