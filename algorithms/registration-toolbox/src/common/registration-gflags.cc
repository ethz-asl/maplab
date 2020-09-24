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
}  // namespace regbox
