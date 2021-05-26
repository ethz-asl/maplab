#ifndef REGISTRATION_TOOLBOX_COMMON_REGISTRATION_GFLAGS_H_
#define REGISTRATION_TOOLBOX_COMMON_REGISTRATION_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace regbox {

// Alignment method independent flags
DECLARE_double(regbox_fixed_covariance);

// pcl config
DECLARE_double(regbox_pcl_downsample_leaf_size_m);
DECLARE_bool(regbox_pcl_use_downsampling);
DECLARE_int32(regbox_pcl_gicp_n_neighbors);
DECLARE_int32(regbox_pcl_max_iterations);
DECLARE_double(regbox_pcl_fitness_max_considered_distance_m);
DECLARE_double(regbox_pcl_max_fitness_score_m);

// libpointmatcher config
DECLARE_string(regbox_lpm_config_path);
DECLARE_string(regbox_lpm_input_filter_config_path);
DECLARE_double(regbox_lpm_icp_match_residual_error_threshold_m);
DECLARE_bool(regbox_lpm_use_computed_covariance);

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_COMMON_REGISTRATION_GFLAGS_H_
