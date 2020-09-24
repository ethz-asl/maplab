#ifndef REGISTRATION_TOOLBOX_COMMON_REGISTRATION_GFLAGS_H_
#define REGISTRATION_TOOLBOX_COMMON_REGISTRATION_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace regbox {

// Point cloud preprocessing.
DECLARE_double(regbox_pcl_downsample_leaf_size_m);

// libpointmatcher config
DECLARE_string(regbox_lpm_config_path);
DECLARE_string(regbox_lpm_input_filter_config_path);

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_COMMON_REGISTRATION_GFLAGS_H_
