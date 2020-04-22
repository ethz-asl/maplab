#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

// Point cloud preprocessing.
DEFINE_double(
    regbox_pcl_downsample_leaf_size_m, 0.4f,
    "Defines the leaf size of the voxel grid.");

}  // namespace regbox
