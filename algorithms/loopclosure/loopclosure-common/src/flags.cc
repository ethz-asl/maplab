#include <gflags/gflags.h>

#include "loopclosure-common/flags.h"
#include "loopclosure-common/types.h"

DEFINE_string(
    feature_descriptor_type, loop_closure::kFeatureDescriptorBRISK,
    "The feature descriptors to use. [brisk, freak]");
DEFINE_double(
    lc_knn_epsilon, 3.0,
    "Epsilon approximation value for the nearest neighbor search.");
DEFINE_double(
    lc_knn_max_radius, 20.0, "Max radius for the nearest neighbor search.");
