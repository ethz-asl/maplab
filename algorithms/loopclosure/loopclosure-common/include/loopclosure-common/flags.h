#ifndef LOOPCLOSURE_COMMON_FLAGS_H_
#define LOOPCLOSURE_COMMON_FLAGS_H_

#include <gflags/gflags.h>

DECLARE_string(feature_descriptor_type);
// Flags for kd-tree-based nearest neighbor search.
DECLARE_double(lc_knn_epsilon);
DECLARE_double(lc_knn_max_radius);

#endif  // LOOPCLOSURE_COMMON_FLAGS_H_
