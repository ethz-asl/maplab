#ifndef DESCRIPTOR_PROJECTION_FLAGS_H_
#define DESCRIPTOR_PROJECTION_FLAGS_H_
#include <string>
#include <vector>

#include <gflags/gflags.h>

DECLARE_string(lc_projection_matrix_filename);
DECLARE_int32(lc_target_dimensionality);
DECLARE_string(lc_projected_quantizer_filename);

namespace descriptor_projection {
typedef std::vector<unsigned int> Track;
}  // namespace descriptor_projection

#endif  // DESCRIPTOR_PROJECTION_FLAGS_H_
