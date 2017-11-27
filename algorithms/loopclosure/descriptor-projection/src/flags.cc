#include <descriptor-projection/flags.h>
#include <gflags/gflags.h>

DEFINE_string(
    lc_projection_matrix_filename, "",
    "The name of the file for the projection matrix.");
DEFINE_string(
    lc_projected_quantizer_filename, "",
    "File where to read the projected quantizer from.");
DEFINE_int32(
    lc_target_dimensionality, 10,
    "The target dimensionality of the projection.");
