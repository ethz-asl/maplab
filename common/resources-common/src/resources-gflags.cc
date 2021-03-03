#include "resources-common/resources-gflags.h"

namespace resources {

// Point cloud compression.
DEFINE_bool(
    resources_compress_pointclouds, false,
    "Defines if pointclouds are"
    "exported using draco compression.");
DEFINE_int32(resources_pointcloud_compression_speed, 7, "Draco encoder speed");
DEFINE_int32(
    resources_pointcloud_compression_quantization_bits, 10,
    "Amount of quantization bits used for draco pointcloud compression.");
}  // namespace resources
