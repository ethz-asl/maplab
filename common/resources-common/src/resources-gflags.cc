#include "resources-common/resources-gflags.h"

// Point cloud compression.
DEFINE_bool(
    resources_compress_pointclouds, false,
    "Defines if pointclouds are"
    "exported using draco compression.");
DEFINE_bool(
    resources_pointcloud_compression_add_indices, false,
    "Save additional field with index of point in cloud to be able to "
    "recover point order when loading file.");
DEFINE_int32(resources_pointcloud_compression_speed, 0, "Draco encoder speed");
DEFINE_int32(
    resources_pointcloud_compression_quantization_bits, 13,
    "Amount of quantization bits used for draco pointcloud compression.");
DEFINE_string(
    resources_pointcloud_ignore_fields, "",
    "Fields that should be "
    "ignored from point cloud input. Options are: 'normals, scalars, color, "
    "labels, rings, time' and any combination of these separated by a comma.");
