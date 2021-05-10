#ifndef RESOURCES_COMMON_RESOURCES_GFLAGS_H_
#define RESOURCES_COMMON_RESOURCES_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

// Point cloud compression.
DECLARE_bool(resources_compress_pointclouds);
DECLARE_bool(resources_pointcloud_compression_add_indices);
DECLARE_int32(resources_pointcloud_compression_speed);
DECLARE_int32(resources_pointcloud_compression_quantization_bits);

#endif  // RESOURCES_COMMON_RESOURCES_GFLAGS_H_
