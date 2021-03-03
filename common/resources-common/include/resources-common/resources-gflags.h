#ifndef RESOURCES_COMMON_RESOURCES_GFLAGS_H_
#define RESOURCES_COMMON_RESOURCES_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace resources {

// Point cloud compression.
DECLARE_bool(resources_compress_pointclouds);
DECLARE_int32(resources_pointcloud_compression_speed);
DECLARE_int32(resources_pointcloud_compression_quantization_bits);
}  // namespace resources

#endif  // RESOURCES_COMMON_RESOURCES_GFLAGS_H_
