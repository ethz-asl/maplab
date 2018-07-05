#ifndef MAP_RESOURCES_RESOURCE_TYPEDEFS_H_
#define MAP_RESOURCES_RESOURCE_TYPEDEFS_H_

#include <Eigen/Core>
#include <maplab-common/pose_types.h>
#include <voxblox/core/common.h>

namespace resources {

struct VoxbloxColorPointCloud {
  voxblox::Pointcloud* points_C;
  voxblox::Colors* colors;
};

}  // namespace resources

#endif  // MAP_RESOURCES_RESOURCE_TYPEDEFS_H_
