#ifndef MAP_RESOURCES_RESOURCE_TYPEDEFS_H_
#define MAP_RESOURCES_RESOURCE_TYPEDEFS_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>
#include <voxblox/core/common.h>

namespace resources {

typedef Eigen::Matrix<uint8_t, 4, 1> RgbaColor;

struct VoxbloxColorPointCloud {
  pose::Position3DVector* points_C;
  voxblox::Colors* colors;
};

struct PointCloud {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<float> xyz;
  std::vector<float> normals;
  std::vector<unsigned char> colors;

  void resize(size_t size) {
    xyz.resize(3 * size);
    normals.resize(3 * size);
    colors.resize(3 * size);
  }

  size_t size() const {
    CHECK_EQ(xyz.size() % 3, 0u);
    return (xyz.size() / 3);
  }

  bool empty() const {
    return xyz.empty();
  }
};

}  // namespace resources

#endif  // MAP_RESOURCES_RESOURCE_TYPEDEFS_H_
