#ifndef RESOURCES_COMMON_POINT_CLOUD_H_
#define RESOURCES_COMMON_POINT_CLOUD_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>

#include "resources-common/tinyply/tinyply.h"

namespace resources {
struct VoxelPosition {
  int64_t x, y, z;

  VoxelPosition(const Eigen::Vector3f& xyz, float voxel_size)
      : VoxelPosition(xyz[0], xyz[1], xyz[2], voxel_size) {}

  VoxelPosition(float _x, float _y, float _z, float voxel_size) {
    _x /= voxel_size;
    _y /= voxel_size;
    _z /= voxel_size;
    x = static_cast<int64_t>((_x >= 0) ? _x : _x - 1.0);
    y = static_cast<int64_t>((_y >= 0) ? _y : _y - 1.0);
    z = static_cast<int64_t>((_z >= 0) ? _z : _z - 1.0);
  }

  bool operator==(const VoxelPosition& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

struct Voxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f xyz;
  size_t count;

  Voxel(const Eigen::Vector3f& _xyz) : xyz(_xyz), count(1) {}

  Voxel& operator+=(const Eigen::Vector3f& new_xyz) {
    xyz = (xyz * count + new_xyz) / (count + 1);
    ++count;

    return *this;
  }
};
}  // namespace resources

namespace std {
template <>
struct hash<resources::VoxelPosition> {
  size_t operator()(const resources::VoxelPosition& s) const {
    // Szudzik triplet pairing function taken from and unrolled
    // https://drhagen.com/blog/multidimensional-pairing-functions/
    // with modifications to support negative values
    const size_t xx = (s.x >= 0) ? 2 * s.x : -2 * s.x - 1;
    const size_t yy = (s.y >= 0) ? 2 * s.y : -2 * s.y - 1;
    const size_t zz = (s.z >= 0) ? 2 * s.z : -2 * s.z - 1;

    if (xx >= yy && xx >= zz) {
      return ((xx + 1) * (xx + 1) * xx) + zz * (xx + 1) + yy;
    } else if (yy >= xx && yy >= zz) {
      return (yy * yy * yy) + ((yy + 1) * (yy + 1) - yy * yy) * xx + yy + zz;
    } else {
      return (zz * zz * zz) + ((zz + 1) * (zz + 1) - zz * zz) * xx + yy;
    }
  }
};
}  // namespace std

namespace resources {
typedef Eigen::Matrix<uint8_t, 4, 1> RgbaColor;

struct BoundingBox3D {
  double x_min, x_max;
  double y_min, y_max;
  double z_min, z_max;

  bool operator==(const BoundingBox3D& other) const {
    return (x_min == other.x_min) && (x_max == other.x_max) &&
           (y_min == other.y_min) && (y_max == other.y_max) &&
           (z_min == other.z_min) && (z_max == other.z_max);
  }
};

class PointCloud {
 public:
  void resize(
      const size_t size, const bool has_normals = true,
      const bool has_colors = true, const bool has_scalars = true,
      const bool has_labels = true, const bool has_times = true);

  size_t size() const {
    CHECK_EQ(xyz.size() % 3, 0u);
    return (xyz.size() / 3);
  }

  bool empty() const {
    return xyz.empty();
  }

  void clear() {
    xyz.clear();
    normals.clear();
    colors.clear();
    scalars.clear();
    labels.clear();
    times_ns.clear();
  }

  bool hasNormals() const {
    return normals.size() == xyz.size() && !normals.empty();
  }

  bool hasColor() const {
    return colors.size() == xyz.size() && !colors.empty();
  }

  bool hasScalars() const {
    return (scalars.size() == xyz.size() / 3u) && !scalars.empty();
  }

  bool hasLabels() const {
    return (labels.size() == xyz.size() / 3u) && !labels.empty();
  }

  bool hasTimes() const {
    return (times_ns.size() == xyz.size() / 3u) && !times_ns.empty();
  }

  bool checkConsistency(const bool verbose = false) const;

  // Append the information in another point cloud to this one. The two point
  // clouds must have the same fields filled in.
  void append(const PointCloud& other);

  // Apply transformation T_A_B to point cloud, assuming the point cloud is
  // currently expressed in the B frame. Applies the transformation to points
  // starting from index (by default applies it to the entire point cloud).
  void applyTransformation(
      const aslam::Transformation& T_A_B, size_t index = 0);

  // Transforms the other point cloud and appends it to the current one. Assumes
  // the other point cloud is in B frame.
  void appendTransformed(
      const PointCloud& other, const aslam::Transformation& T_A_B);

  // Get the min and max timestamps for all the points. Useful for undistortion.
  void getMinMaxTimeNanoseconds(
      int32_t* min_time_ns, int32_t* max_time_ns) const;

  // Undistort the point cloud given a set of initial high accuracy poses
  // from the start to the end of the scan. Linear interpolation will be used
  // to get poses in between the intial ones.
  void undistort(
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps,
      const aslam::TransformationVector& poses);

  // Removes points that are invalid (0, inf, nan), outside the sensor range
  // (too far or too close), or inside a predefined 3D bounding box. The box
  // filter is optional, pass a nullptr to skip.
  size_t filterValidMinMaxBox(
      double min_range_m, double max_range_m, const BoundingBox3D* box_filter);

  // Removes points inside a 3D bounding box.
  void filterBoundingBox3D(BoundingBox3D box_filter);

  // Downsample using a voxel grid. The returned point cloud will only
  // have xyz coordinates, all other information is stripped away.
  void downsampleVoxelized(float voxel_size, PointCloud* voxelized) const;

  void writeToFile(const std::string& file_path) const;
  bool loadFromFile(const std::string& file_path);

  bool operator==(const PointCloud& other) const {
    bool is_same = xyz == other.xyz;
    is_same &= normals == other.normals;
    is_same &= colors == other.colors;
    is_same &= scalars == other.scalars;
    is_same &= labels == other.labels;
    is_same &= times_ns == other.times_ns;
    return is_same;
  }

  std::vector<float> xyz;
  std::vector<float> normals;
  std::vector<unsigned char> colors;
  std::vector<float> scalars;
  std::vector<uint32_t> labels;
  std::vector<int32_t> times_ns;
};

}  // namespace resources

#endif  // RESOURCES_COMMON_POINT_CLOUD_H_
