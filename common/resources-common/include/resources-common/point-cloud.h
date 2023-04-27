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

typedef Eigen::Matrix<uint8_t, 4, 1> RgbaColor;

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

  void removeInvalidPoints();

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
