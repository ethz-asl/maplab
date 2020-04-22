#ifndef RESOURCES_COMMON_POINT_CLOUD_H_
#define RESOURCES_COMMON_POINT_CLOUD_H_

#include <cstdio>
#include <fstream>  // NOLINT
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/threading-helpers.h>

#include "resources-common/tinyply/tinyply.h"

namespace resources {

typedef Eigen::Matrix<uint8_t, 4, 1> RgbaColor;

struct PointCloud {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<float> xyz;
  std::vector<float> normals;
  std::vector<unsigned char> colors;
  std::vector<float> scalars;
  std::vector<uint32_t> labels;

  // Apply transformation T_A_B to pointcloud, assuming the pointcloud is
  // currently expressed in the B frame.
  inline void applyTransformation(const aslam::Transformation& T_A_B) {
    std::function<void(const std::vector<size_t>&)> transform_points_function =
        [this, &T_A_B](const std::vector<size_t>& batch) {
          for (size_t point_idx : batch) {
            const size_t idx = point_idx * 3;
            const Eigen::Vector3d point(xyz[idx], xyz[idx + 1u], xyz[idx + 2u]);
            const Eigen::Vector3f& transformed_point =
                (T_A_B * point).cast<float>();
            xyz[idx] = transformed_point.x();
            xyz[idx + 1u] = transformed_point.y();
            xyz[idx + 2u] = transformed_point.z();
          }
        };

    const size_t num_threads = common::getNumHardwareThreads();
    static constexpr bool kAlwaysParallelize = false;
    common::ParallelProcess(
        size(), transform_points_function, kAlwaysParallelize, num_threads);
  }

  inline void resize(
      const size_t size, const bool has_normals = true,
      const bool has_colors = true, const bool has_scalars = true,
      const bool has_labels = true) {
    xyz.resize(3 * size);

    if (has_normals) {
      normals.resize(3 * size);
    }

    if (has_colors) {
      colors.resize(3 * size);
    }

    if (has_scalars) {
      scalars.resize(1 * size);
    }

    if (has_labels) {
      labels.resize(1 * size);
    }
  }

  inline size_t size() const {
    CHECK_EQ(xyz.size() % 3, 0u);
    return (xyz.size() / 3);
  }

  inline bool empty() const {
    return xyz.empty();
  }

  inline bool hasNormals() const {
    return normals.size() == xyz.size() && !normals.empty();
  }

  inline bool hasColor() const {
    return colors.size() == xyz.size() && !colors.empty();
  }

  inline bool hasScalars() const {
    return (scalars.size() == xyz.size() / 3u) && !scalars.empty();
  }

  inline bool hasLabels() const {
    return (labels.size() == xyz.size() / 3u) && !labels.empty();
  }

  inline bool checkConsistency(const bool verbose = false) const {
    bool consistent = true;
    consistent &= (normals.size() == xyz.size()) || normals.empty();
    consistent &= (colors.size() == xyz.size()) || colors.empty();
    consistent &= (scalars.size() == xyz.size() / 3u) || scalars.empty();
    consistent &= (labels.size() == xyz.size() / 3u) || labels.empty();

    LOG_IF(ERROR, verbose && !consistent)
        << "\nInconsistent point cloud:"
        << "\n - Point vector size:  " << xyz.size()
        << "\n - Normal vector size: " << normals.size()
        << "\n - Color vector size:  " << colors.size()
        << "\n - Scalar vector size: " << scalars.size()
        << "\n - Label vector size: " << labels.size();
    return consistent;
  }

  inline void append(const PointCloud& other) {
    if (other.empty()) {
      return;
    }

    xyz.reserve(xyz.size() + other.xyz.size());
    normals.reserve(normals.size() + other.normals.size());
    colors.reserve(colors.size() + other.colors.size());
    scalars.reserve(scalars.size() + other.scalars.size());
    labels.reserve(labels.size() + other.labels.size());

    xyz.insert(xyz.end(), other.xyz.begin(), other.xyz.end());
    normals.insert(normals.end(), other.normals.begin(), other.normals.end());
    colors.insert(colors.end(), other.colors.begin(), other.colors.end());
    scalars.insert(scalars.end(), other.scalars.begin(), other.scalars.end());
    labels.insert(labels.end(), other.labels.begin(), other.labels.end());

    CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
  }

  inline void removeInvalidPoints() {
    auto it_xyz = xyz.begin();
    auto it_normals = normals.begin();
    auto it_colors = colors.begin();
    auto it_scalars = scalars.begin();
    auto it_labels = labels.begin();

    size_t removed_points = 0u;
    const size_t initial_number_of_points = xyz.size() / 3u;

    while (it_xyz != xyz.end()) {
      const float x = *it_xyz;
      const float y = *(it_xyz + 1);
      const float z = *(it_xyz + 2);

      if (((x * x + y * y + z * z) < 1e-10) || !std::isfinite(x) ||
          !std::isfinite(y) || !std::isfinite(z)) {
        ++removed_points;

        it_xyz = xyz.erase(it_xyz, it_xyz + 3);

        if (!normals.empty()) {
          it_normals = normals.erase(it_normals, it_normals + 3);
        }

        if (!colors.empty()) {
          it_colors = colors.erase(it_colors, it_colors + 3);
        }

        if (!scalars.empty()) {
          it_scalars = scalars.erase(it_scalars);
        }

        if (!labels.empty()) {
          it_labels = labels.erase(it_labels);
        }
      } else {
        it_xyz += 3;

        if (!normals.empty()) {
          it_normals += 3;
        }

        if (!colors.empty()) {
          it_colors += 3;
        }

        if (!scalars.empty()) {
          ++it_scalars;
        }

        if (!labels.empty()) {
          ++it_labels;
        }
      }
    }
    LOG_IF(WARNING, removed_points > 0)
        << "Removed " << removed_points << "/" << initial_number_of_points
        << " invalid points from point cloud!";

    CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
  }

  bool operator==(const PointCloud& other) const {
    bool is_same = xyz == other.xyz;
    is_same &= normals == other.normals;
    is_same &= colors == other.colors;
    is_same &= scalars == other.scalars;
    is_same &= labels == other.labels;
    return is_same;
  }

  inline void writeToFile(const std::string& file_path) const {
    CHECK(common::createPathToFile(file_path));

    std::filebuf filebuf;
    filebuf.open(file_path, std::ios::out | std::ios::binary);
    CHECK(filebuf.is_open());

    std::ostream output_stream(&filebuf);
    tinyply::PlyFile ply_file;

    // Const-casting is necessary as tinyply requires non-const access to the
    // vectors for reading.
    ply_file.add_properties_to_element(
        "vertex", {"x", "y", "z"}, const_cast<std::vector<float>&>(xyz));
    if (!normals.empty()) {
      ply_file.add_properties_to_element(
          "vertex", {"nx", "ny", "nz"},
          const_cast<std::vector<float>&>(normals));
    }
    if (!colors.empty()) {
      ply_file.add_properties_to_element(
          "vertex", {"red", "green", "blue"},
          const_cast<std::vector<unsigned char>&>(colors));
    }

    if (!scalars.empty()) {
      ply_file.add_properties_to_element(
          "vertex", {"scalar"}, const_cast<std::vector<float>&>(scalars));
    }

    if (!labels.empty()) {
      ply_file.add_properties_to_element(
          "vertex", {"label"}, const_cast<std::vector<uint32_t>&>(labels));
    }

    ply_file.comments.push_back("generated by tinyply from maplab");
    ply_file.write(output_stream, true);
    filebuf.close();
  }

  inline bool loadFromFile(const std::string& file_path) {
    if (!common::fileExists(file_path)) {
      VLOG(1) << "Point cloud file does not exist! Path: " << file_path;
      return false;
    }

    std::ifstream stream_ply(file_path);
    if (stream_ply.is_open()) {
      tinyply::PlyFile ply_file(stream_ply);
      const int xyz_point_count = ply_file.request_properties_from_element(
          "vertex", {"x", "y", "z"}, xyz);
      const int colors_count = ply_file.request_properties_from_element(
          "vertex", {"nx", "ny", "nz"}, normals);
      const int normals_count = ply_file.request_properties_from_element(
          "vertex", {"red", "green", "blue"}, colors);
      const int scalar_count = ply_file.request_properties_from_element(
          "vertex", {"scalar"}, scalars);
      const int label_count =
          ply_file.request_properties_from_element("vertex", {"label"}, labels);
      if (xyz_point_count > 0) {
        if (colors_count > 0) {
          // If colors are present, their count should match the point count.
          CHECK_EQ(xyz_point_count, colors_count);
        }
        if (normals_count > 0) {
          // If normals are present, their count should match the point count.
          CHECK_EQ(xyz_point_count, normals_count);
        }

        if (scalar_count > 0) {
          // If a value attribute is present, its count should match the point
          // count.
          CHECK_EQ(xyz_point_count, scalar_count);
        }

        if (label_count > 0) {
          // If a label attribute is present, its count should match the point
          // count.
          CHECK_EQ(xyz_point_count, label_count);
        }

        ply_file.read(stream_ply);
      }
      stream_ply.close();
      return true;
    }
    return false;
  }
};

}  // namespace resources

#endif  // RESOURCES_COMMON_POINT_CLOUD_H_
