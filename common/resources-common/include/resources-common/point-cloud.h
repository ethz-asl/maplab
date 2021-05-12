#ifndef RESOURCES_COMMON_POINT_CLOUD_H_
#define RESOURCES_COMMON_POINT_CLOUD_H_

#include <cstdio>
#include <fstream>  // NOLINT
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/threading-helpers.h>

#include "resources-common/resources-gflags.h"
#include "resources-common/tinyply/tinyply.h"

namespace resources {

typedef Eigen::Matrix<uint8_t, 4, 1> RgbaColor;

static const std::string kPointCloudSuffix = ".ply";
static const std::string kCompressedPointCloudSuffix = ".drc";

struct PointCloud {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<float> xyz;
  std::vector<float> normals;
  std::vector<unsigned char> colors;
  std::vector<float> scalars;
  std::vector<uint32_t> labels;
  std::vector<uint16_t> rings;
  std::vector<float> times;

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

          // Rotate normals if present.
          if (normals.size() == xyz.size()) {
            const aslam::Quaternion& R_A_B = T_A_B.getRotation();
            for (size_t point_idx : batch) {
              const size_t idx = point_idx * 3;
              const Eigen::Vector3d normal(
                  normals[idx], normals[idx + 1u], normals[idx + 2u]);
              const Eigen::Vector3f& transformed_normal =
                  R_A_B.rotate(normal).cast<float>();
              normals[idx] = transformed_normal.x();
              normals[idx + 1u] = transformed_normal.y();
              normals[idx + 2u] = transformed_normal.z();
            }
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
      const bool has_labels = true, const bool has_rings = true,
      const bool has_times = true) {
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

    if (has_rings) {
      rings.resize(1 * size);
    }

    if (has_times) {
      times.resize(1 * size);
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

  inline bool hasRings() const {
    return (rings.size() == xyz.size() / 3u) && !rings.empty();
  }

  inline bool hasTimes() const {
    return (times.size() == xyz.size() / 3u) && !times.empty();
  }

  inline bool checkConsistency(const bool verbose = false) const {
    bool consistent = true;
    consistent &= (normals.size() == xyz.size()) || normals.empty();
    consistent &= (colors.size() == xyz.size()) || colors.empty();
    consistent &= (scalars.size() == xyz.size() / 3u) || scalars.empty();
    consistent &= (labels.size() == xyz.size() / 3u) || labels.empty();
    consistent &= (rings.size() == xyz.size() / 3u) || rings.empty();
    consistent &= (times.size() == xyz.size() / 3u) || times.empty();

    LOG_IF(ERROR, verbose && !consistent)
        << "\nInconsistent point cloud:"
        << "\n - Point vector size:  " << xyz.size()
        << "\n - Normal vector size: " << normals.size()
        << "\n - Color vector size:  " << colors.size()
        << "\n - Scalar vector size: " << scalars.size()
        << "\n - Label vector size: " << labels.size()
        << "\n - Ring vector size: " << rings.size()
        << "\n - Times vector size: " << times.size();
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
    rings.reserve(rings.size() + other.rings.size());
    times.reserve(times.size() + other.times.size());

    xyz.insert(xyz.end(), other.xyz.begin(), other.xyz.end());
    normals.insert(normals.end(), other.normals.begin(), other.normals.end());
    colors.insert(colors.end(), other.colors.begin(), other.colors.end());
    scalars.insert(scalars.end(), other.scalars.begin(), other.scalars.end());
    labels.insert(labels.end(), other.labels.begin(), other.labels.end());
    rings.insert(rings.end(), other.rings.begin(), other.rings.end());
    times.insert(times.end(), other.times.begin(), other.times.end());

    CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
  }

  inline void append(const std::vector<PointCloud>& others) {
    if (others.empty()) {
      return;
    }

    size_t n_points = 0u, n_normals = 0u, n_colors = 0u, n_scalars = 0u,
           n_labels = 0u, n_rings = 0u, n_times = 0u;
    for (const PointCloud& other : others) {
      CHECK(other.checkConsistency(true));
      n_points += other.xyz.size();
      n_normals += other.normals.size();
      n_colors += other.colors.size();
      n_scalars += other.scalars.size();
      n_labels += other.labels.size();
      n_rings += other.rings.size();
      n_times += other.times.size();
    }

    xyz.reserve(xyz.size() + n_points);
    normals.reserve(normals.size() + n_normals);
    colors.reserve(colors.size() + n_colors);
    scalars.reserve(scalars.size() + n_scalars);
    labels.reserve(labels.size() + n_labels);
    rings.reserve(rings.size() + n_rings);
    times.reserve(times.size() + n_times);

    for (const PointCloud& other : others) {
      if (other.empty()) {
        return;
      }
      xyz.insert(xyz.end(), other.xyz.begin(), other.xyz.end());
      normals.insert(normals.end(), other.normals.begin(), other.normals.end());
      colors.insert(colors.end(), other.colors.begin(), other.colors.end());
      scalars.insert(scalars.end(), other.scalars.begin(), other.scalars.end());
      labels.insert(labels.end(), other.labels.begin(), other.labels.end());
      rings.insert(rings.end(), other.rings.begin(), other.rings.end());
      times.insert(times.end(), other.times.begin(), other.times.end());
    }
    CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
  }

  // Transforms the other point cloud and appends it to the current one. Assumes
  // the other point cloud is in B frame.
  inline void appendTransformed(
      const PointCloud& other, const aslam::Transformation& T_A_B) {
    if (other.empty()) {
      return;
    }

    // Remember how many points there wrere before.
    const size_t old_size = size();
    append(other);
    const size_t new_size = size();

    for (size_t point_idx = old_size; point_idx < new_size; ++point_idx) {
      const size_t idx = point_idx * 3;
      const Eigen::Vector3d point(xyz[idx], xyz[idx + 1u], xyz[idx + 2u]);
      const Eigen::Vector3f& transformed_point = (T_A_B * point).cast<float>();
      xyz[idx] = transformed_point.x();
      xyz[idx + 1u] = transformed_point.y();
      xyz[idx + 2u] = transformed_point.z();
    }

    // Rotate normals if present.
    if (normals.size() == xyz.size()) {
      const aslam::Quaternion& R_A_B = T_A_B.getRotation();
      for (size_t point_idx = old_size; point_idx < new_size; ++point_idx) {
        const size_t idx = point_idx * 3;
        const Eigen::Vector3d normal(
            normals[idx], normals[idx + 1u], normals[idx + 2u]);
        const Eigen::Vector3f& transformed_normal =
            R_A_B.rotate(normal).cast<float>();
        normals[idx] = transformed_normal.x();
        normals[idx + 1u] = transformed_normal.y();
        normals[idx + 2u] = transformed_normal.z();
      }
    }

    CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
  }

  inline void removeInvalidPoints() {
    auto it_xyz = xyz.begin();
    auto it_normals = normals.begin();
    auto it_colors = colors.begin();
    auto it_scalars = scalars.begin();
    auto it_labels = labels.begin();
    auto it_rings = rings.begin();
    auto it_times = times.begin();

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

        if (!rings.empty()) {
          it_rings = rings.erase(it_rings);
        }

        if (!times.empty()) {
          it_times = times.erase(it_times);
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

        if (!rings.empty()) {
          ++it_rings;
        }

        if (!times.empty()) {
          ++it_times;
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
    is_same &= rings == other.rings;
    is_same &= times == other.times;
    return is_same;
  }

  void writeToFileCompressed(const std::string& file_path) const;

  inline void writeToFile(const std::string& file_path) const {
    CHECK(common::createPathToFile(file_path));

    if (FLAGS_resources_compress_pointclouds) {
      writeToFileCompressed(file_path);
      return;
    }

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

    if (!rings.empty()) {
      ply_file.add_properties_to_element(
          "vertex", {"ring"}, const_cast<std::vector<uint16_t>&>(rings));
    }

    if (!times.empty()) {
      ply_file.add_properties_to_element(
          "vertex", {"time"}, const_cast<std::vector<float>&>(times));
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
    CHECK_GE(file_path.size(), 4);
    const std::string suffix = file_path.substr(file_path.size() - 4);
    if (suffix == kCompressedPointCloudSuffix) {
      return loadFromCompressedFile(file_path);
    } else {
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
        const int label_count = ply_file.request_properties_from_element(
            "vertex", {"label"}, labels);
        const int ring_count =
            ply_file.request_properties_from_element("vertex", {"ring"}, rings);
        const int time_count =
            ply_file.request_properties_from_element("vertex", {"time"}, times);
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

          if (ring_count > 0) {
            // If a ring attribute is present, its count should match the point
            // count.
            CHECK_EQ(xyz_point_count, ring_count);
          }

          if (time_count > 0) {
            // If a time attribute is present, its count should match the point
            // count.
            CHECK_EQ(xyz_point_count, time_count);
          }

          ply_file.read(stream_ply);
        }
        stream_ply.close();
        return true;
      }
      return false;
    }
  }

  bool loadFromCompressedFile(const std::string& file_path);

  inline bool colorizePointCloud(
      const size_t start_point_idx, const size_t end_point_idx, const uint8_t r,
      const uint8_t g, const uint8_t b) {
    if (colors.size() != xyz.size()) {
      return false;
    }
    CHECK_LT(end_point_idx, size());
    CHECK_LE(start_point_idx, end_point_idx);

    auto it_rgb = colors.begin() + start_point_idx;
    auto end_it = colors.begin() + end_point_idx;
    while (it_rgb != end_it && it_rgb != colors.end()) {
      *it_rgb = r;
      *(it_rgb + 1) = g;
      *(it_rgb + 2) = b;
      it_rgb += 3;
    }

    return true;
  }

  inline bool colorizePointCloud(
      const uint8_t r, const uint8_t g, const uint8_t b) {
    return colorizePointCloud(0, size(), r, g, b);
  }
};

}  // namespace resources

#endif  // RESOURCES_COMMON_POINT_CLOUD_H_
