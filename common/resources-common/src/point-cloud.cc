#include "resources-common/point-cloud.h"

namespace resources {
void PointCloud::resize(
    const size_t size, const bool has_normals, const bool has_colors,
    const bool has_scalars, const bool has_labels, const bool has_times_ns) {
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

  if (has_times_ns) {
    times_ns.resize(1 * size);
  }
}

bool PointCloud::checkConsistency(const bool verbose) const {
  bool consistent = true;
  consistent &= (normals.size() == xyz.size()) || normals.empty();
  consistent &= (colors.size() == xyz.size()) || colors.empty();
  consistent &= (scalars.size() == xyz.size() / 3u) || scalars.empty();
  consistent &= (labels.size() == xyz.size() / 3u) || labels.empty();
  consistent &= (times_ns.size() == xyz.size() / 3u) || times_ns.empty();

  LOG_IF(ERROR, verbose && !consistent)
      << "\nInconsistent point cloud:"
      << "\n - Point vector size:  " << xyz.size()
      << "\n - Normal vector size: " << normals.size()
      << "\n - Color vector size:  " << colors.size()
      << "\n - Scalar vector size: " << scalars.size()
      << "\n - Label vector size: " << labels.size()
      << "\n - Times (ns) vector size: " << times_ns.size();
  return consistent;
}

void PointCloud::append(const PointCloud& other) {
  if (other.empty()) {
    return;
  }

  xyz.reserve(xyz.size() + other.xyz.size());
  normals.reserve(normals.size() + other.normals.size());
  colors.reserve(colors.size() + other.colors.size());
  scalars.reserve(scalars.size() + other.scalars.size());
  labels.reserve(labels.size() + other.labels.size());
  times_ns.reserve(times_ns.size() + other.times_ns.size());

  xyz.insert(xyz.end(), other.xyz.begin(), other.xyz.end());
  normals.insert(normals.end(), other.normals.begin(), other.normals.end());
  colors.insert(colors.end(), other.colors.begin(), other.colors.end());
  scalars.insert(scalars.end(), other.scalars.begin(), other.scalars.end());
  labels.insert(labels.end(), other.labels.begin(), other.labels.end());
  times_ns.insert(times_ns.end(), other.times_ns.begin(), other.times_ns.end());

  CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
}

void PointCloud::applyTransformation(
    const aslam::Transformation& T_A_B, size_t start_point) {
  if (size() == 0) {
    return;
  }

  CHECK_LT(start_point, size());

  // Convert the transformation matrix from double to float, since the point
  // coordinates and normals are stored in float vectors
  kindr::minimal::QuatTransformationTemplate<float> T_A_B_float =
      T_A_B.cast<float>();

  // Remap to Eigen to avoid copies and leverage vectorization
  Eigen::Map<Eigen::Matrix3Xf> eigen_xyz(
      xyz.data() + 3 * start_point, 3, size() - start_point);
  eigen_xyz = T_A_B_float.transformVectorized(eigen_xyz);

  if (hasNormals()) {
    Eigen::Map<Eigen::Matrix3Xf> eigen_normals(
        normals.data() + 3 * start_point, 3, size() - start_point);
    eigen_normals = T_A_B_float.getRotation().rotateVectorized(eigen_normals);
  }
}

void PointCloud::appendTransformed(
    const PointCloud& other, const aslam::Transformation& T_A_B) {
  if (other.empty()) {
    return;
  }

  append(other);
  // Apply the transformation only starting from the first appended point
  applyTransformation(T_A_B, size() - other.size());
}

void PointCloud::removeInvalidPoints() {
  auto it_xyz = xyz.begin();
  auto it_normals = normals.begin();
  auto it_colors = colors.begin();
  auto it_scalars = scalars.begin();
  auto it_labels = labels.begin();
  auto it_times_ns = times_ns.begin();

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

      if (!times_ns.empty()) {
        it_times_ns = times_ns.erase(it_times_ns);
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

      if (!times_ns.empty()) {
        ++it_times_ns;
      }
    }
  }
  LOG_IF(WARNING, removed_points > 0)
      << "Removed " << removed_points << "/" << initial_number_of_points
      << " invalid points from point cloud!";

  CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
}

void PointCloud::writeToFile(const std::string& file_path) const {
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
        "vertex", {"nx", "ny", "nz"}, const_cast<std::vector<float>&>(normals));
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

  if (!times_ns.empty()) {
    ply_file.add_properties_to_element(
        "vertex", {"time_ns"}, const_cast<std::vector<uint32_t>&>(times_ns));
  }

  ply_file.comments.push_back("generated by tinyply from maplab");
  ply_file.write(output_stream, true);
  filebuf.close();
}

bool PointCloud::loadFromFile(const std::string& file_path) {
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
    const int scalars_count =
        ply_file.request_properties_from_element("vertex", {"scalar"}, scalars);
    const int labels_count =
        ply_file.request_properties_from_element("vertex", {"label"}, labels);
    const int times_ns_count = ply_file.request_properties_from_element(
        "vertex", {"time_ns"}, times_ns);
    if (xyz_point_count > 0) {
      if (colors_count > 0) {
        CHECK_EQ(xyz_point_count, colors_count);
      }
      if (normals_count > 0) {
        CHECK_EQ(xyz_point_count, normals_count);
      }

      if (scalars_count > 0) {
        CHECK_EQ(xyz_point_count, scalars_count);
      }

      if (labels_count > 0) {
        CHECK_EQ(xyz_point_count, labels_count);
      }

      if (times_ns_count > 0) {
        CHECK_EQ(xyz_point_count, times_ns_count);
      }

      ply_file.read(stream_ply);
    }
    stream_ply.close();
    return true;
  }
  return false;
}
}  // namespace resources