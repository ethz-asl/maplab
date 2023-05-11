#include "resources-common/point-cloud.h"

#include <limits>
#include <numeric>
#include <unordered_map>

namespace resources {
void PointCloud::resize(
    const size_t size, const bool has_normals, const bool has_colors,
    const bool has_scalars, const bool has_labels, const bool has_times) {
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

  if (has_times) {
    times_ns.resize(1 * size);
  }
}

bool PointCloud::checkConsistency(const bool verbose) const {
  bool consistent = true;
  consistent &= (normals.size() == xyz.size()) || normals.empty();
  consistent &= (colors.size() == xyz.size()) || colors.empty();
  consistent &= (3u * scalars.size() == xyz.size()) || scalars.empty();
  consistent &= (3u * labels.size() == xyz.size()) || labels.empty();
  consistent &= (3u * times_ns.size() == xyz.size()) || times_ns.empty();

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

  // Remap to Eigen to leverage vectorization
  Eigen::Map<Eigen::Matrix3Xd> eigen_xyz(
      xyz.data() + 3 * start_point, 3, size() - start_point);
  eigen_xyz = T_A_B.transformVectorized(eigen_xyz);

  if (hasNormals()) {
    Eigen::Map<Eigen::Matrix3Xd> eigen_normals(
        normals.data() + 3 * start_point, 3, size() - start_point);
    eigen_normals = T_A_B.getRotation().rotateVectorized(eigen_normals);
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

void PointCloud::getMinMaxTimeNanoseconds(
    int32_t* min_time_ns, int32_t* max_time_ns) const {
  CHECK_NOTNULL(min_time_ns);
  CHECK_NOTNULL(max_time_ns);

  // Initialize to the opposite ends and iterate over point times.
  *min_time_ns = std::numeric_limits<int32_t>::max();
  *max_time_ns = std::numeric_limits<int32_t>::min();
  for (size_t i = 0; i < size(); ++i) {
    *min_time_ns = std::min(*min_time_ns, times_ns[i]);
    *max_time_ns = std::max(*max_time_ns, times_ns[i]);
  }
}

void PointCloud::undistort(
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps,
    const aslam::TransformationVector& poses) {
  CHECK_EQ(static_cast<size_t>(timestamps.cols()), poses.size());
  CHECK_GE(poses.size(), 2u) << "Need at least two poses to interpolate.";
  for (int64_t i = 0; i < timestamps.cols() - 1; ++i) {
    CHECK_LT(timestamps[i], timestamps[i + 1])
        << "Timestamps for poses should be strictly increasing to speed "
        << "up the interpolation.";
  }

  // Sort indeces of interal point based on timestamps. This speeds up
  // the search for the timestamps between which to interpolate.
  std::vector<size_t> point_idxs(times_ns.size());
  std::iota(point_idxs.begin(), point_idxs.end(), 0);
  std::stable_sort(
      point_idxs.begin(), point_idxs.end(),
      [this](size_t i1, size_t i2) { return times_ns[i1] < times_ns[i2]; });

  CHECK_GE(times_ns[point_idxs.front()], timestamps[0])
      << "Timestamps for intermediary poses do not cover the entire point "
         "cloud time range. Can't interpolate.";
  CHECK_LE(times_ns[point_idxs.back()], timestamps[timestamps.cols() - 1])
      << "Timestamps for intermediary poses do not cover the entire point "
         "cloud time range. Can't interpolate.";

  // Map the points to Eigen for vectorization
  Eigen::Map<Eigen::Matrix3Xd> eigen_xyz(xyz.data(), 3, size());

  // Initialize the two poses between which we interpolate. By sorting the
  // points by timestamp beforehand, we never need to go back afterward.
  int64_t intermediary_index = 1;
  int32_t time_A = timestamps[0];
  int32_t time_B = timestamps[1];
  aslam::Transformation pose_A = poses[0];
  aslam::Transformation pose_B = poses[1];

  for (size_t i = 0; i < size(); ++i) {
    const size_t point_idx = point_idxs[i];
    while (times_ns[point_idx] > time_B) {
      intermediary_index++;
      CHECK_LT(intermediary_index, poses.size());
      time_A = time_B;
      pose_A = pose_B;
      time_B = timestamps[intermediary_index];
      pose_B = poses[intermediary_index];
    }

    double lambda = static_cast<double>(times_ns[point_idx] - time_A) /
                    static_cast<double>(time_B - time_A);
    aslam::Transformation T =
        kindr::minimal::interpolateComponentwise(pose_A, pose_B, lambda);
    eigen_xyz.col(point_idx) = T.transform(eigen_xyz.col(point_idx));
  }
}

size_t PointCloud::filterValidMinMaxBox(
    double min_range_m, double max_range_m, const BoundingBox3D* box_filter) {
  std::vector<double> xyz_new;
  std::vector<double> normals_new;
  std::vector<unsigned char> colors_new;
  std::vector<float> scalars_new;
  std::vector<uint32_t> labels_new;
  std::vector<int32_t> times_ns_new;

  // We guess that we are on average not going to remove that many points.
  xyz_new.reserve(xyz.size());
  normals_new.reserve(normals.size());
  colors_new.reserve(colors.size());
  scalars_new.reserve(scalars.size());
  labels_new.reserve(labels.size());
  times_ns_new.reserve(times_ns.size());

  // Precompute min and max range squares to avoid having to sqrt
  double min_range_m2 = min_range_m * min_range_m;
  double max_range_m2 = max_range_m * max_range_m;

  size_t num_removed = 0;
  for (size_t i = 0; i < size(); ++i) {
    const size_t i0 = 3 * i;
    const size_t i1 = i0 + 1;
    const size_t i2 = i1 + 1;

    const double x = xyz[i0];
    const double y = xyz[i1];
    const double z = xyz[i2];

    // Check the values are valid
    if ((std::isinf(x) || std::isinf(y) || std::isinf(z)) ||
        (std::isnan(x) || std::isnan(y) || std::isnan(z))) {
      ++num_removed;
      continue;
    }

    // Check if the point is too close or far
    const double range2 = (x * x) + (y * y) + (z * z);
    if (range2 < min_range_m2 || range2 > max_range_m2) {
      ++num_removed;
      continue;
    }

    // Apply box filter if one was given
    if (box_filter != nullptr) {
      if ((x > box_filter->x_min && x < box_filter->x_max) &&
          (y > box_filter->y_min && y < box_filter->y_max) &&
          (z > box_filter->z_min && z < box_filter->z_max)) {
        ++num_removed;
        continue;
      }
    }

    xyz_new.emplace_back(x);
    xyz_new.emplace_back(y);
    xyz_new.emplace_back(z);

    if (hasNormals()) {
      normals_new.emplace_back(normals[i0]);
      normals_new.emplace_back(normals[i1]);
      normals_new.emplace_back(normals[i2]);
    }

    if (hasColor()) {
      colors_new.emplace_back(colors[i0]);
      colors_new.emplace_back(colors[i1]);
      colors_new.emplace_back(colors[i2]);
    }

    if (hasScalars()) {
      scalars_new.emplace_back(scalars[i]);
    }

    if (hasLabels()) {
      labels_new.emplace_back(labels[i]);
    }

    if (hasTimes()) {
      times_ns_new.emplace_back(times_ns[i]);
    }
  }

  xyz.swap(xyz_new);
  normals.swap(normals_new);
  colors.swap(colors_new);
  scalars.swap(scalars_new);
  labels.swap(labels_new);
  times_ns.swap(times_ns_new);

  CHECK(checkConsistency(true)) << "Point cloud is not consistent!";

  return num_removed;
}

void PointCloud::downsampleVoxelized(
    double voxel_size, PointCloud* voxelized) const {
  CHECK_GT(voxel_size, 1e-3) << "Voxel size is too small.";
  CHECK_NOTNULL(voxelized)->clear();

  // Map the points to Eigen for vectorization
  Eigen::Map<const Eigen::Matrix3Xd> eigen_xyz(xyz.data(), 3, size());

  // Assign points to voxels
  std::unordered_map<VoxelPosition, Voxel> voxel_grid;
  for (int i = 0; i < size(); ++i) {
    const Eigen::Vector3d point = eigen_xyz.col(i);
    VoxelPosition position(point, voxel_size);

    auto iter = voxel_grid.find(position);
    if (iter == voxel_grid.end()) {
      voxel_grid.emplace(position, point);
    } else {
      iter->second += point;
    }
  }

  // Assign memory and pull xyz points from voxel grid
  voxelized->xyz.resize(3 * voxel_grid.size());
  Eigen::Map<Eigen::Matrix3Xd> voxelized_xyz(
      voxelized->xyz.data(), 3, voxelized->size());

  int index = 0;
  for (auto iter = voxel_grid.begin(); iter != voxel_grid.end(); ++iter) {
    voxelized_xyz.col(index) = iter->second.xyz;
    ++index;
  }
}

void PointCloud::writeToFile(const std::string& file_path) const {
  CHECK(common::createPathToFile(file_path));

  std::filebuf filebuf;
  filebuf.open(file_path, std::ios::out | std::ios::binary);
  CHECK(filebuf.is_open());

  std::ostream output_stream(&filebuf);
  tinyply::PlyFile ply_file;

  // Const-casting is necessary as tinyply requires non-const access to the
  // vectors for reading. Additionally cast points and normals to float for
  // storage to save space.
  std::vector<float> xyz_float(xyz.begin(), xyz.end());
  ply_file.add_properties_to_element(
      "vertex", {"x", "y", "z"}, const_cast<std::vector<float>&>(xyz_float));

  if (!normals.empty()) {
    std::vector<float> normals_float(normals.begin(), normals.end());
    ply_file.add_properties_to_element(
        "vertex", {"nx", "ny", "nz"},
        const_cast<std::vector<float>&>(normals_float));
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
        "vertex", {"time_ns"}, const_cast<std::vector<int32_t>&>(times_ns));
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

    // Points are stored as floats, but then internally manipulated as doubles.
    std::vector<float> xyz_float;
    const int xyz_point_count = ply_file.request_properties_from_element(
        "vertex", {"x", "y", "z"}, xyz_float);

    // Normals are stored as floats, but then internally manipulated as doubles.
    std::vector<float> normals_float;
    const int normals_count = ply_file.request_properties_from_element(
        "vertex", {"nx", "ny", "nz"}, normals_float);

    const int colors_count = ply_file.request_properties_from_element(
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

      // Convert to internal representation.
      xyz.insert(xyz.end(), xyz_float.begin(), xyz_float.end());
      normals.insert(normals.end(), normals_float.begin(), normals_float.end());
    }

    stream_ply.close();
    CHECK(checkConsistency(true)) << "Point cloud is not consistent!";
    return true;
  }
  return false;
}
}  // namespace resources