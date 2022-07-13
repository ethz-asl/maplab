#ifndef VISUALIZATION_POINT_CLOUD_FILTER_H_
#define VISUALIZATION_POINT_CLOUD_FILTER_H_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <map-resources/resource-conversion.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace visualization {

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;

template <typename T_input, typename T_output>
static void applyRandomDownSamplingFilter(
    const T_input& cloud_in, const size_t& n_points_to_keep,
    T_output* cloud_filtered) {
  CHECK_NOTNULL(cloud_filtered);
  CHECK_GE(cloud_in.size(), n_points_to_keep);
  const bool input_has_normals = backend::hasNormalsInformation(cloud_in);
  const bool input_has_scalars = backend::hasScalarInformation(cloud_in);
  const bool input_has_color = backend::hasColorInformation(cloud_in);
  const bool input_has_labels = backend::hasLabelInformation(cloud_in);

  backend::resizePointCloud(
      n_points_to_keep, input_has_color, input_has_normals, input_has_scalars,
      input_has_labels, cloud_filtered);

  const bool output_has_scalars =
      backend::hasScalarInformation(*cloud_filtered);
  const bool output_has_color = backend::hasColorInformation(*cloud_filtered);
  const bool output_has_labels = backend::hasLabelInformation(*cloud_filtered);

  std::vector<size_t> sampling_indices(cloud_in.size());
  std::iota(sampling_indices.begin(), sampling_indices.end(), 0u);
  std::random_shuffle(sampling_indices.begin(), sampling_indices.end());
  for (size_t idx = 0u; idx < n_points_to_keep; ++idx) {
    Eigen::Vector3d point_C;
    backend::getPointFromPointCloud(cloud_in, sampling_indices[idx], &point_C);
    backend::addPointToPointCloud(point_C, idx, cloud_filtered);

    if (input_has_color && output_has_color) {
      resources::RgbaColor color;
      backend::getColorFromPointCloud(cloud_in, sampling_indices[idx], &color);
      backend::addColorToPointCloud(color, idx, cloud_filtered);
    }

    if (input_has_scalars && output_has_scalars) {
      float scalar;
      backend::getScalarFromPointCloud(
          cloud_in, sampling_indices[idx], &scalar);
      backend::addScalarToPointCloud(scalar, idx, cloud_filtered);
    }

    if (input_has_labels && output_has_labels) {
      uint32_t label;
      backend::getLabelFromPointCloud(cloud_in, sampling_indices[idx], &label);
      backend::addLabelToPointCloud(label, idx, cloud_filtered);
    }
  }
}

}  // namespace visualization

#endif  // VISUALIZATION_POINT_CLOUD_FILTER_H_
