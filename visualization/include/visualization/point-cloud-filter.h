#ifndef VISUALIZATION_POINT_CLOUD_FILTER_H_
#define VISUALIZATION_POINT_CLOUD_FILTER_H_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <map-resources/resource-conversion.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>

namespace visualization {

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;

template <typename T_point>
void voxelGridPointCloud(
    const float vg_leaf_size_m, PclPointCloudPtr<T_point> cloud) {
  CHECK_NOTNULL(cloud);
  CHECK_GT(vg_leaf_size_m, 0.0f);
  pcl::VoxelGrid<T_point> voxel_grid_;
  voxel_grid_.setLeafSize(vg_leaf_size_m, vg_leaf_size_m, vg_leaf_size_m);
  voxel_grid_.setInputCloud(cloud);
  voxel_grid_.filter(*cloud);
}

template <typename T_point>
void beautifyPointCloud(
    PclPointCloudPtr<T_point> cloud, sensor_msgs::PointCloud2* cloud_msg) {
  CHECK_NOTNULL(cloud_msg);
  pcl::toROSMsg(*cloud, *cloud_msg);
  PointMatcher<double>::DataPoints cloud_points =
      PointMatcher_ros::rosMsgToPointMatcherCloud<double>(*cloud_msg);
  const std::string input_filter_file =
      ros::package::getPath("visualization") + "/cfg/input-filter-for-viz.yaml";
  std::ifstream input_filter_config(input_filter_file);
  std::unique_ptr<PointMatcher<double>::DataPointsFilters> input_filters;
  input_filters.reset(
      new PointMatcher<double>::DataPointsFilters(input_filter_config));
  input_filters->apply(cloud_points);

  const size_t n_points = cloud_points.getNbPoints();
  cloud->clear();
  cloud->resize(n_points);
  for (std::size_t idx = 0u; idx < n_points; ++idx) {
    cloud->points[idx].getVector3fMap() =
        cloud_points.features.col(idx).head<3>().cast<float>();
  }
}

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
  const bool input_has_rings = backend::hasRingInformation(cloud_in);
  const bool input_has_times = backend::hasTimeInformation(cloud_in);

  backend::resizePointCloud(
      n_points_to_keep, input_has_color, input_has_normals, input_has_scalars,
      input_has_labels, input_has_rings, input_has_times, cloud_filtered);

  const bool output_has_normals =
      backend::hasNormalsInformation(*cloud_filtered);
  const bool output_has_scalars =
      backend::hasScalarInformation(*cloud_filtered);
  const bool output_has_color = backend::hasColorInformation(*cloud_filtered);
  const bool output_has_labels = backend::hasLabelInformation(*cloud_filtered);
  const bool output_has_rings = backend::hasRingInformation(*cloud_filtered);
  const bool output_has_times = backend::hasTimeInformation(*cloud_filtered);

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

    if (input_has_rings && output_has_rings) {
      uint32_t ring;
      backend::getRingFromPointCloud(cloud_in, sampling_indices[idx], &ring);
      backend::addRingToPointCloud(ring, idx, cloud_filtered);
    }

    if (input_has_times && output_has_times) {
      float time_s;
      backend::getTimeFromPointCloud(cloud_in, sampling_indices[idx], &time_s);
      backend::addTimeToPointCloud(time_s, idx, cloud_filtered);
    }
  }
}

}  // namespace visualization

#endif  // VISUALIZATION_POINT_CLOUD_FILTER_H_
