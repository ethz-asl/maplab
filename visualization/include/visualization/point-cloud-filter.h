#ifndef VISUALIZATION_POINT_CLOUD_FILTER_H_
#define VISUALIZATION_POINT_CLOUD_FILTER_H_

#include <pcl/filters/voxel_grid.h>

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

}  // namespace visualization

#endif  // VISUALIZATION_POINT_CLOUD_FILTER_H_
