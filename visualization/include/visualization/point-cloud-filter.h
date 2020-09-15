#ifndef VISUALIZATION_POINT_CLOUD_FILTER_H_
#define VISUALIZATION_POINT_CLOUD_FILTER_H_

#include <pcl/filters/voxel_grid.h>

namespace visualization {

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;

class PointCloudFilter {
 public:
  explicit PointCloudFilter(const float vg_leaf_size_m);
  template <typename T_point>
  void filterCloud(PclPointCloudPtr<T_point> cloud);

 private:
  const float vg_leaf_size_m_;
};

template <typename T_point>
void PointCloudFilter::filterCloud(PclPointCloudPtr<T_point> cloud) {
  CHECK_NOTNULL(cloud);
  pcl::VoxelGrid<T_point> voxel_grid_;
  voxel_grid_.setLeafSize(vg_leaf_size_m_, vg_leaf_size_m_, vg_leaf_size_m_);
  voxel_grid_.setInputCloud(cloud);
  voxel_grid_.filter(*cloud);
}

}  // namespace visualization

#endif  // VISUALIZATION_POINT_CLOUD_FILTER_H_
