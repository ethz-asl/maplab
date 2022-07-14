#define PCL_NO_PRECOMPILE

#include "registration-toolbox/alignment/pcl-alignment.h"

namespace regbox {

template <>
PclAlignment<
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>,
    pcl::PointXYZI>::PclAlignment() {
  CHECK_GT(FLAGS_regbox_pcl_downsample_leaf_size_m, 0);
  voxel_grid_.setLeafSize(
      FLAGS_regbox_pcl_downsample_leaf_size_m,
      FLAGS_regbox_pcl_downsample_leaf_size_m,
      FLAGS_regbox_pcl_downsample_leaf_size_m);

  CHECK_GT(FLAGS_regbox_pcl_max_iterations, 0);
  aligner_.setMaximumIterations(FLAGS_regbox_pcl_max_iterations);

  CHECK_GT(FLAGS_regbox_pcl_gicp_n_neighbors, 0);
  aligner_.setCorrespondenceRandomness(FLAGS_regbox_pcl_gicp_n_neighbors);
}
}  // namespace regbox
