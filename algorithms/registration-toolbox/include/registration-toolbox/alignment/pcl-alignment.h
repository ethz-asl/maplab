#ifndef REGISTRATION_TOOLBOX_ALIGNMENT_PCL_ALIGNMENT_H_
#define REGISTRATION_TOOLBOX_ALIGNMENT_PCL_ALIGNMENT_H_
#define PCL_NO_PRECOMPILE

#include <limits>

#include <glog/logging.h>
#include <map-resources/resource-conversion.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include "registration-toolbox/alignment/base-alignment.h"
#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;

template <typename T_alignment, typename T_point>
class PclAlignment : public BaseAlignment<resources::PointCloud> {
 public:
  PclAlignment();
  virtual ~PclAlignment() = default;

 protected:
  RegistrationResult registerCloudImpl(
      const resources::PointCloud& target, const resources::PointCloud& source,
      const aslam::Transformation& prior_T_target_source) override;

 private:
  T_alignment aligner_;
  pcl::VoxelGrid<T_point> voxel_grid_;
  void downSamplePointCloud(
      const PclPointCloudPtr<T_point>& cloud,
      const PclPointCloudPtr<T_point>& cloud_ds);
};

template <typename T_alignment, typename T_point>
PclAlignment<T_alignment, T_point>::PclAlignment() {
  CHECK_GT(FLAGS_regbox_pcl_downsample_leaf_size_m, 0);
  voxel_grid_.setLeafSize(
      FLAGS_regbox_pcl_downsample_leaf_size_m,
      FLAGS_regbox_pcl_downsample_leaf_size_m,
      FLAGS_regbox_pcl_downsample_leaf_size_m);

  CHECK_GT(FLAGS_regbox_pcl_max_iterations, 0);
  aligner_.setMaximumIterations(FLAGS_regbox_pcl_max_iterations);
}

template <typename T_alignment, typename T_point>
RegistrationResult PclAlignment<T_alignment, T_point>::registerCloudImpl(
    const resources::PointCloud& target, const resources::PointCloud& source,
    const aslam::Transformation& prior_T_target_source) {
  PclPointCloudPtr<T_point> target_pcl(new pcl::PointCloud<T_point>);
  PclPointCloudPtr<T_point> source_pcl(new pcl::PointCloud<T_point>);

  backend::convertPointCloudType(source, source_pcl.get());
  backend::convertPointCloudType(target, target_pcl.get());

  PclPointCloudPtr<T_point> target_aligner_input;
  PclPointCloudPtr<T_point> source_aligner_input;

  if (FLAGS_regbox_pcl_use_downsampling) {
    target_aligner_input =
        PclPointCloudPtr<T_point>(new pcl::PointCloud<T_point>);
    source_aligner_input =
        PclPointCloudPtr<T_point>(new pcl::PointCloud<T_point>);
    downSamplePointCloud(target, target_aligner_input);
    downSamplePointCloud(source, source_aligner_input);
  } else {
    target_aligner_input = target;
    source_aligner_input = source;
  }

  bool is_converged = true;
  PclPointCloudPtr<T_point> transformed(new pcl::PointCloud<T_point>);

  try {
    const Eigen::Matrix4f prior =
        prior_T_target_source.getTransformationMatrix().cast<float>();
    aligner_.setInputTarget(target_aligner_input);
    aligner_.setInputSource(source_aligner_input);
    aligner_.align(*transformed, prior);
  } catch (std::exception& e) {
    LOG(ERROR) << "PCL registration failed to align the point clouds.";
    VLOG(3) << "PCL registration error: " << e.what();
    is_converged = false;  // just a procaution.
  }

  CHECK_GT(FLAGS_regbox_pcl_fitness_max_considered_distance_m, 0);
  CHECK_GT(FLAGS_regbox_pcl_max_fitness_score_m, 0);
  const double fitness_score = aligner_.getFitnessScore(
      FLAGS_regbox_pcl_fitness_max_considered_distance_m);
  is_converged &= fitness_score <= FLAGS_regbox_pcl_max_fitness_score_m;
  const Eigen::Matrix4f T_f = aligner_.getFinalTransformation();
  const Eigen::Matrix4d T = T_f.cast<double>();
  const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 1e-2;

  if (T.topLeftCorner<3, 3>().normalized().squaredNorm() <=
      static_cast<double>(1) +
          kindr::minimal::EPS<double>::normalization_value()) {

            resources::PointCloud source_registered = source;
            const auto T_kindr = this->convertEigenToKindr(T.cast<double>());
            source_registered.applyTransformation(T_kindr);
            source_registered.applyTransformation(T_kindr);
    return RegistrationResult(
      source_registered, cov, T_kindr, aligner_.hasConverged() & is_converged);
  } else {
    LOG(ERROR)
        << "PCL registration gave invalid rotation matrix, returning prior.";
    return RegistrationResult(source, cov, prior_T_target_source, false);
  }
}

template <typename T_alignment, typename T_point>
void PclAlignment<T_alignment, T_point>::downSamplePointCloud(
    const PclPointCloudPtr<T_point>& cloud,
    const PclPointCloudPtr<T_point>& cloud_ds) {
  CHECK_NOTNULL(cloud);
  CHECK_NOTNULL(cloud_ds);

  const float inverse_grid_size = 1. / FLAGS_regbox_pcl_downsample_leaf_size_m;

  // Since the PCL VoxelGrid checks if all voxels inside the bounding box of the
  // point cloud fit into memory and otherwise skips downsampling, we have to
  // split the point cloud into multiple volumes that are small enough to fit.

  size_t volume_level = 1;

  T_point min_point;
  T_point max_point;
  pcl::getMinMax3D(*cloud, min_point, max_point);
  const std::int64_t dx = static_cast<std::int64_t>(
                              (max_point.x - min_point.x) * inverse_grid_size) +
                          1;
  const std::int64_t dy = static_cast<std::int64_t>(
                              (max_point.y - min_point.y) * inverse_grid_size) +
                          1;
  const std::int64_t dz = static_cast<std::int64_t>(
                              (max_point.z - min_point.z) * inverse_grid_size) +
                          1;
  const std::int64_t dxdydz = dx * dy * dz;
  // Find amount of levels that we have to split the pointcloud into
  bool indices_fit_in_voxel_grid = false;
  while (!indices_fit_in_voxel_grid) {
    if ((dxdydz / (volume_level * volume_level * volume_level)) <
        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
      indices_fit_in_voxel_grid = true;
    } else {
      volume_level++;
    }
  }

  for (size_t x_level = 0; x_level < volume_level; ++x_level) {
    const float min_x = min_point.x + (max_point.x - min_point.x) *
                                          static_cast<float>(x_level) /
                                          static_cast<float>(volume_level);
    const float max_x = min_point.x + (max_point.x - min_point.x) *
                                          static_cast<float>(x_level + 1) /
                                          static_cast<float>(volume_level);

    for (size_t y_level = 0; y_level < volume_level; ++y_level) {
      const float min_y = min_point.y + (max_point.y - min_point.y) *
                                            static_cast<float>(y_level) /
                                            static_cast<float>(volume_level);
      const float max_y = min_point.y + (max_point.y - min_point.y) *
                                            static_cast<float>(y_level + 1) /
                                            static_cast<float>(volume_level);
      for (size_t z_level = 0; z_level < volume_level; ++z_level) {
        const float min_z = min_point.z + (max_point.z - min_point.z) *
                                              static_cast<float>(z_level) /
                                              static_cast<float>(volume_level);

        const float max_z = min_point.z + (max_point.z - min_point.z) *
                                              static_cast<float>(z_level + 1) /
                                              static_cast<float>(volume_level);

        // Calculate borders of current subvolume
        pcl::PointXYZ current_volume_min(min_x, min_y, min_z);
        pcl::PointXYZ current_volume_max(max_x, max_y, max_z);

        // Filter the points in the respective subvolumes
        pcl::CropBox<T_point> box_filter;
        PclPointCloudPtr<T_point> points_in_current_volume(
            new pcl::PointCloud<T_point>);
        box_filter.setMin(Eigen::Vector4f(
            current_volume_min.x, current_volume_min.y, current_volume_min.z,
            1.0));
        box_filter.setMax(Eigen::Vector4f(
            current_volume_max.x, current_volume_max.y, current_volume_max.z,
            1.0));
        box_filter.setInputCloud(cloud);
        box_filter.filter(*points_in_current_volume);
        PclPointCloudPtr<T_point> filtered_cloud_in_current_volume(
            new pcl::PointCloud<T_point>);

        // Perform voxel grid downsampling on points inside that volume and add
        // to output pointcloud

        voxel_grid_.setInputCloud(points_in_current_volume);
        voxel_grid_.filter(*filtered_cloud_in_current_volume);
        *cloud_ds += *filtered_cloud_in_current_volume;
      }
    }
  }
}

using IcpAlignment = PclAlignment<
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>, pcl::PointXYZI>;
using GeneralizedIcpAlignment = PclAlignment<
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>,
    pcl::PointXYZI>;

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_ALIGNMENT_PCL_ALIGNMENT_H_
