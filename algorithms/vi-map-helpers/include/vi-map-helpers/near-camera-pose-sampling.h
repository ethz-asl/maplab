#ifndef VI_MAP_HELPERS_NEAR_CAMERA_POSE_SAMPLING_H_
#define VI_MAP_HELPERS_NEAR_CAMERA_POSE_SAMPLING_H_

#include <Eigen/Dense>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace vi_map_helpers {

class NearCameraPoseSampling {
 public:
  struct Options {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    size_t camera_index;
    Eigen::Vector2d x_y_spacing_m;
    size_t num_yaw_samples;
    double radius_m;
  };

  typedef Aligned<std::vector, aslam::Transformation> PoseVector;
  typedef PoseVector::const_iterator const_iterator;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NearCameraPoseSampling(const vi_map::VIMap& map, const Options& options);

  inline const_iterator begin() const {
    return T_G_C_samples_.begin();
  }

  inline const_iterator end() const {
    return T_G_C_samples_.end();
  }

  inline size_t size() const {
    return T_G_C_samples_.size();
  }

 private:
  typedef AlignedUnorderedSet<Eigen::Vector2i> PlanarGrid;
  void gridOccupationFrom_p_G_C(
      const Eigen::Matrix3Xd& p_G_C, PlanarGrid* result) const;
  void populateFromGrid(const PlanarGrid& grid, const double average_height_m);

  const Options options_;
  const double radius2_m2_;
  Eigen::Matrix2Xi grid_offsets_around_camera_pose_;
  PoseVector T_G_C_samples_;
};

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_NEAR_CAMERA_POSE_SAMPLING_H_
