#ifndef VIO_COMMON_MAP_UPDATE_H_
#define VIO_COMMON_MAP_UPDATE_H_

#include <memory>
#include <vio-common/vio-types.h>

namespace vio {

struct MapUpdate {
  MAPLAB_POINTER_TYPEDEFS(MapUpdate);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Map update states.
  int64_t timestamp_ns;
  EstimatorState vio_state;
  UpdateType map_update_type;
  std::shared_ptr<const SynchronizedNFrame> keyframe;

  /// IMU measurements since the last nframe (including the last IMU measurement
  /// of the of the previous edge for integration).
  // TODO(nico): find a nice way of making this one const as well (ctor maybe)
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements;

  ViNodeState vinode;
  ViNodeCovariance vinode_covariance;

  // Localization update states.
  common::LocalizationState localization_state;
  aslam::Transformation T_G_M;

  // TODO(nico): find a way of checking imu validity as well
  // NOTE(mfehr): we need to make this check independent of whether there is a
  // visual keyframe or not if we want to support mapping without vision.
  inline bool check() const {
    return static_cast<bool>(keyframe);
  }
};

}  // namespace vio

#endif  // VIO_COMMON_MAP_UPDATE_H_
