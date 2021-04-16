#ifndef VI_MAP_LIDAR_LANDMARK_QUALITY_METRICS_H_
#define VI_MAP_LIDAR_LANDMARK_QUALITY_METRICS_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>

namespace vi_map {
class VIMap;
// TODO: mariusbr -> introduce lidar landmark
class Landmark;
}  // namespace vi_map

namespace vi_map {

struct LidarLandmarkWellConstrainedSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Maximum distance from closest observer for a LiDAR landmark to be well
  /// constrained [m].
  double max_distance_to_closest_observer_lidar;
  /// Minimum distance from closest observer for a LiDAR landmark to be well
  /// constrained [m].
  double min_distance_to_closest_observer_lidar;
  /// Minimum angle disparity of observers for a LiDAR landmark to be well
  /// constrained [deg].
  double min_observation_angle_deg_lidar;
  /// Minimum number of observers for a LiDAR landmark to be well constrained.
  size_t min_observers_lidar;
  /// Maximum distance between landmark measurements of a LiDAR [m].
  double max_position_deviation_lidar;
  /// Maximal ratio between the distance between landmark measurements and
  /// the distance from the observer to the closest measurement.
  double max_position_uncertainty_lidar;

  LidarLandmarkWellConstrainedSettings();
};

bool isLidarLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    const bool re_evaluate_quality);

inline bool isLidarLandmarkWellConstrained(
    const Aligned<std::vector, Eigen::Vector3d>& G_normalized_incidence_rays,
    const double signed_distance_to_closest_observer,
    const LidarLandmarkWellConstrainedSettings& settings);

// Includes an additional check for 3D LiDAR data
bool isLidarLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    const bool re_evaluate_quality, const double min_distance_to_lidar,
    const double position_uncertainty);

}  // namespace vi_map
#include "./lidar-landmark-quality-metrics-inl.h"
#endif  // VI_MAP_LIDAR_LANDMARK_QUALITY_METRICS_H_
