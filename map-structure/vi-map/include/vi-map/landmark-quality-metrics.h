#ifndef VI_MAP_LANDMARK_QUALITY_METRICS_H_
#define VI_MAP_LANDMARK_QUALITY_METRICS_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>

namespace vi_map {
class VIMap;
class Landmark;
}

namespace vi_map {

struct LandmarkWellConstrainedSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Maximum distance from closest observer for a landmark to be well
  /// constrained [m].
  double max_distance_to_closest_observer;
  /// Minimum distance from closest observer for a landmark to be well
  /// constrained [m].
  double min_distance_to_closest_observer;
  /// Minimum angle disparity of observers for a landmark to be well constrained
  /// [deg].
  double min_observation_angle_deg;
  /// Minimum number of observers for a landmark to be well constrained.
  size_t min_observers;

  LandmarkWellConstrainedSettings();
};

// These methods use the number of observers, the incident rays from the
// observers as well as the distance to the camera to determine the landmark
// quality.
bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark);

inline bool isLandmarkWellConstrained(
    const Aligned<std::vector, Eigen::Vector3d>& G_normalized_incidence_rays,
    double signed_distance_to_closest_observer,
    const LandmarkWellConstrainedSettings& settings);

// Re-evaluates the quality even if the landmark quality is known if the
// parameter re_evaluate_quality is set to true.
bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    bool re_evaluate_quality);

}  // namespace vi_map
#include "./landmark-quality-metrics-inl.h"
#endif  // VI_MAP_LANDMARK_QUALITY_METRICS_H_
