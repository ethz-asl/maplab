#ifndef VI_MAP_LANDMARK_QUALITY_METRICS_H_
#define VI_MAP_LANDMARK_QUALITY_METRICS_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <vi-map/vi-map.h>

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
  /// Maximum reprojection error for any observation of the landmark.
  /// When negative this check will be ignored.
  double max_reprojection_error_px;
  /// Minimum number of observers for a landmark to be well constrained.
  size_t min_observers;

  LandmarkWellConstrainedSettings();
};

double computeSquaredReprojectionError(
    const Vertex& vertex, const int frame_idx, const int keypoint_idx,
    const Eigen::Vector3d& landmark_p_C);

bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark);

// Re-evaluates the quality even if the landmark quality is known if the
// parameter re_evaluate_quality is set to true.
bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    bool re_evaluate_quality);

}  // namespace vi_map

#endif  // VI_MAP_LANDMARK_QUALITY_METRICS_H_
