#ifndef VI_MAP_LANDMARK_QUALITY_METRICS_INL_H_
#define VI_MAP_LANDMARK_QUALITY_METRICS_INL_H_

#include <algorithm>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/statistics/statistics.h>
#include <maplab-common/conversions.h>

namespace vi_map {

inline bool isLandmarkWellConstrained(
    const Aligned<std::vector, Eigen::Vector3d>& G_normalized_incidence_rays,
    double signed_distance_to_closest_observer,
    const LandmarkWellConstrainedSettings& settings) {
  const size_t num_observations = G_normalized_incidence_rays.size();
  if (num_observations < settings.min_observers) {
    return false;
  }

  if (signed_distance_to_closest_observer >
          settings.max_distance_to_closest_observer ||
      signed_distance_to_closest_observer <
          settings.min_distance_to_closest_observer) {
    statistics::StatsCollector stats("Landmark too close or too far away.");
    stats.AddSample(signed_distance_to_closest_observer);
    return false;
  }

  double min_cos_angle = 1.0;
  for (size_t i = 0; i < G_normalized_incidence_rays.size(); ++i) {
    for (size_t j = i + 1; j < G_normalized_incidence_rays.size(); ++j) {
      min_cos_angle = std::min(
          min_cos_angle, std::abs(
                             G_normalized_incidence_rays[i].dot(
                                 G_normalized_incidence_rays[j])));
    }
  }

  constexpr double kRadToDeg = 180.0 / M_PI;
  double angle_deg = acos(min_cos_angle) * kRadToDeg;
  bool quality_good = angle_deg >= settings.min_observation_angle_deg;
  if (!quality_good) {
    statistics::StatsCollector stats(
        "Landmark observations angles too close together.");
    stats.IncrementOne();
  }
  return quality_good;
}
}  // namespace vi_map

#endif  // VI_MAP_LANDMARK_QUALITY_METRICS_INL_H_
