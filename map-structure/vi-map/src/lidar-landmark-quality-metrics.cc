#include "vi-map/lidar-landmark-quality-metrics.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/statistics/statistics.h>
#include <gflags/gflags.h>
#include <vi-map/landmark.h>
#include <vi-map/vi-map.h>

DEFINE_double(
    vi_map_landmark_quality_min_observation_angle_deg_lidar, 0.0,
    "Minimum angle disparity of observers for a LiDAR landmark to be "
    "well constrained.");

DEFINE_uint64(
    vi_map_landmark_quality_min_observers_lidar, 2,
    "Minimum number of observers for a LiDAR landmark to be "
    "well constrained.");

DEFINE_double(
    vi_map_landmark_quality_max_distance_from_closest_observer_lidar, 60,
    "Maximum distance from closest observer for a LiDAR landmark to be "
    "well constrained [m].");

DEFINE_double(
    vi_map_landmark_quality_min_distance_from_closest_observer_lidar, 1.5,
    "Minimum distance from closest observer for a LiDAR landmark to be "
    "well constrained [m].");

DEFINE_double(
    vi_map_landmark_quality_max_position_deviation_lidar, 4.0,
    "Maximum distance between landmark measurements of a LiDAR [m].");

DEFINE_double(
    vi_map_landmark_quality_max_position_uncertainty_lidar, 0.1,
    "Maximal ratio between the distance between LiDAR landmark measurements "
    "and the distance from the observer to the closest measurement.");

namespace vi_map {
LidarLandmarkWellConstrainedSettings::LidarLandmarkWellConstrainedSettings()
    : max_distance_to_closest_observer_lidar(
          FLAGS_vi_map_landmark_quality_max_distance_from_closest_observer_lidar),
      min_distance_to_closest_observer_lidar(
          FLAGS_vi_map_landmark_quality_min_distance_from_closest_observer_lidar),
      min_observation_angle_deg_lidar(
          FLAGS_vi_map_landmark_quality_min_observation_angle_deg_lidar),
      min_observers_lidar(FLAGS_vi_map_landmark_quality_min_observers_lidar),
      max_position_deviation_lidar(
          FLAGS_vi_map_landmark_quality_max_position_deviation_lidar),
      max_position_uncertainty_lidar(
          FLAGS_vi_map_landmark_quality_max_position_uncertainty_lidar) {}

bool isLidarLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark) {
  constexpr bool kReEvaluateQuality = false;
  return isLidarLandmarkWellConstrained(map, landmark, kReEvaluateQuality);
}

bool isLidarLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    const bool re_evaluate_quality, const double min_distance_to_lidar,
    const double position_uncertainty) {
  LidarLandmarkWellConstrainedSettings settings;
  if (position_uncertainty / min_distance_to_lidar >
          settings.max_position_uncertainty_lidar ||
      position_uncertainty > settings.max_position_deviation_lidar) {
    return false;
  }
  return isLidarLandmarkWellConstrained(map, landmark, re_evaluate_quality);
}

bool isLidarLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    const bool re_evaluate_quality) {
  vi_map::Landmark::Quality quality = landmark.getQuality();
  // Localization landmarks are always good.
  if (quality == vi_map::Landmark::Quality::kLocalizationSummaryLandmark) {
    return true;
  }
  if (re_evaluate_quality) {
    quality = vi_map::Landmark::Quality::kUnknown;
  } else if (quality != vi_map::Landmark::Quality::kUnknown) {
    return quality == vi_map::Landmark::Quality::kGood;
  }

  // Use default settings.
  LidarLandmarkWellConstrainedSettings settings;

  const vi_map::KeypointIdentifierList& backlinks = landmark.getObservations();
  if (backlinks.size() < settings.min_observers_lidar) {
    statistics::StatsCollector stats("LiDAR Landmark has too few backlinks");
    stats.IncrementOne();
    return false;
  }

  const Eigen::Vector3d& p_G_fi = map.getLandmark_G_p_fi(landmark.id());
  Aligned<std::vector, Eigen::Vector3d> G_normalized_incidence_rays;
  G_normalized_incidence_rays.reserve(backlinks.size());
  double distance_to_closest_observer = std::numeric_limits<double>::max();
  for (const vi_map::KeypointIdentifier& backlink : backlinks) {
    const vi_map::Vertex& vertex = map.getVertex(backlink.frame_id.vertex_id);

    const pose::Transformation T_G_C =
        map.getVertex_T_G_I(backlink.frame_id.vertex_id) *
        map.getMissionNCamera(vertex.getMissionId())
            .get_T_C_B(backlink.frame_id.frame_index)
            .inverse();

    const Eigen::Vector3d p_C_fi = map.getLandmark_p_C_fi(
        landmark.id(), vertex, backlink.frame_id.frame_index);
    const Eigen::Vector3d G_incidence_ray = T_G_C.getPosition() - p_G_fi;

    const double distance = G_incidence_ray.norm();
    distance_to_closest_observer =
        std::min(distance_to_closest_observer, distance);

    if (distance > 0) {
      G_normalized_incidence_rays.emplace_back(G_incidence_ray / distance);
    }
  }

  return isLidarLandmarkWellConstrained(
      G_normalized_incidence_rays, distance_to_closest_observer, settings);
}

}  // namespace vi_map
