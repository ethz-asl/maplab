#include "vi-map/landmark-quality-metrics.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/statistics/statistics.h>
#include <gflags/gflags.h>
#include <vi-map/landmark.h>

DEFINE_double(
    elq_min_observation_angle_deg, 5,
    "Minimum angle disparity of observers for a landmark to be "
    "well constrained.");

DEFINE_uint64(
    elq_min_observers, 4,
    "Minimum number of observers for a landmark to be "
    "well constrained.");

DEFINE_double(
    elq_max_distance_from_closest_observer, 40,
    "Maximum distance from closest observer for a landmark to be "
    "well constrained [m].");

DEFINE_double(
    elq_min_distance_from_closest_observer, 0.05,
    "Minimum distance from closest observer for a landmark to be "
    "well constrained [m].");

DEFINE_double(
    elq_max_reprojection_error_px, -1,
    "Maximum reporjection error of any landmark observation [px]. When "
    "negative this check will be ignored.");

namespace vi_map {
LandmarkWellConstrainedSettings::LandmarkWellConstrainedSettings()
    : max_distance_to_closest_observer(
          FLAGS_elq_max_distance_from_closest_observer),
      min_distance_to_closest_observer(
          FLAGS_elq_min_distance_from_closest_observer),
      min_observation_angle_deg(FLAGS_elq_min_observation_angle_deg),
      max_reprojection_error_px(FLAGS_elq_max_reprojection_error_px),
      min_observers(FLAGS_elq_min_observers) {}

double computeSquaredReprojectionError(
    const vi_map::Vertex& vertex, const int frame_idx, const int keypoint_idx,
    const Eigen::Vector3d& landmark_p_C) {
  Eigen::Vector2d reprojected_point;
  aslam::ProjectionResult projection_result =
      vertex.getCamera(frame_idx)->project3(landmark_p_C, &reprojected_point);

  if (projection_result == aslam::ProjectionResult::KEYPOINT_VISIBLE ||
      projection_result ==
          aslam::ProjectionResult::KEYPOINT_OUTSIDE_IMAGE_BOX) {
    return (reprojected_point -
            vertex.getVisualFrame(frame_idx).getKeypointMeasurement(
                keypoint_idx))
        .squaredNorm();
  }
  return std::numeric_limits<double>::max();
}

bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark) {
  constexpr bool kReEvaluateQuality = false;
  return isLandmarkWellConstrained(map, landmark, kReEvaluateQuality);
}

bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    bool re_evaluate_quality) {
  vi_map::Landmark::Quality quality = landmark.getQuality();
  // Localization landmarks are always good.
  if (quality == vi_map::Landmark::Quality::kLocalizationSummaryLandmark) {
    return true;
  }
  if (re_evaluate_quality) {
    quality = vi_map::Landmark::Quality::kUnknown;
  }

  if (quality != vi_map::Landmark::Quality::kUnknown) {
    return quality == vi_map::Landmark::Quality::kGood;
  }

  // Use default settings.
  LandmarkWellConstrainedSettings settings;

  const vi_map::KeypointIdentifierList& backlinks = landmark.getObservations();
  if (backlinks.size() < settings.min_observers) {
    return false;
  }

  const double max_reprojection_error_px_sq =
      settings.max_reprojection_error_px * settings.max_reprojection_error_px;

  const Eigen::Vector3d& p_G_fi = map.getLandmark_G_p_fi(landmark.id());
  Aligned<std::vector, Eigen::Vector3d> G_normalized_incidence_rays;
  G_normalized_incidence_rays.reserve(backlinks.size());
  double signed_distance_to_closest_observer =
      std::numeric_limits<double>::max();
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
    const double signed_distance = distance * (p_C_fi(2) < 0.0 ? -1.0 : 1.0);
    signed_distance_to_closest_observer =
        std::min(signed_distance_to_closest_observer, signed_distance);

    if (distance > 0) {
      G_normalized_incidence_rays.emplace_back(G_incidence_ray / distance);
    }

    if (settings.max_reprojection_error_px > 0.0) {
      const double reprojection_error_px_sq = computeSquaredReprojectionError(
          vertex, backlink.frame_id.frame_index, backlink.keypoint_index,
          p_C_fi);
      if (reprojection_error_px_sq > max_reprojection_error_px_sq) {
        return false;
      }
    }
  }

  if (signed_distance_to_closest_observer >
          settings.max_distance_to_closest_observer ||
      signed_distance_to_closest_observer <
          settings.min_distance_to_closest_observer) {
    return false;
  }

  const double max_disparity_angle_rad =
      common::getMaxDisparityRadAngleOfUnitVectorBundle(
          G_normalized_incidence_rays);

  constexpr double kRadToDeg = 180.0 / M_PI;
  double angle_deg = max_disparity_angle_rad * kRadToDeg;
  if (angle_deg < settings.min_observation_angle_deg) {
    return false;
  }

  return true;
}
}  // namespace vi_map
