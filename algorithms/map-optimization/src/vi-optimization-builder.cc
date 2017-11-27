#include "map-optimization/vi-optimization-builder.h"

#include <gflags/gflags.h>
#include <vi-map-helpers/mission-clustering-coobservation.h>

#include "map-optimization/optimization-state-fixing.h"

DEFINE_bool(
    ba_include_visual, true, "Whether or not to include visual error-terms.");
DEFINE_bool(
    ba_include_inertial, true, "Whether or not to include IMU error-terms.");

DEFINE_bool(
    ba_fix_ncamera_intrinsics, true,
    "Whether or not to fix the intrinsics of the ncamera(s).");
DEFINE_bool(
    ba_fix_ncamera_extrinsics_rotation, true,
    "Whether or not to fix the rotation extrinsics of the ncamera(s).");
DEFINE_bool(
    ba_fix_ncamera_extrinsics_translation, true,
    "Whether or not to fix the translation extrinsics of the ncamera(s).");
DEFINE_bool(
    ba_fix_landmark_positions, false,
    "Whether or not to fix the positions of the landmarks.");
DEFINE_bool(
    ba_fix_accel_bias, false,
    "Whether or not to fix the bias of the IMU accelerometer.");
DEFINE_bool(
    ba_fix_gyro_bias, false,
    "Whether or not to fix the bias of the IMU gyroscope.");
DEFINE_bool(
    ba_fix_velocity, false,
    "Whether or not to fix the velocity of the vertices.");

DEFINE_double(
    ba_latitude, common::locations::kLatitudeZurichDegrees,
    "Latitude to estimate the gravity magnitude.");
DEFINE_double(
    ba_altitude_meters, common::locations::kAltitudeZurichMeters,
    "Altitude in meters to estimate the gravity magnitude.");

DEFINE_int32(
    ba_min_landmark_per_frame, 0,
    "Minimum number of landmarks a frame must observe to be included in the "
    "problem.");

namespace map_optimization {

ViProblemOptions ViProblemOptions::initFromGFlags() {
  ViProblemOptions options;

  options.add_inertial_constraints = FLAGS_ba_include_inertial;
  options.fix_gyro_bias = FLAGS_ba_fix_gyro_bias;
  options.fix_accel_bias = FLAGS_ba_fix_accel_bias;
  options.fix_velocity = FLAGS_ba_fix_velocity;
  options.min_landmarks_per_frame = FLAGS_ba_min_landmark_per_frame;

  common::GravityProvider gravity_provider(
      FLAGS_ba_altitude_meters, FLAGS_ba_latitude);
  options.gravity_magnitude = gravity_provider.getGravityMagnitude();

  // Visual constraints.
  options.add_visual_constraints = FLAGS_ba_include_visual;
  options.fix_intrinsics = FLAGS_ba_fix_ncamera_intrinsics;
  options.fix_extrinsics_rotation = FLAGS_ba_fix_ncamera_extrinsics_rotation;
  options.fix_extrinsics_translation =
      FLAGS_ba_fix_ncamera_extrinsics_translation;
  options.fix_landmark_positions = FLAGS_ba_fix_landmark_positions;

  return options;
}

OptimizationProblem* constructViProblem(
    const vi_map::MissionIdSet& mission_ids, const ViProblemOptions& options,
    vi_map::VIMap* map) {
  CHECK(map);
  CHECK(options.isValid());

  LOG_IF(
      FATAL,
      !options.add_visual_constraints && !options.add_inertial_constraints)
      << "Either enable visual or inertial constraints; otherwise don't call "
      << "this function.";

  OptimizationProblem* problem = new OptimizationProblem(map, mission_ids);
  if (options.add_visual_constraints) {
    addVisualTerms(
        options.fix_landmark_positions, options.fix_intrinsics,
        options.fix_extrinsics_rotation, options.fix_extrinsics_translation,
        options.min_landmarks_per_frame, problem);
  }
  if (options.add_inertial_constraints) {
    addInertialTerms(
        options.fix_gyro_bias, options.fix_accel_bias, options.fix_velocity,
        options.gravity_magnitude, problem);
  }

  // Fixing open DoF of the visual(-inertial) problem. We assume that if there
  // is inertial data, that all missions will have them.
  const bool visual_only =
      options.add_visual_constraints && !options.add_inertial_constraints;

  // Determine and apply the gauge fixes.
  MissionClusterGaugeFixes fixes_of_mission_cluster;
  if (!visual_only) {
    fixes_of_mission_cluster.position_dof_fixed = true;
    fixes_of_mission_cluster.rotation_dof_fixed = FixedRotationDoF::kYaw;
    fixes_of_mission_cluster.scale_fixed = false;
  } else {
    fixes_of_mission_cluster.position_dof_fixed = true;
    fixes_of_mission_cluster.rotation_dof_fixed = FixedRotationDoF::kAll;
    fixes_of_mission_cluster.scale_fixed = true;
  }
  const size_t num_clusters = problem->getMissionCoobservationClusters().size();
  std::vector<MissionClusterGaugeFixes> vi_cluster_fixes(
      num_clusters, fixes_of_mission_cluster);

  // Merge with already applied fixes (if necessary).
  const std::vector<MissionClusterGaugeFixes>* already_applied_cluster_fixes =
      problem->getAppliedGaugeFixesForInitialVertices();
  if (already_applied_cluster_fixes) {
    std::vector<MissionClusterGaugeFixes> merged_fixes;
    mergeGaugeFixes(
        vi_cluster_fixes, *already_applied_cluster_fixes, &merged_fixes);
    problem->applyGaugeFixesForInitialVertices(merged_fixes);
  } else {
    problem->applyGaugeFixesForInitialVertices(vi_cluster_fixes);
  }

  // Baseframes are fixed in the non mission-alignment problems.
  fixAllBaseframesInProblem(problem);

  return problem;
}

}  // namespace map_optimization
