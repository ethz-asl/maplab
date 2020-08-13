#include "map-optimization/vi-optimization-builder.h"

#include <gflags/gflags.h>
#include <vi-map-helpers/mission-clustering-coobservation.h>

#include "map-optimization/augment-loopclosure.h"
#include "map-optimization/optimization-state-fixing.h"

DEFINE_bool(
    ba_include_visual, true, "Whether or not to include visual error-terms.");
DEFINE_bool(
    ba_include_lidar, false,
    "Whether or not to include error-terms from lidar landmarks.");
DEFINE_bool(
    ba_use_visual_outlier_rejection_solver, true,
    "Reject outlier landmarks during the solve?");
DEFINE_bool(
    ba_include_inertial, true, "Whether or not to include IMU error-terms.");
DEFINE_bool(
    ba_include_wheel_odometry, false,
    "Whether or not to include wheel (relative pose) error-terms.");

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
    ba_fix_wheel_odometry_extrinsics, true,
    "Whether or not to fix the extrinsics of the wheel odometry sensor.");
DEFINE_bool(
    ba_fix_vertices, false,
    "Whether or not to fix vertex poses in optimization.");
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

DEFINE_bool(
    ba_include_6dof_odometry, false,
    "Whether or not to include 6DoF odometry constraints (T_St_Stp1) for an "
    "odometry sensor frame S.");

DEFINE_bool(
    ba_fix_6dof_odometry_extrinsics, true,
    "Whether or not to fix the extrinsics of the 6DoF odometry sensor.");

DEFINE_bool(
    ba_include_absolute_pose_constraints, false,
    "Whether or not to include absolute 6DoF pose constraints (T_G_B), e.g. "
    "GPS or AprilTag detections.");

DEFINE_bool(
    ba_fix_absolute_pose_sensor_extrinsics, true,
    "Whether or not to fix the extrinsics of the absolute 6DoF pose "
    "constraints sensor (T_B_S), e.g. GPS to IMU calibration. This flag will "
    "only take effect if absolute pose constraints are enabled!");

DEFINE_bool(
    ba_absolute_pose_sensor_fix_mission_baseframes, true,
    "Whether or not to fix the mission baseframes (T_G_M) during "
    "optimization. This flag will only take effect if absolute pose "
    "constraints are enabled!");

DEFINE_bool(
    ba_include_loop_closure_edges, false,
    "Whether or not to add the loop closure edges present in the map to the "
    "optimization problem. The visual loop closure does not use these edges by "
    "default, but merges the landmarks (i.e. loop closures are part of the "
    "visual error terms). Pose graph relaxation on the other hand adds visual "
    "loop closures as edges by default.");

namespace map_optimization {
ViProblemOptions ViProblemOptions::initFromGFlags() {
  ViProblemOptions options;

  // Vertex constraints
  options.fix_vertices = FLAGS_ba_fix_vertices;

  // Inertial constraints
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

  // Lidar constraints
  options.add_lidar_constraints = FLAGS_ba_include_lidar;

  // Wheel odometry constraints
  options.add_wheel_odometry_constraints = FLAGS_ba_include_wheel_odometry;
  options.fix_wheel_extrinsics = FLAGS_ba_fix_wheel_odometry_extrinsics;

  // 6DoF odometry constraints.
  options.add_6dof_odometry_constraints = FLAGS_ba_include_6dof_odometry;
  options.fix_6dof_odometry_extrinsics = FLAGS_ba_fix_6dof_odometry_extrinsics;

  // Absolute 6DoF pose constraints
  options.add_absolute_pose_constraints =
      FLAGS_ba_include_absolute_pose_constraints;
  options.fix_absolute_pose_sensor_extrinsics =
      FLAGS_ba_fix_absolute_pose_sensor_extrinsics;
  options.fix_baseframes = FLAGS_ba_absolute_pose_sensor_fix_mission_baseframes;

  // Loop closure constraints (can be from an external source)
  options.add_loop_closure_edges = FLAGS_ba_include_loop_closure_edges;

  options.solver_options = initSolverOptionsFromFlags();

  options.enable_visual_outlier_rejection =
      FLAGS_ba_use_visual_outlier_rejection_solver;
  options.visual_outlier_rejection_options =
      map_optimization::OutlierRejectionSolverOptions::initFromFlags();

  options.printToConsole();

  return options;
}

OptimizationProblem* constructOptimizationProblem(
    const vi_map::MissionIdSet& mission_ids, const ViProblemOptions& options,
    vi_map::VIMap* map) {
  CHECK(map);
  CHECK(options.isValid());

  OptimizationProblem* problem = new OptimizationProblem(map, mission_ids);
  size_t num_visual_constraints_added = 0u;
  if (options.add_visual_constraints || options.add_lidar_constraints) {
    num_visual_constraints_added = addVisualTerms(
        options.fix_landmark_positions, options.fix_intrinsics,
        options.fix_extrinsics_rotation, options.fix_extrinsics_translation,
        options.add_visual_constraints, options.add_lidar_constraints,
        options.min_landmarks_per_frame, problem);
    if (num_visual_constraints_added == 0u) {
      LOG(WARNING)
          << "WARNING: Visual constraints enabled, but none "
          << "were found, adapting DoF settings of optimization problem...";
    }
  }
  size_t num_inertial_constraints_added = 0u;
  if (options.add_inertial_constraints) {
    num_inertial_constraints_added = addInertialTerms(
        options.fix_gyro_bias, options.fix_accel_bias, options.fix_velocity,
        options.gravity_magnitude, problem);
    if (num_inertial_constraints_added == 0u) {
      LOG(WARNING)
          << "WARNING: Inertial constraints enabled, but none "
          << "were found, adapting DoF settings of optimization problem...";
    }
  }

  size_t num_wheel_odometry_constraints_added = 0u;
  if (options.add_wheel_odometry_constraints) {
    num_wheel_odometry_constraints_added =
        addWheelOdometryTerms(options.fix_wheel_extrinsics, problem);
    if (num_wheel_odometry_constraints_added == 0u) {
      LOG(WARNING)
          << "WARNING: Wheel odometry constraints enabled, but none "
          << "were found, adapting DoF settings of optimization problem...";
    }
  }

  size_t num_6dof_odometry_constraints_added = 0u;
  if (options.add_6dof_odometry_constraints) {
    num_6dof_odometry_constraints_added =
        add6DoFOdometryTerms(options.fix_6dof_odometry_extrinsics, problem);
    if (num_6dof_odometry_constraints_added == 0u) {
      LOG(WARNING)
          << "WARNING: 6DoF odometry constraints enabled, but none "
          << "were found, adapting DoF settings of optimization problem...";
    }
  }

  size_t num_absolute_6dof_constraints_added = 0u;
  if (options.add_absolute_pose_constraints) {
    num_absolute_6dof_constraints_added = addAbsolutePoseConstraintsTerms(
        options.fix_absolute_pose_sensor_extrinsics, problem);
    if (num_absolute_6dof_constraints_added == 0u) {
      LOG(WARNING)
          << "WARNING: Absolute 6DoF constraints enabled, but none "
          << "were found, adapting DoF settings of optimization problem...";
    }
  }

  size_t num_lc_edges = 0u;
  if (options.add_loop_closure_edges) {
    num_lc_edges = numLoopclosureEdges(*map);
    if (num_lc_edges == 0u) {
      LOG(WARNING) << "WARNING: Loop closure edges are enabled, but none "
                   << "were found.";
    } else {
      size_t actually_added_lc_error_terms =
          augmentOptimizationProblemWithLoopclosureEdges(problem);
      CHECK(actually_added_lc_error_terms == num_lc_edges)
          << "The pose graph has " << num_lc_edges << "loop closure edges, but "
          << actually_added_lc_error_terms
          << "were added to the optimization problem!";
    }
  }

  if (options.fix_vertices) {
    LOG(INFO) << "Fixing vertex positions.";
    fixAllVerticesInProblem(problem);
  }

  // We analyze the available constraints in each mission clusters, i.e.
  // missions that are connected through either visual constraints or loop
  // closure edges, and determine the gauge fixes.
  const std::vector<vi_map::MissionIdSet>& mission_clusters =
      problem->getMissionCoobservationClusters();
  const size_t num_clusters = mission_clusters.size();
  std::vector<MissionClusterGaugeFixes> mission_cluster_gauge_fixes(
      num_clusters);
  CHECK_EQ(mission_clusters.size(), mission_cluster_gauge_fixes.size());

  for (size_t cluster_idx = 0u; cluster_idx < num_clusters; ++cluster_idx) {
    MissionClusterGaugeFixes& mission_cluster_gauge_fix =
        mission_cluster_gauge_fixes[cluster_idx];
    const vi_map::MissionIdSet& mission_cluster = mission_clusters[cluster_idx];

    const size_t cluster_num_absolute_6dof_present =
        vi_map_helpers::getNumAbsolute6DoFConstraintsForMissionCluster(
            *map, mission_cluster);
    const size_t cluster_num_absolute_6dof_used = std::min(
        cluster_num_absolute_6dof_present, num_absolute_6dof_constraints_added);
    const bool cluster_has_inertial =
        vi_map_helpers::hasInertialConstraintsInAllMissionsInCluster(
            *map, mission_cluster) &&
        (num_inertial_constraints_added > 0u);
    const bool cluster_has_visual =
        vi_map_helpers::hasVisualConstraintsInAllMissionsInCluster(
            *map, mission_cluster) &&
        (num_visual_constraints_added > 0u);
    const bool cluster_has_wheel_odometry =
        vi_map_helpers::hasWheelOdometryConstraintsInAllMissionsInCluster(
            *map, mission_cluster) &&
        (num_wheel_odometry_constraints_added > 0u);
    const bool cluster_has_6dof_odometry =
        vi_map_helpers::has6DoFOdometryConstraintsInAllMissionsInCluster(
            *map, mission_cluster) &&
        (num_6dof_odometry_constraints_added > 0u);

    // Note that if there are lc edges they always are within the cluster,
    // because otherwise the other mission would have been part of the cluster.
    const bool cluster_has_lc_edges =
        vi_map_helpers::hasLcEdgesInMissionCluster(*map, mission_cluster) &&
        (num_lc_edges > 0u);

    CHECK(
        cluster_has_inertial || cluster_has_visual ||
        cluster_has_wheel_odometry || cluster_has_6dof_odometry)
        << "Either inertial, visual or wheel odometry constraints need to be "
           "available to form a stable graph.";

    // Determine observability of scale, global position and global orientation.
    const bool scale_is_observable =
        cluster_has_inertial || cluster_has_wheel_odometry ||
        cluster_has_6dof_odometry ||
        (cluster_has_visual && cluster_num_absolute_6dof_used > 1u);

    const bool global_position_is_observable =
        cluster_num_absolute_6dof_used > 0u;

    const bool global_yaw_is_observable = cluster_num_absolute_6dof_used > 0u;

    const bool global_roll_pitch_is_observable =
        cluster_num_absolute_6dof_used > 0u || cluster_has_inertial;

    std::stringstream ss;
    ss << "\nMission Cluster: ";
    for (const vi_map::MissionId& mission_id : mission_cluster) {
      ss << "\n\t" << mission_id;
    }

    ss << "\n\nConstraints:";
    ss << "\n\tInertial constraints:\t\t"
       << ((cluster_has_inertial) ? "on" : "off");
    ss << "\n\tVisual constraints:\t\t"
       << ((cluster_has_visual) ? "on" : "off");
    ss << "\n\tWheel odometry constraints:\t"
       << ((cluster_has_wheel_odometry) ? "on" : "off");
    ss << "\n\t6DoF odometry constraints:\t"
       << ((cluster_has_6dof_odometry) ? "on" : "off");
    ss << "\n\tAbsolute 6DoF constraints:\t"
       << ((cluster_num_absolute_6dof_used > 0) ? "on" : "off");
    ss << "\n\tLoop closure edge constraints:\t"
       << ((cluster_has_lc_edges > 0) ? "on" : "off");

    ss << "\n\nIs observable:";
    ss << "\n\tScale: " << ((scale_is_observable) ? "yes" : "no");
    ss << "\n\tGlobal position: "
       << ((global_position_is_observable) ? "yes" : "no");
    ss << "\n\tGlobal yaw: " << ((global_yaw_is_observable) ? "yes" : "no");
    ss << "\n\tGlobal roll/pitch: "
       << ((global_roll_pitch_is_observable) ? "yes" : "no");
    ss << "\n";
    VLOG(1) << ss.str();

    // Apply gauge fixes for this cluster.
    mission_cluster_gauge_fix.position_dof_fixed =
        !global_position_is_observable;
    mission_cluster_gauge_fix.scale_fixed = !scale_is_observable;

    // Currently there is no scenario where yaw is observable but roll and
    // pitch, this could change if magnetometer are added. This check should
    // make sure that the logic below does not do weird stuff it add such a
    // scenario, but forget to change code here.
    CHECK(!(global_yaw_is_observable && !global_roll_pitch_is_observable));

    mission_cluster_gauge_fix.rotation_dof_fixed =
        global_roll_pitch_is_observable
            ? (global_yaw_is_observable ? FixedRotationDoF::kNone
                                        : FixedRotationDoF::kYaw)
            : FixedRotationDoF::kAll;
  }

  // Merge with already applied fixes (if necessary).
  const std::vector<MissionClusterGaugeFixes>* already_applied_cluster_fixes =
      problem->getAppliedGaugeFixesForInitialVertices();
  if (already_applied_cluster_fixes) {
    std::vector<MissionClusterGaugeFixes> merged_fixes;
    mergeGaugeFixes(
        mission_cluster_gauge_fixes, *already_applied_cluster_fixes,
        &merged_fixes);
    problem->applyGaugeFixesForInitialVertices(merged_fixes);
  } else {
    problem->applyGaugeFixesForInitialVertices(mission_cluster_gauge_fixes);
  }

  // Only case were we do NOT fix the baseframe is if there are absolute 6dof
  // constraints and the option to fix them has been disabled.
  if (num_absolute_6dof_constraints_added == 0u || options.fix_baseframes) {
    fixAllBaseframesInProblem(problem);
    VLOG(1) << "Baseframes fixed: yes";
  } else {
    VLOG(1) << "Baseframes fixed: no";
  }

  return problem;
}

}  // namespace map_optimization
