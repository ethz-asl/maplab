#ifndef MAP_OPTIMIZATION_VI_OPTIMIZATION_BUILDER_H_
#define MAP_OPTIMIZATION_VI_OPTIMIZATION_BUILDER_H_

#include <maplab-common/gravity-provider.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/vi-map.h>

#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "map-optimization/optimization-problem.h"
#include "map-optimization/optimization-terms-addition.h"

namespace map_optimization {

struct ViProblemOptions {
  static ViProblemOptions initFromGFlags();

  // Inertial constraints.
  bool add_inertial_constraints;
  bool fix_gyro_bias;
  bool fix_accel_bias;
  bool fix_velocity;
  size_t min_landmarks_per_frame;
  double gravity_magnitude = std::numeric_limits<double>::quiet_NaN();

  // Visual constraints.
  bool add_visual_constraints;
  bool fix_intrinsics;
  bool fix_extrinsics_rotation;
  bool fix_extrinsics_translation;
  bool fix_landmark_positions;

  // Wheel constraints
  bool add_wheel_odometry_constraints;
  bool fix_wheel_extrinsics;
  bool fix_vertices;

  // Absolute pose constraints
  bool add_absolute_pose_constraints;
  bool fix_absolute_pose_sensor_extrinsics;
  bool fix_baseframes;

  bool add_loop_closure_edges;

  bool isValid() const {
    return (gravity_magnitude > 0.0);
  }

  void printToConsole(const int verbosity_level = 3) {
    VLOG(verbosity_level)
        << "Setting up optimization problem using the following options.";
    VLOG(verbosity_level) << "add_inertial_constraints:            "
                          << add_inertial_constraints;
    VLOG(verbosity_level) << "fix_gyro_bias:                       "
                          << fix_gyro_bias;
    VLOG(verbosity_level) << "fix_accel_bias:                      "
                          << fix_accel_bias;
    VLOG(verbosity_level) << "fix_velocity:                        "
                          << fix_velocity;
    VLOG(verbosity_level) << "min_landmarks_per_frame:             "
                          << min_landmarks_per_frame;
    VLOG(verbosity_level) << "gravity_magnitude:                   "
                          << gravity_magnitude;
    VLOG(verbosity_level) << "add_visual_constraints:              "
                          << add_visual_constraints;
    VLOG(verbosity_level) << "fix_intrinsics:                      "
                          << fix_intrinsics;
    VLOG(verbosity_level) << "fix_extrinsics_rotation:             "
                          << fix_extrinsics_rotation;
    VLOG(verbosity_level) << "fix_extrinsics_translation:          "
                          << fix_extrinsics_translation;
    VLOG(verbosity_level) << "fix_landmark_positions:              "
                          << fix_landmark_positions;
    VLOG(verbosity_level) << "add_absolute_pose_constraints:       "
                          << add_absolute_pose_constraints;
    VLOG(verbosity_level) << "fix_absolute_pose_sensor_extrinsics: "
                          << fix_absolute_pose_sensor_extrinsics;
    VLOG(verbosity_level) << "fix_baseframes:                      "
                          << fix_baseframes;
    VLOG(verbosity_level) << "add_loop_closure_edges:              "
                          << add_loop_closure_edges;
  }

 protected:
  ViProblemOptions() = default;
};

// Caller takes ownership.
OptimizationProblem* constructOptimizationProblem(
    const vi_map::MissionIdSet& mission_ids, const ViProblemOptions& options,
    vi_map::VIMap* map);

}  // namespace map_optimization
#endif  // MAP_OPTIMIZATION_VI_OPTIMIZATION_BUILDER_H_
