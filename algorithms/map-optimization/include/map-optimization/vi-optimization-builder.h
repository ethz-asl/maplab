#ifndef MAP_OPTIMIZATION_VI_OPTIMIZATION_BUILDER_H_
#define MAP_OPTIMIZATION_VI_OPTIMIZATION_BUILDER_H_

#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <maplab-common/gravity-provider.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/vi-map.h>

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

  bool isValid() const {
    return (gravity_magnitude > 0.0);
  }

 protected:
  ViProblemOptions() = default;
};

// Caller takes ownership.
OptimizationProblem* constructViProblem(
    const vi_map::MissionIdSet& mission_ids, const ViProblemOptions& options,
    vi_map::VIMap* map);

}  // namespace map_optimization
#endif  // MAP_OPTIMIZATION_VI_OPTIMIZATION_BUILDER_H_
