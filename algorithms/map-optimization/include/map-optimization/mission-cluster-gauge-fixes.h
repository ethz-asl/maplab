#ifndef MAP_OPTIMIZATION_MISSION_CLUSTER_GAUGE_FIXES_H_
#define MAP_OPTIMIZATION_MISSION_CLUSTER_GAUGE_FIXES_H_

#include <algorithm>
#include <vector>

#include <vi-map/unique-id.h>

namespace map_optimization {
// Ordered by increasing open DoF.
enum class FixedRotationDoF : size_t { kNone = 0u, kYaw = 1u, kAll = 2u };

struct MissionClusterGaugeFixes {
  // Fix position in first vertex in first mission of cluster.
  bool position_dof_fixed = true;
  // Fix rotation in first vertex in first mission of cluster.
  FixedRotationDoF rotation_dof_fixed = FixedRotationDoF::kAll;
  // Fix scale by fixing the landmark (expressed in a vertex frame) observed
  // by the first vertex of the first mission in the cluster.
  bool scale_fixed;

  // Merge fixes from two subproblems. Less fixes have priority.
  void merge(const MissionClusterGaugeFixes& other) {
    // Unfix position, if other has it unfixed.
    if (position_dof_fixed && !other.position_dof_fixed) {
      position_dof_fixed = false;
    }

    // FixedRotationDoF::kNone > FixedRotationDoF::kYaw >
    // FixedRotationDoF::kAll.
    rotation_dof_fixed = static_cast<FixedRotationDoF>(
        std::min(
            static_cast<size_t>(rotation_dof_fixed),
            static_cast<size_t>(other.rotation_dof_fixed)));

    // Release scale, if one has it unfixed.
    if (scale_fixed || !other.scale_fixed) {
      scale_fixed = false;
    }
  }
};

inline void mergeGaugeFixes(
    const std::vector<MissionClusterGaugeFixes>& fixes_A,
    const std::vector<MissionClusterGaugeFixes>& fixes_B,
    std::vector<MissionClusterGaugeFixes>* merged_fixes_AB) {
  CHECK_NOTNULL(merged_fixes_AB)->clear();
  CHECK(!fixes_A.empty());
  CHECK_EQ(fixes_A.size(), fixes_B.size());

  const size_t num_clusters = fixes_A.size();
  for (size_t cluster_idx = 0u; cluster_idx < num_clusters; ++cluster_idx) {
    MissionClusterGaugeFixes merged_AB = fixes_A[cluster_idx];
    merged_AB.merge(fixes_B[cluster_idx]);
    merged_fixes_AB->emplace_back(merged_AB);
  }
  CHECK_EQ(merged_fixes_AB->size(), num_clusters);
}

}  // namespace map_optimization
#include "map-optimization/mission-cluster-gauge-fixes-inl.h"
#endif  // MAP_OPTIMIZATION_MISSION_CLUSTER_GAUGE_FIXES_H_
